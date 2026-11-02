/*
 * QEMU SDL display driver
 *
 * Copyright (c) 2003 Fabrice Bellard
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
/* Ported SDL 1.2 code to 2.0 by Dave Airlie. */

#include "qemu/osdep.h"
#include "qemu/module.h"
#include "qemu/cutils.h"
#include "ui/console.h"
#include "ui/input.h"
#include "ui/sdl2.h"
#include "system/runstate.h"
#include "system/runstate-action.h"
#include "system/system.h"
#include "qemu/log.h"
#include "qemu-main.h"

#if defined(CONFIG_DARWIN) || defined(CONFIG_WIN32)
void spacebox_scroll_tick(SDL_Window *window);
#endif
#ifdef CONFIG_WIN32
/* ui/spacebox-scroll-win.c: true when the wheel event went to the guest's
 * scroll agent */
bool spacebox_scroll_wheel(const SDL_MouseWheelEvent *wev);
#ifdef CONFIG_OPENGL
#include <epoxy/wgl.h>
#endif
#endif

static int sdl2_num_outputs;
static struct sdl2_console *sdl2_console;

static SDL_Surface *guest_sprite_surface;
static int gui_grab; /* if true, all keyboard/mouse events are grabbed */
static bool alt_grab;
static bool ctrl_grab;

static int gui_saved_grab;
static int gui_fullscreen;
static int gui_grab_code = KMOD_LALT | KMOD_LCTRL;
static SDL_Cursor *sdl_cursor_normal;
static SDL_Cursor *sdl_cursor_hidden;
static int absolute_enabled;
static bool guest_cursor;
static int guest_x, guest_y;
static SDL_Cursor *guest_sprite;
static Notifier mouse_mode_notifier;

#define SDL2_REFRESH_INTERVAL_BUSY 10
#define SDL2_MAX_IDLE_COUNT (2 * GUI_REFRESH_INTERVAL_DEFAULT \
                             / SDL2_REFRESH_INTERVAL_BUSY + 1)

/* introduced in SDL 2.0.10 */
#ifndef SDL_HINT_RENDER_BATCHING
#define SDL_HINT_RENDER_BATCHING "SDL_RENDER_BATCHING"
#endif

static void sdl_update_caption(struct sdl2_console *scon);

static struct sdl2_console *get_scon_from_window(uint32_t window_id)
{
    int i;
    for (i = 0; i < sdl2_num_outputs; i++) {
        if (sdl2_console[i].real_window == SDL_GetWindowFromID(window_id)) {
            return &sdl2_console[i];
        }
    }
    return NULL;
}

/* Opt-in 2x client window for the Spacebox native-Mac test build. */
static int spacebox_pixel_scale(void)
{
#ifdef CONFIG_DARWIN
    const char *scale = getenv("SPACEBOX_RETINA_SCALE");
    return scale && !strcmp(scale, "2") ? 2 : 1;
#else
    return 1;
#endif
}

#ifdef CONFIG_WIN32
/*
 * Windows: with SPACEBOX_RETINA_SCALE set (any value) the process is per-
 * monitor DPI aware and SDL scales by the monitor's DPI, as macOS does for a
 * Retina display: the window's size is in points (pixels at 96 per inch),
 * its drawable in physical pixels; at 200% and above the guest gets the
 * drawable's size, below 200% what the Mac's sink gives it on a display
 * without Retina, twice the points (spacebox_update_ui_info).
 * The scale is the drawable's width over the window's (1.5 at 150%), read
 * again at each change of the window; before there is a window, the scale of
 * the main display.
 */
static double spacebox_win_scale = 1;
/* the keyboard is grabbed (spacebox_win_kbd_grab) */
static bool spacebox_kbd_grab, spacebox_kbd_saved;

/*
 * The size last told to the guest (spacebox_update_ui_info) and when, in ms
 * of the monotonic clock. A guest that is told a new size may first show
 * another mode for a moment: GNOME's compositor in the task's guest showed a
 * mode it had held earlier (3840 x 2160, its full screen at 100%) for about
 * 190 ms before the told one (turn G6e). QEMU's window follows each change of
 * the guest's surface (sdl2_window_resize) and the window's new size is told
 * to the guest, so the window took that mode's size in points and the guest
 * was told it back: after a resize to 1280 x 800 points at 125% the window
 * ended at 1920 x 1080 points, larger than the display. So the window does
 * not follow a surface of another size than the one told while the guest may
 * still be on its way to it (SPACEBOX_WIN_FOLLOW_MS after the tell); later
 * changes of the guest's own (a mode chosen in the guest) are followed as
 * before, and Ctrl+Alt+U gives the window the guest's size at any time.
 */
static int spacebox_win_told_w, spacebox_win_told_h;
static int64_t spacebox_win_told_ms;
#define SPACEBOX_WIN_FOLLOW_MS 2000

static bool spacebox_win_dpi(void)
{
    return getenv("SPACEBOX_RETINA_SCALE") != NULL;
}

/*
 * The refresh of a monitor in mHz, as its active mode has it (a fraction:
 * 59.94 Hz is 59940/1000 on BF2's display), from Windows' display
 * configuration; 0 when not known. SDL gives whole Hertz, and Windows' 59 for
 * 59.94: told 59 Hz, the guest's compositor drew at 59 a second and a video of
 * 60 frames a second lost about one frame each second (turn G5a). The Mac's
 * sink tells the guest its display's own refresh in the same way.
 */
unsigned sdl2_win_refresh_mhz(void *monitor)
{
    MONITORINFOEXW mi = { .cbSize = sizeof(mi) };
    UINT32 np = 0, nm = 0;
    DISPLAYCONFIG_PATH_INFO *paths;
    DISPLAYCONFIG_MODE_INFO *modes;
    unsigned mhz = 0;

    if (!monitor || !GetMonitorInfoW(monitor, (MONITORINFO *)&mi) ||
        GetDisplayConfigBufferSizes(QDC_ONLY_ACTIVE_PATHS, &np, &nm) != ERROR_SUCCESS) {
        return 0;
    }
    paths = g_new0(DISPLAYCONFIG_PATH_INFO, np);
    modes = g_new0(DISPLAYCONFIG_MODE_INFO, nm);
    if (QueryDisplayConfig(QDC_ONLY_ACTIVE_PATHS, &np, paths, &nm, modes, NULL) == ERROR_SUCCESS) {
        for (UINT32 i = 0; i < np && !mhz; i++) {
            DISPLAYCONFIG_SOURCE_DEVICE_NAME src = {
                .header = {
                    .type = DISPLAYCONFIG_DEVICE_INFO_GET_SOURCE_NAME,
                    .size = sizeof(src),
                    .adapterId = paths[i].sourceInfo.adapterId,
                    .id = paths[i].sourceInfo.id,
                },
            };
            DISPLAYCONFIG_RATIONAL r = paths[i].targetInfo.refreshRate;

            if (DisplayConfigGetDeviceInfo(&src.header) == ERROR_SUCCESS &&
                !wcscmp(src.viewGdiDeviceName, mi.szDevice) && r.Denominator) {
                mhz = (unsigned)(((uint64_t)r.Numerator * 1000 + r.Denominator / 2) / r.Denominator);
            }
        }
    }
    g_free(paths);
    g_free(modes);
    return mhz;
}

static unsigned spacebox_win_window_refresh_mhz(SDL_Window *window)
{
    SDL_SysWMinfo wm;

    SDL_VERSION(&wm.version);
    if (!SDL_GetWindowWMInfo(window, &wm) || wm.subsystem != SDL_SYSWM_WINDOWS) {
        return 0;
    }
    return sdl2_win_refresh_mhz(MonitorFromWindow(wm.info.win.window, MONITOR_DEFAULTTONEAREST));
}

/* the guest's pixels for a point: the Mac's 2 up to a scale of 200%, the
 * monitor's scale above it (spacebox_update_ui_info) */
static double spacebox_win_guest_scale(void)
{
    return spacebox_win_scale > 1.99 ? spacebox_win_scale : 2.0;
}

static int spacebox_points(int pixels)
{
    return (int)lround(pixels / (spacebox_win_dpi() ? spacebox_win_guest_scale() : 1.0));
}

/*
 * Keys of a Windows keyboard that SDL drops or Windows takes:
 * - Print Screen: Windows 11 opens its screen capture with it, and SDL's own
 *   hook of a grabbed keyboard (the Windows keys, Alt, Ctrl, Tab, Esc) does
 *   not take it; while the keyboard is grabbed this hook does, for the guest.
 * - Han/Eng and Hanja of a Korean keyboard (scan codes 0x72 and 0x71, 0xf2
 *   and 0xf1 with the break bit): SDL 2 has no scan code for them and drops
 *   them; they become SDL's LANG1 and LANG2 (QEMU's lang1 and lang2, the
 *   guest's KEY_HANGEUL and KEY_HANJA). The keys have no break code, so each
 *   press is a press and a release at once, as Linux's own keyboard driver
 *   makes them. (A layout that makes the right Alt and Ctrl these keys gives
 *   the scan codes of Alt and Ctrl, which SDL has: the guest gets those keys.)
 */
static HHOOK spacebox_print_hook_handle;

static void spacebox_win_key(SDL_Scancode code, bool down)
{
    SDL_Event ev = {0};
    SDL_Window *focus = SDL_GetKeyboardFocus();

    ev.type = down ? SDL_KEYDOWN : SDL_KEYUP;
    ev.key.windowID = focus ? SDL_GetWindowID(focus) : 0;
    ev.key.state = down ? SDL_PRESSED : SDL_RELEASED;
    ev.key.keysym.scancode = code;
    ev.key.keysym.sym = SDL_GetKeyFromScancode(code);
    SDL_PushEvent(&ev);
}

static LRESULT CALLBACK spacebox_print_proc(int code, WPARAM wp, LPARAM lp)
{
    KBDLLHOOKSTRUCT *k = (KBDLLHOOKSTRUCT *)lp;

    if (code == HC_ACTION && k->vkCode == VK_SNAPSHOT && SDL_GetKeyboardFocus()) {
        spacebox_win_key(SDL_SCANCODE_PRINTSCREEN, wp == WM_KEYDOWN || wp == WM_SYSKEYDOWN);
        return 1;
    }
    return CallNextHookEx(NULL, code, wp, lp);
}

static void spacebox_win_print_hook(bool on)
{
    if (on && !spacebox_print_hook_handle) {
        spacebox_print_hook_handle = SetWindowsHookExW(WH_KEYBOARD_LL, spacebox_print_proc,
                                                       GetModuleHandleW(NULL), 0);
    } else if (!on && spacebox_print_hook_handle) {
        UnhookWindowsHookEx(spacebox_print_hook_handle);
        spacebox_print_hook_handle = NULL;
    }
}

/* hw/display/virtio-gpu-remote.c: the stop script's request (it detaches first). */
bool spacebox_sink_stop_request(uint64_t id, int64_t pid);
static UINT spacebox_stop_msg;   /* "SpaceboxSinkStop": WPARAM the request, LPARAM the pid */

static void SDLCALL spacebox_win_message(void *data, void *hwnd, unsigned int msg,
                                         Uint64 wp, Sint64 lp)
{
    int sc = (lp >> 16) & 0xff;

    if (spacebox_stop_msg && msg == spacebox_stop_msg) {
        if (!spacebox_sink_stop_request(wp, lp)) {
            shutdown_action = SHUTDOWN_ACTION_POWEROFF;
            qemu_system_shutdown_request(SHUTDOWN_CAUSE_HOST_UI);
        }
        return;
    }

    if ((msg == WM_KEYDOWN || msg == WM_SYSKEYDOWN) && (sc & 0x7f) >= 0x71 && (sc & 0x7f) <= 0x72) {
        SDL_Scancode code = (sc & 0x7f) == 0x72 ? SDL_SCANCODE_LANG1 : SDL_SCANCODE_LANG2;

        spacebox_win_key(code, true);
        spacebox_win_key(code, false);
    }
}
#else
static int spacebox_points(int pixels)
{
    return pixels / spacebox_pixel_scale();
}
#endif

static void spacebox_update_ui_info(struct sdl2_console *scon)
{
    if (!scon->real_window || !dpy_ui_info_supported(scon->dcl.con)) return;
    QemuUIInfo info = *dpy_get_ui_info(scon->dcl.con);
    SDL_DisplayMode dm = {0};
    int width, height;
    SDL_GetWindowSize(scon->real_window, &width, &height);
    info.width = width * spacebox_pixel_scale();
    info.height = height * spacebox_pixel_scale();
#ifdef CONFIG_WIN32
    if (spacebox_win_dpi()) {
        int pw, ph;

        /* the window's size in physical pixels */
        SDL_GL_GetDrawableSize(scon->real_window, &pw, &ph);
        if (pw > 0 && ph > 0 && width > 0) {
            spacebox_win_scale = (double)pw / width;
            info.width = pw;
            info.height = ph;
        }
        if (spacebox_win_scale > 1.99) {
            /* the density for the guest's compositor: the Mac's 220 per inch at 2x */
            info.width_mm = info.width * 254 / 10 / (110 * spacebox_win_scale);
            info.height_mm = info.height * 254 / 10 / (110 * spacebox_win_scale);
        } else {
            /*
             * A display below 200%: what the Mac's sink gives the guest on a
             * display without Retina (spacebox_pixel_scale: twice the
             * window's points, 220 per inch), shown scaled to the window
             * (whose size in points is then half the guest's surface:
             * spacebox_points); at 200% that is the window's own pixels, as
             * on a Retina display. The guest's desktop keeps the scale 2 (its
             * agent), so with the window's own pixels at 100% it would have
             * half the window's points (the user's answer of 2026-10-26: as
             * the Mac, at 125% to 175% too).
             */
            info.width = width * spacebox_win_guest_scale();
            info.height = height * spacebox_win_guest_scale();
            info.width_mm = info.width * 254 / 10 / 220;
            info.height_mm = info.height * 254 / 10 / 220;
        }
    }
#endif
    if (spacebox_pixel_scale() == 2) {
        /*
         * Tell the guest how dense the pixels are (220 per inch) through the
         * monitor's physical size. Its compositor then picks the 2x scale by
         * itself for every new size; with QEMU's default of 100 per inch it
         * came up at 1x first and had to be corrected afterwards.
         */
        info.width_mm = info.width * 254 / 10 / 220;
        info.height_mm = info.height * 254 / 10 / 220;
    }
    if (SDL_GetCurrentDisplayMode(SDL_GetWindowDisplayIndex(scon->real_window), &dm) == 0 && dm.refresh_rate > 0) {
        info.refresh_rate = dm.refresh_rate * 1000;
    }
#ifdef CONFIG_WIN32
    {
        unsigned mhz = spacebox_win_window_refresh_mhz(scon->real_window);

        if (mhz) {
            info.refresh_rate = mhz;
        }
        if (info.width != spacebox_win_told_w || info.height != spacebox_win_told_h) {
            spacebox_win_told_w = info.width;
            spacebox_win_told_h = info.height;
            spacebox_win_told_ms = g_get_monotonic_time() / 1000;
        }
    }
#endif
    dpy_set_ui_info(scon->dcl.con, &info, true);
    fprintf(stderr, "[SPACEBOX-DISPLAY-INFO] pixels=%ux%u refresh_mhz=%u\n",
            info.width, info.height, info.refresh_rate);
}

void sdl2_window_create(struct sdl2_console *scon)
{
    int flags = 0;

    if (!scon->surface) {
        return;
    }
    assert(!scon->real_window);

    if (gui_fullscreen) {
        flags |= SDL_WINDOW_FULLSCREEN_DESKTOP;
    } else {
        flags |= SDL_WINDOW_RESIZABLE;
    }
    if (scon->hidden) {
        flags |= SDL_WINDOW_HIDDEN;
    }
#ifdef CONFIG_OPENGL
    if (scon->opengl) {
        flags |= SDL_WINDOW_OPENGL;
    }
#endif

#ifdef CONFIG_DARWIN
    if (spacebox_pixel_scale() == 2) {
        flags |= SDL_WINDOW_ALLOW_HIGHDPI;
    }
    if (scon->opengl && scon->opts->gl != DISPLAY_GL_MODE_ES) {
        SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
        SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
        SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 2);
    }
#endif
#ifdef CONFIG_WIN32
    if (spacebox_win_dpi()) {
        float ddpi;

        flags |= SDL_WINDOW_ALLOW_HIGHDPI;
        if (spacebox_win_scale == 1 && !SDL_GetDisplayDPI(0, &ddpi, NULL, NULL) && ddpi > 0) {
            spacebox_win_scale = ddpi / 96.0;
        }
    }
    if (scon->opengl && scon->opts->gl != DISPLAY_GL_MODE_ES) {
        /* a core context of 4.x, as the Mac's core profile (4.1 there) */
        SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
        SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 4);
        SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 1);
    }
#endif
    scon->real_window = SDL_CreateWindow("", SDL_WINDOWPOS_UNDEFINED,
                                         SDL_WINDOWPOS_UNDEFINED,
                                         spacebox_points(surface_width(scon->surface)),
                                         spacebox_points(surface_height(scon->surface)),
                                         flags);
    /*
     * Spacebox: SDL turns text input on by itself on desktop platforms. With it
     * on, the host input method (for example the macOS Korean one) composes text
     * from the keystrokes and SDL does not deliver them as key events. A guest
     * display needs the raw keys; the guest has its own input method.
     */
    if (qemu_console_is_graphic(scon->dcl.con)) {
        SDL_StopTextInput();
    }
    if (scon->opengl) {
        const char *driver = "opengl";

        if (scon->opts->gl == DISPLAY_GL_MODE_ES) {
            driver = "opengles2";
            SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK,
                                SDL_GL_CONTEXT_PROFILE_ES);
        }

        SDL_SetHint(SDL_HINT_RENDER_DRIVER, driver);
        SDL_SetHint(SDL_HINT_RENDER_BATCHING, "1");

        scon->winctx = SDL_GL_CreateContext(scon->real_window);
#ifdef CONFIG_WIN32
        if (scon->winctx) {
            int profile = 0, major = 0, minor = 0;

            SDL_GL_GetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, &profile);
            SDL_GL_GetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, &major);
            SDL_GL_GetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, &minor);
            fprintf(stderr, "[SPACEBOX-GL] window context: %s, %s, %s; profile %s, asked %d.%d\n",
                    (const char *)glGetString(GL_VENDOR), (const char *)glGetString(GL_RENDERER),
                    (const char *)glGetString(GL_VERSION),
                    profile == SDL_GL_CONTEXT_PROFILE_CORE ? "core" : "not core", major, minor);
        } else {
            fprintf(stderr, "[SPACEBOX-GL] no window context: %s\n", SDL_GetError());
        }
#endif
        const char *interval = getenv("SPACEBOX_SWAP_INTERVAL");
        int requested = interval ? atoi(interval) : 1;
        int rc = SDL_GL_SetSwapInterval(requested);
        SDL_DisplayMode dm = {0};
        SDL_GetCurrentDisplayMode(SDL_GetWindowDisplayIndex(scon->real_window), &dm);
        fprintf(stderr, "[SPACEBOX-SYNC] requested=%d actual=%d rc=%d display_hz=%d mode=%dx%d\n",
                requested, SDL_GL_GetSwapInterval(), rc, dm.refresh_rate, dm.w, dm.h);
#ifdef CONFIG_OPENGL
        sdl2_gl_present_init(scon);
#endif
    } else {
        /* The SDL renderer is only used by sdl2-2D, when OpenGL is disabled */
        scon->real_renderer = SDL_CreateRenderer(scon->real_window, -1, 0);
    }
#ifdef CONFIG_WIN32
    if (spacebox_win_dpi()) {
#else
    if (spacebox_pixel_scale() == 2) {
#endif
        int w, h, pw, ph;
        SDL_SetWindowPosition(scon->real_window, 100, 100);
        SDL_GetWindowSize(scon->real_window, &w, &h);
        SDL_GL_GetDrawableSize(scon->real_window, &pw, &ph);
        fprintf(stderr, "[SPACEBOX-WINDOW] logical=%dx%d drawable=%dx%d shown=%d\n",
                w, h, pw, ph, !!(SDL_GetWindowFlags(scon->real_window) & SDL_WINDOW_SHOWN));
    }
    spacebox_update_ui_info(scon);
#ifdef CONFIG_WIN32
    /* the placeholder's size: the guest's first surface is followed */
    spacebox_win_told_ms = 0;
#endif
    sdl_update_caption(scon);
}

void sdl2_window_destroy(struct sdl2_console *scon)
{
    if (!scon->real_window) {
        return;
    }

    if (scon->winctx) {
#ifdef CONFIG_OPENGL
        sdl2_gl_present_destroy(scon);
#endif
        SDL_GL_DeleteContext(scon->winctx);
        scon->winctx = NULL;
    }
    if (scon->real_renderer) {
        SDL_DestroyRenderer(scon->real_renderer);
        scon->real_renderer = NULL;
    }
    SDL_DestroyWindow(scon->real_window);
    scon->real_window = NULL;
}

#ifdef CONFIG_WIN32
static void spacebox_win_window_resize(struct sdl2_console *scon, bool asked);

/* a change of the guest's surface (sdl2_gl_switch, sdl2_2d_switch) */
void sdl2_window_resize(struct sdl2_console *scon)
{
    spacebox_win_window_resize(scon, false);
}

/* ASKED: Ctrl+Alt+U, the window to the guest's size whatever was told */
static void spacebox_win_window_resize(struct sdl2_console *scon, bool asked)
{
    int sw, sh;
    int64_t since;

    if (!scon->real_window) {
        return;
    }
    sw = surface_width(scon->surface);
    sh = surface_height(scon->surface);
    since = g_get_monotonic_time() / 1000 - spacebox_win_told_ms;
    if (!asked && spacebox_win_told_ms && since < SPACEBOX_WIN_FOLLOW_MS &&
        (sw != spacebox_win_told_w || sh != spacebox_win_told_h)) {
        fprintf(stderr, "[SPACEBOX-WINDOW] the guest's surface %dx%d is not the size told %" PRId64
                " ms ago (%dx%d): the window keeps its size\n",
                sw, sh, since, spacebox_win_told_w, spacebox_win_told_h);
        return;
    }
#else
void sdl2_window_resize(struct sdl2_console *scon)
{
    if (!scon->real_window) {
        return;
    }
#endif

    SDL_SetWindowSize(scon->real_window,
                      spacebox_points(surface_width(scon->surface)),
                      spacebox_points(surface_height(scon->surface)));
    spacebox_update_ui_info(scon);
}

static void sdl2_redraw(struct sdl2_console *scon)
{
    if (scon->opengl) {
#ifdef CONFIG_OPENGL
        sdl2_gl_redraw(scon);
#endif
    } else {
        sdl2_2d_redraw(scon);
    }
}

/* hw/display/virtio-gpu-remote.c: the user's toggle of the GPU acceleration */
void spacebox_gpu_toggle(void);
const char *spacebox_gpu_caption(unsigned *gen);

static void sdl_update_caption(struct sdl2_console *scon)
{
    char win_title[1024];
    char icon_title[1024];
    const char *status = "";
    unsigned gen;
    const char *gpu = spacebox_gpu_caption(&gen);

#ifdef CONFIG_WIN32
    bool grabbed = gui_grab || spacebox_kbd_grab;
#else
    bool grabbed = gui_grab;
#endif

    if (!runstate_is_running()) {
        status = " [Stopped]";
    } else if (grabbed) {
        if (alt_grab) {
#ifdef CONFIG_DARWIN
            status = " - Press ⌃⌥⇧G to exit grab";
#else
            status = " - Press Ctrl-Alt-Shift-G to exit grab";
#endif
        } else if (ctrl_grab) {
            status = " - Press Right-Ctrl-G to exit grab";
        } else {
#ifdef CONFIG_DARWIN
            status = " - Press ⌃⌥G to exit grab";
#else
            status = " - Press Ctrl-Alt-G to exit grab";
#endif
        }
    }

    if (qemu_name) {
        snprintf(win_title, sizeof(win_title), "QEMU (%s-%d)%s%s", qemu_name,
                 scon->idx, status, gpu);
        snprintf(icon_title, sizeof(icon_title), "QEMU (%s)", qemu_name);
    } else {
        snprintf(win_title, sizeof(win_title), "QEMU%s%s", status, gpu);
        snprintf(icon_title, sizeof(icon_title), "QEMU");
    }

    if (scon->real_window) {
        static char said[1024];

        SDL_SetWindowTitle(scon->real_window, win_title);
        if (*gpu && strcmp(said, win_title)) {
            /* for the checks of the toggle */
            g_strlcpy(said, win_title, sizeof(said));
            fprintf(stderr, "[SPACEBOX-WINDOW] title: %s\n", win_title);
        }
    }
}

static void sdl_hide_cursor(struct sdl2_console *scon)
{
    if (scon->opts->has_show_cursor && scon->opts->show_cursor) {
        return;
    }

    SDL_ShowCursor(SDL_DISABLE);
    SDL_SetCursor(sdl_cursor_hidden);

    if (!qemu_input_is_absolute(scon->dcl.con)) {
        SDL_SetRelativeMouseMode(SDL_TRUE);
    }
}

static void sdl_show_cursor(struct sdl2_console *scon)
{
    if (scon->opts->has_show_cursor && scon->opts->show_cursor) {
        return;
    }

    if (!qemu_input_is_absolute(scon->dcl.con)) {
        SDL_SetRelativeMouseMode(SDL_FALSE);
    }

    if (guest_cursor &&
        (gui_grab || qemu_input_is_absolute(scon->dcl.con) || absolute_enabled)) {
        SDL_SetCursor(guest_sprite);
    } else {
        SDL_SetCursor(sdl_cursor_normal);
    }

    SDL_ShowCursor(SDL_ENABLE);
}

static void sdl_grab_start(struct sdl2_console *scon)
{
    QemuConsole *con = scon ? scon->dcl.con : NULL;

    if (!con || !qemu_console_is_graphic(con)) {
        return;
    }
    /*
     * If the application is not active, do not try to enter grab state. This
     * prevents 'SDL_WM_GrabInput(SDL_GRAB_ON)' from blocking all the
     * application (SDL bug).
     */
    if (!(SDL_GetWindowFlags(scon->real_window) & SDL_WINDOW_INPUT_FOCUS)) {
        return;
    }
    if (guest_cursor) {
        SDL_SetCursor(guest_sprite);
        if (!qemu_input_is_absolute(scon->dcl.con) && !absolute_enabled) {
            SDL_WarpMouseInWindow(scon->real_window, guest_x, guest_y);
        }
    } else {
        sdl_hide_cursor(scon);
    }
    /* An absolute tablet needs no pointer confinement. */
    SDL_SetWindowGrab(scon->real_window,
        (qemu_input_is_absolute(scon->dcl.con) || absolute_enabled) ? SDL_FALSE : SDL_TRUE);
    gui_grab = 1;
    sdl_update_caption(scon);
}

static void sdl_grab_end(struct sdl2_console *scon)
{
    SDL_SetWindowGrab(scon->real_window, SDL_FALSE);
    gui_grab = 0;
    sdl_show_cursor(scon);
    sdl_update_caption(scon);
}

#ifdef CONFIG_WIN32
/*
 * The keyboard is grabbed, so that the Windows keys, Alt+Tab and Print Screen
 * go to the guest, by the hotkey Ctrl+Alt+G and in full screen, and released
 * by the hotkey again, at the end of full screen and when the window loses
 * the focus. It is not grabbed by the pointer's entering the window, which
 * on an absolute tablet starts QEMU's grab (absolute_mouse_grab): the user's
 * Alt+Tab and Windows keys stay the user's then.
 */
static void spacebox_win_kbd_grab(struct sdl2_console *scon, bool on)
{
    spacebox_kbd_grab = on;
    SDL_SetWindowKeyboardGrab(scon->real_window, on ? SDL_TRUE : SDL_FALSE);
    spacebox_win_print_hook(on);
    sdl_update_caption(scon);
}
#endif

static void absolute_mouse_grab(struct sdl2_console *scon)
{
    int mouse_x, mouse_y;
    int scr_w, scr_h;
    SDL_GetMouseState(&mouse_x, &mouse_y);
    SDL_GetWindowSize(scon->real_window, &scr_w, &scr_h);
    if (mouse_x > 0 && mouse_x < scr_w - 1 &&
        mouse_y > 0 && mouse_y < scr_h - 1) {
        sdl_grab_start(scon);
    }
}

static void sdl_mouse_mode_change(Notifier *notify, void *data)
{
    if (qemu_input_is_absolute(sdl2_console[0].dcl.con)) {
        if (!absolute_enabled) {
            absolute_enabled = 1;
            SDL_SetRelativeMouseMode(SDL_FALSE);
            absolute_mouse_grab(&sdl2_console[0]);
        }
    } else if (absolute_enabled) {
        if (!gui_fullscreen) {
            sdl_grab_end(&sdl2_console[0]);
        }
        absolute_enabled = 0;
    }
}

static void sdl_send_mouse_event(struct sdl2_console *scon, int dx, int dy,
                                 int x, int y, int state)
{
    static uint32_t bmap[INPUT_BUTTON__MAX] = {
        [INPUT_BUTTON_LEFT]       = SDL_BUTTON(SDL_BUTTON_LEFT),
        [INPUT_BUTTON_MIDDLE]     = SDL_BUTTON(SDL_BUTTON_MIDDLE),
        [INPUT_BUTTON_RIGHT]      = SDL_BUTTON(SDL_BUTTON_RIGHT),
        [INPUT_BUTTON_SIDE]       = SDL_BUTTON(SDL_BUTTON_X1),
        [INPUT_BUTTON_EXTRA]      = SDL_BUTTON(SDL_BUTTON_X2)
    };
    static uint32_t prev_state;

    if (prev_state != state) {
        qemu_input_update_buttons(scon->dcl.con, bmap, prev_state, state);
        prev_state = state;
    }

    if (qemu_input_is_absolute(scon->dcl.con)) {
        qemu_input_queue_abs(scon->dcl.con, INPUT_AXIS_X,
                             x, 0, surface_width(scon->surface));
        qemu_input_queue_abs(scon->dcl.con, INPUT_AXIS_Y,
                             y, 0, surface_height(scon->surface));
    } else {
        if (guest_cursor) {
            x -= guest_x;
            y -= guest_y;
            guest_x += x;
            guest_y += y;
            dx = x;
            dy = y;
        }
        qemu_input_queue_rel(scon->dcl.con, INPUT_AXIS_X, dx);
        qemu_input_queue_rel(scon->dcl.con, INPUT_AXIS_Y, dy);
    }
    qemu_input_event_sync();
}

static void toggle_full_screen(struct sdl2_console *scon)
{
    gui_fullscreen = !gui_fullscreen;
    if (gui_fullscreen) {
        SDL_SetWindowFullscreen(scon->real_window,
                                SDL_WINDOW_FULLSCREEN_DESKTOP);
        gui_saved_grab = gui_grab;
        sdl_grab_start(scon);
#ifdef CONFIG_WIN32
        spacebox_kbd_saved = spacebox_kbd_grab;
        spacebox_win_kbd_grab(scon, true);
#endif
    } else {
        if (!gui_saved_grab) {
            sdl_grab_end(scon);
        }
#ifdef CONFIG_WIN32
        spacebox_win_kbd_grab(scon, spacebox_kbd_saved);
#endif
        SDL_SetWindowFullscreen(scon->real_window, 0);
    }
    sdl2_redraw(scon);
}

static int get_mod_state(void)
{
    SDL_Keymod mod = SDL_GetModState();

    if (alt_grab) {
        return (mod & (gui_grab_code | KMOD_LSHIFT)) ==
            (gui_grab_code | KMOD_LSHIFT);
    } else if (ctrl_grab) {
        return (mod & KMOD_RCTRL) == KMOD_RCTRL;
    } else {
        return (mod & gui_grab_code) == gui_grab_code;
    }
}

/* get_mod_state() of the modifiers that a key event carries (an event put in
 * SDL's queue, as the input-test hook does, leaves SDL's own state as it was) */
static int event_mod_state(SDL_Keymod mod)
{
    if (alt_grab) {
        return (mod & (gui_grab_code | KMOD_LSHIFT)) ==
            (gui_grab_code | KMOD_LSHIFT);
    } else if (ctrl_grab) {
        return (mod & KMOD_RCTRL) == KMOD_RCTRL;
    }
    return (mod & gui_grab_code) == gui_grab_code;
}

static void handle_keydown(SDL_Event *ev)
{
    int win;
    struct sdl2_console *scon = get_scon_from_window(ev->key.windowID);
    int gui_key_modifier_pressed = get_mod_state() ||
                                   event_mod_state(ev->key.keysym.mod);

    if (!scon) {
        return;
    }

    scon->gui_keysym = false;

    if (!scon->ignore_hotkeys && gui_key_modifier_pressed && !ev->key.repeat) {
        switch (ev->key.keysym.scancode) {
        case SDL_SCANCODE_2:
        case SDL_SCANCODE_3:
        case SDL_SCANCODE_4:
        case SDL_SCANCODE_5:
        case SDL_SCANCODE_6:
        case SDL_SCANCODE_7:
        case SDL_SCANCODE_8:
        case SDL_SCANCODE_9:
            if (gui_grab) {
                sdl_grab_end(scon);
            }

            win = ev->key.keysym.scancode - SDL_SCANCODE_1;
            if (win < sdl2_num_outputs) {
                sdl2_console[win].hidden = !sdl2_console[win].hidden;
                if (sdl2_console[win].real_window) {
                    if (sdl2_console[win].hidden) {
                        SDL_HideWindow(sdl2_console[win].real_window);
                    } else {
                        SDL_ShowWindow(sdl2_console[win].real_window);
                    }
                }
                sdl2_release_modifiers(scon);
                scon->gui_keysym = true;
            }
            break;
        case SDL_SCANCODE_F:
            toggle_full_screen(scon);
            scon->gui_keysym = true;
            break;
        case SDL_SCANCODE_A:
            /* Spacebox: the GPU acceleration off (the guest renders, its
             * pictures are shown here) or on; the title says which */
            spacebox_gpu_toggle();
            scon->gui_keysym = true;
            break;
        case SDL_SCANCODE_G:
            scon->gui_keysym = true;
#ifdef CONFIG_WIN32
            /*
             * The hotkey's grab is the keyboard's, which only the hotkey (and
             * full screen) sets; the pointer's grab follows it. On an absolute
             * tablet the pointer's moves in the window start the pointer's
             * grab by themselves (absolute_mouse_grab), so toggling the two
             * apart left them out of phase: the hotkey then grabbed one and
             * released the other, and the title never showed a release.
             */
            if (!gui_fullscreen) {
                bool on = !spacebox_kbd_grab;

                spacebox_win_kbd_grab(scon, on);
                if (on && !gui_grab) {
                    sdl_grab_start(scon);
                } else if (!on && gui_grab) {
                    sdl_grab_end(scon);
                }
                break;
            }
#endif
            if (!gui_grab) {
                sdl_grab_start(scon);
            } else if (!gui_fullscreen) {
                sdl_grab_end(scon);
            }
            break;
        case SDL_SCANCODE_U:
#ifdef CONFIG_WIN32
            spacebox_win_window_resize(scon, true);
#else
            sdl2_window_resize(scon);
#endif
            if (!scon->opengl) {
                /* re-create scon->texture */
                sdl2_2d_switch(&scon->dcl, scon->surface);
            }
            scon->gui_keysym = true;
            break;
#if 0
        case SDL_SCANCODE_KP_PLUS:
        case SDL_SCANCODE_KP_MINUS:
            if (!gui_fullscreen) {
                int scr_w, scr_h;
                int width, height;
                SDL_GetWindowSize(scon->real_window, &scr_w, &scr_h);

                width = MAX(scr_w + (ev->key.keysym.scancode ==
                                     SDL_SCANCODE_KP_PLUS ? 50 : -50),
                            160);
                height = (surface_height(scon->surface) * width) /
                    surface_width(scon->surface);
                fprintf(stderr, "%s: scale to %dx%d\n",
                        __func__, width, height);
                sdl_scale(scon, width, height);
                sdl2_redraw(scon);
                scon->gui_keysym = true;
            }
#endif
        default:
            break;
        }
    }
    if (!scon->gui_keysym) {
        sdl2_process_key(scon, &ev->key);
    }
}

static void handle_keyup(SDL_Event *ev)
{
    struct sdl2_console *scon = get_scon_from_window(ev->key.windowID);

    if (!scon) {
        return;
    }

    scon->ignore_hotkeys = false;
    sdl2_process_key(scon, &ev->key);
}

static void handle_textinput(SDL_Event *ev)
{
    struct sdl2_console *scon = get_scon_from_window(ev->text.windowID);
    QemuConsole *con = scon ? scon->dcl.con : NULL;

    if (!con) {
        return;
    }

    if (!scon->gui_keysym && QEMU_IS_TEXT_CONSOLE(con)) {
        qemu_text_console_put_string(QEMU_TEXT_CONSOLE(con), ev->text.text, strlen(ev->text.text));
    }
}

static void handle_mousemotion(SDL_Event *ev)
{
    int max_x, max_y;
    struct sdl2_console *scon = get_scon_from_window(ev->motion.windowID);
    int scr_w, scr_h, surf_w, surf_h, x, y, dx, dy;

    if (!scon || !qemu_console_is_graphic(scon->dcl.con)) {
        return;
    }

    SDL_GetWindowSize(scon->real_window, &scr_w, &scr_h);
    if (qemu_input_is_absolute(scon->dcl.con) || absolute_enabled) {
        max_x = scr_w - 1;
        max_y = scr_h - 1;
        if (gui_grab && !gui_fullscreen
            && (ev->motion.x == 0 || ev->motion.y == 0 ||
                ev->motion.x == max_x || ev->motion.y == max_y)) {
            sdl_grab_end(scon);
        }
        if (!gui_grab &&
            (ev->motion.x > 0 && ev->motion.x < max_x &&
             ev->motion.y > 0 && ev->motion.y < max_y)) {
            sdl_grab_start(scon);
        }
    }
    surf_w = surface_width(scon->surface);
    surf_h = surface_height(scon->surface);
    x = (int64_t)ev->motion.x * surf_w / scr_w;
    y = (int64_t)ev->motion.y * surf_h / scr_h;
    dx = (int64_t)ev->motion.xrel * surf_w / scr_w;
    dy = (int64_t)ev->motion.yrel * surf_h / scr_h;
    if (gui_grab || qemu_input_is_absolute(scon->dcl.con) || absolute_enabled) {
        sdl_send_mouse_event(scon, dx, dy, x, y, ev->motion.state);
    }
}

static void handle_mousebutton(SDL_Event *ev)
{
    int buttonstate = SDL_GetMouseState(NULL, NULL);
    SDL_MouseButtonEvent *bev;
    struct sdl2_console *scon = get_scon_from_window(ev->button.windowID);
    int scr_w, scr_h, x, y;

    if (!scon || !qemu_console_is_graphic(scon->dcl.con)) {
        return;
    }

    bev = &ev->button;
    SDL_GetWindowSize(scon->real_window, &scr_w, &scr_h);
    x = (int64_t)bev->x * surface_width(scon->surface) / scr_w;
    y = (int64_t)bev->y * surface_height(scon->surface) / scr_h;

    if (!gui_grab && !qemu_input_is_absolute(scon->dcl.con)) {
        if (ev->type == SDL_MOUSEBUTTONUP && bev->button == SDL_BUTTON_LEFT) {
            /* start grabbing all events */
            sdl_grab_start(scon);
        }
    } else {
        if (ev->type == SDL_MOUSEBUTTONDOWN) {
            buttonstate |= SDL_BUTTON(bev->button);
        } else {
            buttonstate &= ~SDL_BUTTON(bev->button);
        }
        sdl_send_mouse_event(scon, 0, 0, x, y, buttonstate);
    }
}

static void handle_mousewheel(SDL_Event *ev)
{
    struct sdl2_console *scon = get_scon_from_window(ev->wheel.windowID);
    SDL_MouseWheelEvent *wev = &ev->wheel;
    InputButton btn;
#ifdef CONFIG_WIN32
    /* as the Mac's scroll events, which go to the agent before SDL has them */
    if (scon && qemu_console_is_graphic(scon->dcl.con) && spacebox_scroll_wheel(wev)) {
        return;
    }
#endif
    const char *trace=getenv("SPACEBOX_SCROLL_TRACE");
    if (trace) { FILE *f=fopen(trace,"a"); if(f) { fprintf(f,"{\"stage\":\"sdl\",\"host_us\":%lld,\"x\":%d,\"y\":%d,\"precise_x\":%.9g,\"precise_y\":%.9g,\"direction\":%u}\n",(long long)g_get_monotonic_time(),wev->x,wev->y,wev->preciseX,wev->preciseY,wev->direction); fclose(f); } }

    if (!scon || !qemu_console_is_graphic(scon->dcl.con)) {
        return;
    }

    if (wev->y > 0) {
        btn = INPUT_BUTTON_WHEEL_UP;
    } else if (wev->y < 0) {
        btn = INPUT_BUTTON_WHEEL_DOWN;
    } else if (wev->x < 0) {
        btn = INPUT_BUTTON_WHEEL_RIGHT;
    } else if (wev->x > 0) {
        btn = INPUT_BUTTON_WHEEL_LEFT;
    } else {
        return;
    }

    qemu_input_queue_btn(scon->dcl.con, btn, true);
    qemu_input_event_sync();
    qemu_input_queue_btn(scon->dcl.con, btn, false);
    qemu_input_event_sync();
}

/* hw/display/virtio-gpu-remote.c: a remote sink's close (it detaches first). */
bool spacebox_sink_close(void);

static void handle_windowevent(SDL_Event *ev)
{
    struct sdl2_console *scon = get_scon_from_window(ev->window.windowID);
    bool allow_close = true;

    if (!scon) {
        return;
    }

    switch (ev->window.event) {
    case SDL_WINDOWEVENT_RESIZED:
    case SDL_WINDOWEVENT_SIZE_CHANGED:
        spacebox_update_ui_info(scon);
        sdl2_redraw(scon);
        break;
    case SDL_WINDOWEVENT_EXPOSED:
        sdl2_redraw(scon);
        break;
    case SDL_WINDOWEVENT_FOCUS_GAINED:
        if (qemu_console_is_graphic(scon->dcl.con)) {
            SDL_StopTextInput(); /* see sdl2_window_create() */
        }
        /* fall through */
    case SDL_WINDOWEVENT_ENTER:
        if (!gui_grab && (qemu_input_is_absolute(scon->dcl.con) || absolute_enabled)) {
            absolute_mouse_grab(scon);
        }
        /* If a new console window opened using a hotkey receives the
         * focus, SDL sends another KEYDOWN event to the new window,
         * closing the console window immediately after.
         *
         * Work around this by ignoring further hotkey events until a
         * key is released.
         */
        scon->ignore_hotkeys = get_mod_state();
        break;
    case SDL_WINDOWEVENT_FOCUS_LOST:
        sdl2_release_modifiers(scon);
        if (gui_grab && !gui_fullscreen) {
            sdl_grab_end(scon);
        }
#ifdef CONFIG_WIN32
        if (spacebox_kbd_grab && !gui_fullscreen) {
            spacebox_win_kbd_grab(scon, false);
        }
#endif
        break;
    case SDL_WINDOWEVENT_RESTORED:
        update_displaychangelistener(&scon->dcl, GUI_REFRESH_INTERVAL_DEFAULT);
        break;
    case SDL_WINDOWEVENT_MINIMIZED:
        update_displaychangelistener(&scon->dcl, 500);
        break;
    case SDL_WINDOWEVENT_CLOSE:
        if (qemu_console_is_graphic(scon->dcl.con)) {
            if (spacebox_sink_close()) {
                break; /* the remote sink detaches first, then ends */
            }
            if (scon->opts->has_window_close && !scon->opts->window_close) {
                allow_close = false;
            }
            if (allow_close) {
                shutdown_action = SHUTDOWN_ACTION_POWEROFF;
                qemu_system_shutdown_request(SHUTDOWN_CAUSE_HOST_UI);
            }
        } else {
            SDL_HideWindow(scon->real_window);
            scon->hidden = true;
        }
        break;
    case SDL_WINDOWEVENT_SHOWN:
        scon->hidden = false;
        break;
    case SDL_WINDOWEVENT_HIDDEN:
        scon->hidden = true;
        break;
    }
}

#include "spacebox-input-test.h"

void sdl2_poll_events(struct sdl2_console *scon)
{
    SDL_Event ev1, *ev = &ev1;
    bool allow_close = true;
    int idle = 1;

    if (scon->last_vm_running != runstate_is_running()) {
        scon->last_vm_running = runstate_is_running();
        sdl_update_caption(scon);
    }

    spacebox_window_request(scon);
    spacebox_input_test(scon);
    {
        /* the toggle's state changed: the title says it */
        static unsigned gpu_gen;
        unsigned gen;

        spacebox_gpu_caption(&gen);
        if (gen != gpu_gen) {
            gpu_gen = gen;
            sdl_update_caption(scon);
        }
    }
#if defined(CONFIG_DARWIN) || defined(CONFIG_WIN32)
    spacebox_scroll_tick(scon->real_window);
#endif
    while (SDL_PollEvent(ev)) {
        switch (ev->type) {
        case SDL_KEYDOWN:
            idle = 0;
            spacebox_input_real(ev);
            handle_keydown(ev);
            break;
        case SDL_KEYUP:
            idle = 0;
            spacebox_input_real(ev);
            handle_keyup(ev);
            break;
        case SDL_TEXTINPUT:
            idle = 0;
            handle_textinput(ev);
            break;
        case SDL_QUIT:
            if (spacebox_sink_close()) {
                break;
            }
            if (scon->opts->has_window_close && !scon->opts->window_close) {
                allow_close = false;
            }
            if (allow_close) {
                shutdown_action = SHUTDOWN_ACTION_POWEROFF;
                qemu_system_shutdown_request(SHUTDOWN_CAUSE_HOST_UI);
            }
            break;
        case SDL_MOUSEMOTION:
            idle = 0;
            spacebox_input_real(ev);
            handle_mousemotion(ev);
            break;
        case SDL_MOUSEBUTTONDOWN:
        case SDL_MOUSEBUTTONUP:
            idle = 0;
            spacebox_input_real(ev);
            handle_mousebutton(ev);
            break;
        case SDL_MOUSEWHEEL:
            idle = 0;
            handle_mousewheel(ev);
            break;
        case SDL_WINDOWEVENT:
            handle_windowevent(ev);
            break;
        default:
            break;
        }
    }

    if (idle) {
        if (scon->idle_counter < SDL2_MAX_IDLE_COUNT) {
            scon->idle_counter++;
            if (scon->idle_counter >= SDL2_MAX_IDLE_COUNT) {
                scon->dcl.update_interval = GUI_REFRESH_INTERVAL_DEFAULT;
            }
        }
    } else {
        scon->idle_counter = 0;
        scon->dcl.update_interval = SDL2_REFRESH_INTERVAL_BUSY;
    }
}

static void sdl_mouse_warp(DisplayChangeListener *dcl,
                           int x, int y, bool on)
{
    struct sdl2_console *scon = container_of(dcl, struct sdl2_console, dcl);

    if (!qemu_console_is_graphic(scon->dcl.con)) {
        return;
    }

    if (on) {
        if (!guest_cursor) {
            sdl_show_cursor(scon);
        }
        if (gui_grab || qemu_input_is_absolute(scon->dcl.con) || absolute_enabled) {
            SDL_SetCursor(guest_sprite);
            if (!qemu_input_is_absolute(scon->dcl.con) && !absolute_enabled) {
                SDL_WarpMouseInWindow(scon->real_window, x, y);
            }
        }
    } else if (gui_grab) {
        sdl_hide_cursor(scon);
    }
    guest_cursor = on;
    guest_x = x, guest_y = y;
}

static void sdl_mouse_define(DisplayChangeListener *dcl,
                             QEMUCursor *c)
{

    if (guest_sprite) {
        SDL_FreeCursor(guest_sprite);
    }

    if (guest_sprite_surface) {
        SDL_FreeSurface(guest_sprite_surface);
    }

    guest_sprite_surface =
        SDL_CreateRGBSurfaceFrom(c->data, c->width, c->height, 32, c->width * 4,
                                 0xff0000, 0x00ff00, 0xff, 0xff000000);

    if (!guest_sprite_surface) {
        fprintf(stderr, "Failed to make rgb surface from %p\n", c);
        return;
    }
    guest_sprite = SDL_CreateColorCursor(guest_sprite_surface,
                                         c->hot_x, c->hot_y);
    if (!guest_sprite) {
        fprintf(stderr, "Failed to make color cursor from %p\n", c);
        return;
    }
    if (guest_cursor &&
        (gui_grab || qemu_input_is_absolute(dcl->con) || absolute_enabled)) {
        SDL_SetCursor(guest_sprite);
    }
}

static void sdl_cleanup(void)
{
    if (guest_sprite) {
        SDL_FreeCursor(guest_sprite);
    }
    SDL_QuitSubSystem(SDL_INIT_VIDEO);
}

static const DisplayChangeListenerOps dcl_2d_ops = {
    .dpy_name             = "sdl2-2d",
    .dpy_gfx_update       = sdl2_2d_update,
    .dpy_gfx_switch       = sdl2_2d_switch,
    .dpy_gfx_check_format = sdl2_2d_check_format,
    .dpy_refresh          = sdl2_2d_refresh,
    .dpy_mouse_set        = sdl_mouse_warp,
    .dpy_cursor_define    = sdl_mouse_define,
};

#ifdef CONFIG_OPENGL
static const DisplayChangeListenerOps dcl_gl_ops = {
    .dpy_name                = "sdl2-gl",
    .dpy_gfx_update          = sdl2_gl_update,
    .dpy_gfx_switch          = sdl2_gl_switch,
    .dpy_gfx_check_format    = console_gl_check_format,
    .dpy_refresh             = sdl2_gl_refresh,
    .dpy_mouse_set           = sdl_mouse_warp,
    .dpy_cursor_define       = sdl_mouse_define,

    .dpy_gl_scanout_disable  = sdl2_gl_scanout_disable,
    .dpy_gl_scanout_texture  = sdl2_gl_scanout_texture,
    .dpy_gl_update           = sdl2_gl_scanout_flush,
};

static bool
sdl2_gl_is_compatible_dcl(DisplayGLCtx *dgc,
                          DisplayChangeListener *dcl)
{
    return dcl->ops == &dcl_gl_ops;
}

static const DisplayGLCtxOps gl_ctx_ops = {
    .dpy_gl_ctx_is_compatible_dcl = sdl2_gl_is_compatible_dcl,
    .dpy_gl_ctx_create       = sdl2_gl_create_context,
    .dpy_gl_ctx_destroy      = sdl2_gl_destroy_context,
    .dpy_gl_ctx_make_current = sdl2_gl_make_context_current,
};
#endif

static void sdl2_display_early_init(DisplayOptions *o)
{
    assert(o->type == DISPLAY_TYPE_SDL);
    if (o->has_gl && o->gl) {
#ifdef CONFIG_OPENGL
        display_opengl = 1;
#endif
    }
}

static void sdl2_display_init(DisplayState *ds, DisplayOptions *o)
{
    uint8_t data = 0;
    int i;
    SDL_SysWMinfo info;
    SDL_Surface *icon = NULL;
    char *dir;

    assert(o->type == DISPLAY_TYPE_SDL);

    if (SDL_GetHintBoolean("QEMU_ENABLE_SDL_LOGGING", SDL_FALSE)) {
        SDL_LogSetAllPriority(SDL_LOG_PRIORITY_VERBOSE);
    }

#ifdef CONFIG_WIN32
    if (spacebox_win_dpi()) {
        SDL_SetHint(SDL_HINT_WINDOWS_DPI_AWARENESS, "permonitorv2");
        SDL_SetHint(SDL_HINT_WINDOWS_DPI_SCALING, "1");
    }
#endif
#ifdef CONFIG_WIN32
    spacebox_stop_msg = RegisterWindowMessageW(L"SpaceboxSinkStop");
    SDL_SetWindowsMessageHook(spacebox_win_message, NULL);
#endif
    if (SDL_Init(SDL_INIT_VIDEO)) {
        fprintf(stderr, "Could not initialize SDL(%s) - exiting\n",
                SDL_GetError());
        exit(1);
    }
#ifdef SDL_HINT_VIDEO_X11_NET_WM_BYPASS_COMPOSITOR /* only available since SDL 2.0.8 */
    SDL_SetHint(SDL_HINT_VIDEO_X11_NET_WM_BYPASS_COMPOSITOR, "0");
#endif
#ifdef CONFIG_DARWIN
    /* Keep macOS application switching available with the absolute tablet. */
    SDL_SetHint(SDL_HINT_GRAB_KEYBOARD, "0");
#elif defined(CONFIG_WIN32)
    /* The keyboard is grabbed apart from the pointer (spacebox_win_kbd_grab):
     * with this hint each release of the pointer released it too. */
    SDL_SetHint(SDL_HINT_GRAB_KEYBOARD, "0");
#else
    SDL_SetHint(SDL_HINT_GRAB_KEYBOARD, "1");
#endif
#ifdef SDL_HINT_ALLOW_ALT_TAB_WHILE_GRABBED
    SDL_SetHint(SDL_HINT_ALLOW_ALT_TAB_WHILE_GRABBED, "0");
#endif
    SDL_SetHint(SDL_HINT_WINDOWS_NO_CLOSE_ON_ALT_F4, "1");
    SDL_EnableScreenSaver();
    memset(&info, 0, sizeof(info));
    SDL_VERSION(&info.version);

    gui_fullscreen = o->has_full_screen && o->full_screen;

    if (o->u.sdl.has_grab_mod) {
        if (o->u.sdl.grab_mod == HOT_KEY_MOD_LSHIFT_LCTRL_LALT) {
            alt_grab = true;
        } else if (o->u.sdl.grab_mod == HOT_KEY_MOD_RCTRL) {
            ctrl_grab = true;
        }
    }

    for (i = 0;; i++) {
        QemuConsole *con = qemu_console_lookup_by_index(i);
        if (!con) {
            break;
        }
    }
    sdl2_num_outputs = i;
    if (sdl2_num_outputs == 0) {
        return;
    }
    sdl2_console = g_new0(struct sdl2_console, sdl2_num_outputs);
    for (i = 0; i < sdl2_num_outputs; i++) {
        QemuConsole *con = qemu_console_lookup_by_index(i);
        assert(con != NULL);
        if (!qemu_console_is_graphic(con) &&
            qemu_console_get_index(con) != 0) {
            sdl2_console[i].hidden = true;
        }
        sdl2_console[i].idx = i;
        sdl2_console[i].opts = o;
#ifdef CONFIG_OPENGL
        sdl2_console[i].opengl = display_opengl;
        sdl2_console[i].dcl.ops = display_opengl ? &dcl_gl_ops : &dcl_2d_ops;
        sdl2_console[i].dgc.ops = display_opengl ? &gl_ctx_ops : NULL;
#else
        sdl2_console[i].opengl = 0;
        sdl2_console[i].dcl.ops = &dcl_2d_ops;
#endif
        sdl2_console[i].dcl.con = con;
        sdl2_console[i].kbd = qkbd_state_init(con);
        if (display_opengl) {
            qemu_console_set_display_gl_ctx(con, &sdl2_console[i].dgc);
        }
        register_displaychangelistener(&sdl2_console[i].dcl);

#if defined(SDL_VIDEO_DRIVER_WINDOWS) || defined(SDL_VIDEO_DRIVER_X11)
        if (SDL_GetWindowWMInfo(sdl2_console[i].real_window, &info)) {
#if defined(SDL_VIDEO_DRIVER_WINDOWS)
            qemu_console_set_window_id(con, (uintptr_t)info.info.win.window);
#elif defined(SDL_VIDEO_DRIVER_X11)
            qemu_console_set_window_id(con, info.info.x11.window);
#endif
        }
#endif
    }

#ifdef CONFIG_SDL_IMAGE
    dir = get_relocated_path(CONFIG_QEMU_ICONDIR "/hicolor/128x128/apps/qemu.png");
    icon = IMG_Load(dir);
#else
    /* Spacebox: the product logo when it is installed (a BMP, alpha is
     * honoured). On macOS SDL shows this image in the Dock. */
    dir = get_relocated_path(CONFIG_QEMU_ICONDIR "/hicolor/512x512/apps/spacebox.bmp");
    icon = SDL_LoadBMP(dir);
    if (!icon) {
        /* QEMU's own: a 32x32x4 image. White pixels are transparent. */
        g_free(dir);
        dir = get_relocated_path(CONFIG_QEMU_ICONDIR "/hicolor/32x32/apps/qemu.bmp");
        icon = SDL_LoadBMP(dir);
        if (icon) {
            uint32_t colorkey = SDL_MapRGB(icon->format, 255, 255, 255);
            SDL_SetColorKey(icon, SDL_TRUE, colorkey);
        }
    }
    if (icon && getenv("SPACEBOX_PRESENT_STATS")) {
        fprintf(stderr, "[SPACEBOX] window icon %dx%d, %d bits per pixel, from %s\n",
                icon->w, icon->h, icon->format->BitsPerPixel, dir);
    }
#endif
    g_free(dir);
    if (icon) {
        SDL_SetWindowIcon(sdl2_console[0].real_window, icon);
    }

    mouse_mode_notifier.notify = sdl_mouse_mode_change;
    qemu_add_mouse_mode_change_notifier(&mouse_mode_notifier);

    sdl_cursor_hidden = SDL_CreateCursor(&data, &data, 8, 1, 0, 0);
    sdl_cursor_normal = SDL_GetCursor();

    if (gui_fullscreen) {
        sdl_grab_start(&sdl2_console[0]);
    }

    atexit(sdl_cleanup);
#if defined(CONFIG_WIN32) && defined(CONFIG_OPENGL)
    /*
     * The presenter's hidden window goes before SDL's own quit (registered
     * later, so run earlier): left to sdl_cleanup with the sink's context
     * current on it, the sink crashed inside SDL2.dll as it ended (turn G5a).
     */
    atexit(sdl2_win_gl_offscreen_close);
#endif

    /* SDL's event polling (in dpy_refresh) must happen on the main thread. */
    qemu_main = NULL;
}

static QemuDisplay qemu_display_sdl2 = {
    .type       = DISPLAY_TYPE_SDL,
    .early_init = sdl2_display_early_init,
    .init       = sdl2_display_init,
};

static void register_sdl1(void)
{
    qemu_display_register(&qemu_display_sdl2);
}

type_init(register_sdl1);

#ifdef CONFIG_OPENGL
module_dep("ui-opengl");
#endif

#if defined(CONFIG_WIN32) && defined(CONFIG_OPENGL)
/*
 * include/ui/sdl2.h: SDL's calls that make a context current, told to epoxy
 * at each new context and, on a thread, at the first context made current
 * there. epoxy's function pointers of WGL may in principle differ from
 * context to context, and telling it resets every pointer, to be looked up
 * again (wglGetProcAddress) at its next call. Every context of the sink is
 * made by SDL on its window, with one pixel format of one driver, so the
 * pointers are the same in all of them; telling epoxy at each change of the
 * current context (the renderer changes contexts many times a frame) made
 * the guest's drawing about 10 times slower on BF2's NVIDIA GL (the frames
 * of hard-case: 1.3 s against 0.12 to 0.18 s for 120, the same 60 hashes
 * either way). A new context is still told: without that the sink called a
 * null pointer at its start.
 */
#undef SDL_GL_MakeCurrent
#undef SDL_GL_CreateContext
static __thread bool spacebox_gl_told;

static void spacebox_gl_tell_epoxy(void)
{
    if (!spacebox_gl_told) {
        epoxy_handle_external_wglMakeCurrent();
        spacebox_gl_told = true;
    }
}

/*
 * The window's device context belongs to the presenter's thread while the
 * presenter runs (ui/spacebox-presenter.h). WGL binds a context to the device
 * context it is made current on, and the sink's thread changes its current
 * context many times a frame (the renderer's contexts); bound to the window's
 * device context while the presenter's thread swapped that window's buffers,
 * the sink crashed inside NVIDIA's GL driver (nvoglv64.dll, on its own thread
 * or the sink's) within a few plays of 3840 x 2160 video, and not in 12 such
 * plays with the presenter off (turn G5a). CGL has no such binding on the
 * Mac. So every other thread makes its contexts current, and makes them, on a
 * hidden window of the window's pixel format; a GL context may be current on
 * any drawable of its pixel format. Contexts made before the hidden window
 * stay usable on it.
 */
static SDL_Window *spacebox_gl_offscreen;
static __thread bool spacebox_gl_on_window;

static SDL_Window *spacebox_gl_drawable(SDL_Window *window)
{
    return spacebox_gl_offscreen && !spacebox_gl_on_window ? spacebox_gl_offscreen : window;
}

bool sdl2_win_gl_offscreen_open(SDL_Window *window)
{
    /* SDL gives an OpenGL window the pixel format of the attributes now set,
     * those the window was made with */
    spacebox_gl_offscreen = SDL_CreateWindow("", 0, 0, 1, 1,
                                             SDL_WINDOW_OPENGL | SDL_WINDOW_HIDDEN);
    if (!spacebox_gl_offscreen) {
        fprintf(stderr, "[SPACEBOX-GL] no hidden window for the sink's contexts: %s\n",
                SDL_GetError());
    }
    return spacebox_gl_offscreen != NULL;
}

void sdl2_win_gl_offscreen_close(void)
{
    SDL_Window *w = spacebox_gl_offscreen;

    spacebox_gl_offscreen = NULL;
    if (w) {
        SDL_GL_MakeCurrent(w, NULL);
        SDL_DestroyWindow(w);
    }
}

void sdl2_win_gl_on_window(void)
{
    spacebox_gl_on_window = true;
}

int sdl2_win_gl_make_current(SDL_Window *window, SDL_GLContext ctx)
{
    int r = SDL_GL_MakeCurrent(spacebox_gl_drawable(window), ctx);

    if (!r && ctx) {
        spacebox_gl_tell_epoxy();
    }
    return r;
}

SDL_GLContext sdl2_win_gl_create_context(SDL_Window *window)
{
    SDL_GLContext ctx = SDL_GL_CreateContext(spacebox_gl_drawable(window));

    if (ctx) {
        /* SDL made it current */
        epoxy_handle_external_wglMakeCurrent();
        spacebox_gl_told = true;
    }
    return ctx;
}
#endif
