#include "qemu/timer.h"
#include "ui/spacebox-hook.h"

/* Launcher command for this application's own visible window. */
static void spacebox_window_request(struct sdl2_console *scon)
{
    const char *path = getenv("SPACEBOX_WINDOW_REQUEST");
    g_autofree char *text = NULL;
    if (!path || !scon->real_window || access(path, F_OK) || !spacebox_hook_take(path, &text)) return;
    SDL_RaiseWindow(scon->real_window);
    int w, h, pw, ph;
    SDL_GetWindowSize(scon->real_window, &w, &h);
    SDL_GL_GetDrawableSize(scon->real_window, &pw, &ph);
    fprintf(stderr, "[SPACEBOX-USER-WINDOW] logical=%dx%d drawable=%dx%d shown=%d minimized=%d mouse_grab=%d\n",
            w, h, pw, ph,
            !!(SDL_GetWindowFlags(scon->real_window) & SDL_WINDOW_SHOWN),
            !!(SDL_GetWindowFlags(scon->real_window) & SDL_WINDOW_MINIMIZED),
            SDL_GetWindowGrab(scon->real_window));
}

/* The hook's own events still in SDL's queue, by kind (fields of the event do not survive the
 * queue of every SDL build, so they are counted: each one taken out of the queue is the
 * hook's while its count is above 0). */
static int spacebox_test_keys, spacebox_test_buttons, spacebox_test_motions;

/* With the input-test hook on, a key, button or motion that this application did not
 * post itself came from the person at the Mac; it is logged (motion at most every
 * 200 ms), so that a check can tell that its run had other input than its own. */
static void spacebox_input_real(const SDL_Event *ev)
{
    static int64_t last_motion;
    int64_t now = g_get_real_time();

    if (!getenv("SPACEBOX_INPUT_TEST")) {
        return;
    }
    switch (ev->type) {
    case SDL_KEYDOWN:
    case SDL_KEYUP:
        if (spacebox_test_keys > 0) {
            spacebox_test_keys--;
            return;
        }
        fprintf(stderr, "[SPACEBOX-INPUT-TEST] real key scancode=%d down=%d at %.3f\n",
                ev->key.keysym.scancode, ev->type == SDL_KEYDOWN, now / 1e6);
        break;
    case SDL_MOUSEBUTTONDOWN:
    case SDL_MOUSEBUTTONUP:
        if (spacebox_test_buttons > 0) {
            spacebox_test_buttons--;
            return;
        }
        fprintf(stderr, "[SPACEBOX-INPUT-TEST] real button %d down=%d at %.3f\n",
                ev->button.button, ev->type == SDL_MOUSEBUTTONDOWN, now / 1e6);
        break;
    case SDL_MOUSEMOTION:
        if (spacebox_test_motions > 0) {
            spacebox_test_motions--;
            return;
        }
        if (now - last_motion < 200000) {
            return;
        }
        last_motion = now;
        fprintf(stderr, "[SPACEBOX-INPUT-TEST] real motion at %.3f\n", now / 1e6);
        break;
    }
}

/*
 * "type HOLD_MS GAP_MS SHIFT TEXT": TEXT (the letters a to z, and "_" for a space) typed at a
 * fixed pace by a timer of the main loop: each key down at start + i * (HOLD_MS + GAP_MS), up
 * HOLD_MS later, with the left Shift down just before each key and up just after it when SHIFT
 * is 1. The times are absolute, so a late event does not move the ones after it. Each event goes
 * to the handlers of SDL's key events (handle_keydown, handle_keyup) as a key taken from the
 * queue does, without the wait for the next poll of the queue (one frame), which would bunch
 * the keys. Each event is logged with the time it was handed over and how late it was.
 */
static struct {
    QEMUTimer *timer;
    struct sdl2_console *scon;
    char text[128];
    int n, i;
    bool up_next, shift;
    int64_t start_ns, hold_ns, gap_ns;
} spacebox_typing;

static void spacebox_typing_key(int scancode, bool down)
{
    SDL_Event ev = {0};

    ev.type = down ? SDL_KEYDOWN : SDL_KEYUP;
    ev.key.windowID = SDL_GetWindowID(spacebox_typing.scon->real_window);
    ev.key.state = down ? SDL_PRESSED : SDL_RELEASED;
    ev.key.keysym.scancode = scancode;
    ev.key.keysym.sym = SDL_GetKeyFromScancode(scancode);
    if (down) {
        handle_keydown(&ev);
    } else {
        handle_keyup(&ev);
    }
}

static void spacebox_typing_tick(void *opaque)
{
    char ch = spacebox_typing.text[spacebox_typing.i];
    int sc = ch == '_' ? SDL_SCANCODE_SPACE : SDL_SCANCODE_A + (ch - 'a');
    bool down = !spacebox_typing.up_next;
    int64_t due = spacebox_typing.start_ns +
                  spacebox_typing.i * (spacebox_typing.hold_ns + spacebox_typing.gap_ns) +
                  (down ? 0 : spacebox_typing.hold_ns);
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_REALTIME);

    if (down && spacebox_typing.shift) {
        spacebox_typing_key(SDL_SCANCODE_LSHIFT, true);
    }
    spacebox_typing_key(sc, down);
    if (!down && spacebox_typing.shift) {
        spacebox_typing_key(SDL_SCANCODE_LSHIFT, false);
    }
    fprintf(stderr, "[SPACEBOX-INPUT-TEST] typed %d scancode=%d down=%d at %.6f late_us=%" PRId64 "\n",
            spacebox_typing.i, sc, down, g_get_real_time() / 1e6, (now - due) / 1000);
    if (down) {
        spacebox_typing.up_next = true;
        due += spacebox_typing.hold_ns;
    } else {
        spacebox_typing.up_next = false;
        if (++spacebox_typing.i == spacebox_typing.n) {
            fprintf(stderr, "[SPACEBOX-INPUT-TEST] typing done: %d keys\n", spacebox_typing.n);
            return;
        }
        due += spacebox_typing.gap_ns;
    }
    timer_mod_ns(spacebox_typing.timer, due);
}

static void spacebox_typing_start(struct sdl2_console *scon, const char *line)
{
    int hold, gap, shift, i;
    char text[128];

    if (sscanf(line, "%*s %d %d %d %127s", &hold, &gap, &shift, text) != 4 ||
        hold < 1 || gap < 1 || (spacebox_typing.timer && timer_pending(spacebox_typing.timer))) {
        fprintf(stderr, "[SPACEBOX-INPUT-TEST] type: not taken: %s", line);
        return;
    }
    for (i = 0; text[i]; i++) {
        if (text[i] != '_' && (text[i] < 'a' || text[i] > 'z')) {
            fprintf(stderr, "[SPACEBOX-INPUT-TEST] type: not taken (a to z and _ only): %s", line);
            return;
        }
    }
    if (!spacebox_typing.timer) {
        spacebox_typing.timer = timer_new_ns(QEMU_CLOCK_REALTIME, spacebox_typing_tick, NULL);
    }
    spacebox_typing.scon = scon;
    g_strlcpy(spacebox_typing.text, text, sizeof(spacebox_typing.text));
    spacebox_typing.n = strlen(text);
    spacebox_typing.i = 0;
    spacebox_typing.up_next = false;
    spacebox_typing.shift = shift;
    spacebox_typing.hold_ns = hold * 1000000LL;
    spacebox_typing.gap_ns = gap * 1000000LL;
    spacebox_typing.start_ns = qemu_clock_get_ns(QEMU_CLOCK_REALTIME) + 1000000;
    fprintf(stderr, "[SPACEBOX-INPUT-TEST] typing %d keys, hold %d ms, gap %d ms, shift %d\n",
            spacebox_typing.n, hold, gap, shift);
    timer_mod_ns(spacebox_typing.timer, spacebox_typing.start_ns);
}

/* Synthetic events in this SDL application's queue. Opt-in local validation;
 * does not post events to macOS or inspect any other application. */
static void spacebox_input_test(struct sdl2_console *scon)
{
    const char *path = getenv("SPACEBOX_INPUT_TEST");
    if (!path || !scon->real_window) return;
    g_autofree char *text = NULL;
    if (!spacebox_hook_take(path, &text)) return;
    char line[256] = "", command[24];
    int a = 0, b = 0, mod = 0;
    /* its first line, as fgets reads it */
    const char *nl = strchr(text, '\n');
    g_strlcpy(line, text, MIN(sizeof(line), (nl ? (size_t)(nl - text) + 1 : strlen(text)) + 1));
    int count = sscanf(line, "%23s %d %d %d", command, &a, &b, &mod);
    if (count < 1) return;
    if (!strcmp(command, "type")) {
        spacebox_typing_start(scon, line);
        return;
    }
    static int test_x, test_y;
    SDL_Event ev = {0};
    Uint32 id = SDL_GetWindowID(scon->real_window);
    if (!strcmp(command, "key") && (count == 3 || count == 4)) {
        /* "key SCANCODE DOWN [MOD]": MOD, the modifiers held (SDL_Keymod), as a
         * person's key event carries them */
        ev.type = b ? SDL_KEYDOWN : SDL_KEYUP;
        ev.key.windowID = id;
        ev.key.state = b ? SDL_PRESSED : SDL_RELEASED;
        ev.key.keysym.scancode = a;
        ev.key.keysym.sym = SDL_GetKeyFromScancode(a);
        ev.key.keysym.mod = mod;
        if (SDL_PushEvent(&ev) == 1) {
            spacebox_test_keys++;
        }
    } else if (!strcmp(command, "motion") && count == 3) {
        ev.type = SDL_MOUSEMOTION;
        ev.motion.windowID = id;
        test_x=a;test_y=b;
        ev.motion.x = a;
        ev.motion.y = b;
        if (SDL_PushEvent(&ev) == 1) {
            spacebox_test_motions++;
        }
    } else if (!strcmp(command, "button") && count == 3) {
        ev.type = b ? SDL_MOUSEBUTTONDOWN : SDL_MOUSEBUTTONUP;
        ev.button.windowID=id;ev.button.button=a;ev.button.state=b?SDL_PRESSED:SDL_RELEASED;
        ev.button.x=test_x;ev.button.y=test_y;ev.button.clicks=1;
        if (SDL_PushEvent(&ev) == 1) {
            spacebox_test_buttons++;
        }
    } else if (!strcmp(command, "resize") && count == 3 && a>=640 && b>=400) {
        SDL_SetWindowSize(scon->real_window,a,b);
    } else if (!strcmp(command, "focusgain")) {
        ev.type=SDL_WINDOWEVENT;ev.window.windowID=id;ev.window.event=SDL_WINDOWEVENT_FOCUS_GAINED;SDL_PushEvent(&ev);
    } else if (!strcmp(command, "focuslost")) {
        ev.type = SDL_WINDOWEVENT;
        ev.window.windowID = id;
        ev.window.event = SDL_WINDOWEVENT_FOCUS_LOST;
        SDL_PushEvent(&ev);
    } else if (!strcmp(command, "raise")) {
        SDL_RaiseWindow(scon->real_window);
    } else if (!strcmp(command, "close")) {
        /* as a click on the window's close button */
        ev.type = SDL_WINDOWEVENT;
        ev.window.windowID = id;
        ev.window.event = SDL_WINDOWEVENT_CLOSE;
        SDL_PushEvent(&ev);
    } else if (!strcmp(command, "quit")) {
        /* as the application's Quit (Cmd-Q) */
        ev.type = SDL_QUIT;
        SDL_PushEvent(&ev);
    }
    int w, h, pw, ph;
    SDL_GetWindowSize(scon->real_window, &w, &h);
    SDL_GL_GetDrawableSize(scon->real_window, &pw, &ph);
    fprintf(stderr, "[SPACEBOX-INPUT-TEST] command=%s a=%d b=%d absolute=%d grab=%d mouse_grab=%d logical=%dx%d drawable=%dx%d\n",
            command, a, b, qemu_input_is_absolute(scon->dcl.con), gui_grab,
            SDL_GetWindowGrab(scon->real_window), w, h, pw, ph);
}
