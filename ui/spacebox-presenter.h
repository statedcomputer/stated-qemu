/* Experimental Mac presenter: private bounded snapshots, shared GL contexts.
 * Guest processing never waits for display refresh or a consumer GL fence.
 * The display-link callback only signals a condition variable.
 * On Windows the same presenter, with the display's timing from Windows (the
 * parts under CONFIG_WIN32 below: a thread that waits for the vertical blank
 * plays the display link's part). */
#if defined(CONFIG_DARWIN) || defined(CONFIG_WIN32)
#define SPACEBOX_PRESENTER 1
#ifdef CONFIG_DARWIN
#include <CoreVideo/CoreVideo.h>
#include <CoreGraphics/CoreGraphics.h>
#include <mach/mach_time.h>
#include <pthread/qos.h>
#else
#define COBJMACROS
#include <dxgi.h>
#include <dwmapi.h>
#include <SDL_syswm.h>
#endif
#include "qemu/thread.h"
#include "qemu/atomic.h"
#include "ui/spacebox-video-record.h"
#include "spacebox-frame-code.h"
/* Frames kept ahead of the one on screen. Without pacing only a few are ever
 * in use (their textures are made on first use); with pacing up to 24
 * (hw/display/virtio-gpu-remote.c, sp_sink_pace_frame). */
#define SP_SLOTS 32
/* hw/display/virtio-gpu-remote.c: when to show the frame being submitted, 0 = at once */
extern int64_t spacebox_frame_due_us;
extern int64_t spacebox_frame_gap_us;
enum SpSlotState { SP_FREE, SP_WRITING, SP_READY, SP_READING, SP_DISPLAYED };
typedef struct SpSlot {
    egl_fb fb;
    GLsync ready, consumed;
    enum SpSlotState state;
    uint64_t sequence;
    int64_t submitted;
    int64_t due;    /* show no earlier than this, 0 = at once */
    int64_t gap;    /* usual spacing of the stream this frame belongs to, 0 = none */
    /* the record of video frames: the tickets of the video pictures the frame
     * shows, and the size of the picture it was made from (the scanout) */
    uint64_t tickets[SP_REC_TICKETS];
    unsigned ntickets;
    int64_t flush_dev, flush_read;  /* its flush's message: sent, read */
    int src_w, src_h;
    int64_t show_due;   /* due, or the due of the frame it replaces (sp_pair_updates) */
    bool superseded;    /* replaced by the next frame (sp_pair_updates) */
} SpSlot;
typedef struct SpPresenter {
    SDL_Window *window;
    SDL_GLContext output;
    QemuThread thread;
    QemuMutex lock;
    QemuCond wake;
#ifdef CONFIG_DARWIN
    CVDisplayLinkRef link;
    CGDirectDisplayID display;  /* the display the link follows: the window's */
#else
    QemuThread vblank;          /* waits for the vertical blanks (the link's part) */
    bool vblank_stop;
    IDXGIFactory1 *dxgi;
    IDXGIOutput *dxgi_output;   /* the display the thread follows: the window's (lock) */
    HMONITOR monitor;
#endif
    int64_t display_checked;
    bool stop;
    uint64_t tick, sequence, presented, discarded, busy, superseded;
    int64_t shown_due;  /* the show_due of the frame on screen, 0 = none */
    /* the last tick of the display link (host time, mach_absolute_time;
     * on Windows QueryPerformanceCounter): the refresh it came at and the
     * refresh it prepares for */
    uint64_t tick_now_host, tick_out_host;
    SpSlot slots[SP_SLOTS];
} SpPresenter;

#ifdef CONFIG_DARWIN
static CVReturn sp_present_tick(CVDisplayLinkRef link, const CVTimeStamp *now,
                               const CVTimeStamp *out, CVOptionFlags in,
                               CVOptionFlags *flags, void *opaque)
{
    SpPresenter *p = opaque;
    qemu_mutex_lock(&p->lock);
    p->tick++;
    p->tick_now_host = now->hostTime;
    p->tick_out_host = out->hostTime;
    qemu_cond_signal(&p->wake);
    qemu_mutex_unlock(&p->lock);
    return kCVReturnSuccess;
}

/*
 * The record of video frames: at which refresh of the display a swap is shown
 * (t_show_us), from the display link's own times of the window's display. The
 * link's tick before the swap gives the refresh it came at (vsync) and the
 * one that what is made now will be shown at (out); a swap that starts k
 * refreshes after that vsync is shown k refreshes after out. Its accuracy is
 * measured against a capture of the screen (screen/README.md, "The refresh
 * time of a swap"). 0 when the link has not ticked.
 */
static int64_t sp_present_host_us(uint64_t host)
{
    static mach_timebase_info_data_t tb;
    static int64_t off;     /* g_get_monotonic_time() - the host time in us */
    static int64_t off_at;
    int64_t now = g_get_monotonic_time();

    if (!tb.denom) {
        mach_timebase_info(&tb);
    }
    if (!off_at || now - off_at > G_USEC_PER_SEC) {
        /* both clocks read at once; again each second (one may count sleep) */
        off = now - (int64_t)(mach_absolute_time() * tb.numer / tb.denom / 1000);
        off_at = now;
    }
    return (int64_t)(host * tb.numer / tb.denom / 1000) + off;
}

/* The display link follows the display that shows the window (checked every
 * two seconds, on the thread that submits frames, which may call SDL). */
static void sp_present_follow_window(SpPresenter *p)
{
    int64_t now = g_get_monotonic_time();
    int index = SDL_GetWindowDisplayIndex(p->window);
    SDL_Rect b;
    CGDirectDisplayID ids[16];
    uint32_t n = 0;

    if (now - p->display_checked < 2 * G_USEC_PER_SEC) {
        return;
    }
    p->display_checked = now;
    if (index < 0 || SDL_GetDisplayBounds(index, &b) ||
        CGGetActiveDisplayList(16, ids, &n) != kCGErrorSuccess) {
        return;
    }
    for (uint32_t i = 0; i < n; i++) {
        CGRect r = CGDisplayBounds(ids[i]);

        if ((int)r.origin.x == b.x && (int)r.origin.y == b.y &&
            (int)r.size.width == b.w && (int)r.size.height == b.h) {
            if (ids[i] != p->display &&
                CVDisplayLinkSetCurrentCGDisplay(p->link, ids[i]) == kCVReturnSuccess) {
                fprintf(stderr, "[SPACEBOX-PRESENT] the window is on display %u: "
                        "the display link follows it\n", ids[i]);
                p->display = ids[i];
            }
            return;
        }
    }
}

/* the display link's nominal refresh, 0 = not known */
static int64_t sp_display_refresh_us(SpPresenter *p)
{
    CVTime period = CVDisplayLinkGetNominalOutputVideoRefreshPeriod(p->link);

    if (!(period.flags & kCVTimeIsIndefinite) && period.timeScale > 0 && period.timeValue > 0) {
        return period.timeValue * 1000000 / period.timeScale;
    }
    return 0;
}

static void sp_present_thread_priority(void)
{
    pthread_set_qos_class_self_np(QOS_CLASS_USER_INITIATED, 0);
}

/* the counter of the Windows record (qpc_call, qpc_ret): none here */
static int64_t sp_present_qpc(void)
{
    return 0;
}

static bool sp_display_open(SpPresenter *p)
{
    CVReturn rc = CVDisplayLinkCreateWithCGDisplay(CGMainDisplayID(), &p->link);

    if (rc) {
        fprintf(stderr, "[SPACEBOX-PRESENT-ERROR] display link: %d\n", rc);
        return false;
    }
    p->display = CGMainDisplayID();
    return true;
}

static void sp_display_start(SpPresenter *p)
{
    CVDisplayLinkSetOutputCallback(p->link, sp_present_tick, p);
}

static void sp_display_run(SpPresenter *p)
{
    CVDisplayLinkStart(p->link);
    fprintf(stderr, "[SPACEBOX-PRESENT] mode=thread slots=%d display=%u nonblocking_producer=1\n", SP_SLOTS, CGMainDisplayID());
}

static void sp_display_stop(SpPresenter *p)
{
    CVDisplayLinkStop(p->link);
}

static void sp_display_release(SpPresenter *p)
{
    CVDisplayLinkRelease(p->link);
}
#else
/*
 * Windows: the display's vertical blank from DXGI (IDXGIOutput::WaitForVBlank
 * of the output that shows the window), waited for by a thread of its own that
 * plays the display link's part, and the refresh and the time of the vertical
 * blank from the DWM's composition timing (DwmGetCompositionTimingInfo); host
 * times are QueryPerformanceCounter's counts. The refresh that what is made
 * now is shown at (the link's "out") is taken as the next one.
 */
static int64_t sp_qpc_us(uint64_t c)
{
    static LARGE_INTEGER f;

    if (!f.QuadPart) {
        QueryPerformanceFrequency(&f);
    }
    return (int64_t)(c / f.QuadPart) * 1000000 + (int64_t)(c % f.QuadPart) * 1000000 / f.QuadPart;
}

static int64_t sp_present_qpc(void)
{
    LARGE_INTEGER c;

    QueryPerformanceCounter(&c);
    return c.QuadPart;
}

static int64_t sp_present_host_us(uint64_t host)
{
    static int64_t off;     /* g_get_monotonic_time() - the host time in us */
    static int64_t off_at;
    int64_t now = g_get_monotonic_time();

    if (!off_at || now - off_at > G_USEC_PER_SEC) {
        /* both clocks read at once; again each second */
        off = now - sp_qpc_us(sp_present_qpc());
        off_at = now;
    }
    return sp_qpc_us(host) + off;
}

static void *sp_present_vblank_thread(void *opaque)
{
    SpPresenter *p = opaque;

    SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_HIGHEST);
    while (!qatomic_read(&p->vblank_stop)) {
        IDXGIOutput *o;
        DWM_TIMING_INFO ti = { .cbSize = sizeof(ti) };
        uint64_t now, vblank, period = 0;
        HRESULT hr = E_FAIL;

        qemu_mutex_lock(&p->lock);
        o = p->dxgi_output;
        if (o) {
            IDXGIOutput_AddRef(o);
        }
        qemu_mutex_unlock(&p->lock);
        if (o) {
            hr = IDXGIOutput_WaitForVBlank(o);
            IDXGIOutput_Release(o);
        }
        if (FAILED(hr) && FAILED(DwmFlush())) {
            /* no display to wait for (none, or locked): the nominal refresh */
            g_usleep(16667);
        }
        now = sp_present_qpc();
        vblank = now;
        if (SUCCEEDED(DwmGetCompositionTimingInfo(NULL, &ti)) && ti.qpcRefreshPeriod) {
            period = ti.qpcRefreshPeriod;
            /* the DWM's time of this vertical blank, when it has it already */
            if (ti.qpcVBlank <= now && now - ti.qpcVBlank < period / 2) {
                vblank = ti.qpcVBlank;
            }
        }
        qemu_mutex_lock(&p->lock);
        p->tick++;
        p->tick_now_host = vblank;
        p->tick_out_host = period ? vblank + period : 0;
        qemu_cond_signal(&p->wake);
        qemu_mutex_unlock(&p->lock);
    }
    return NULL;
}

/* The thread follows the display that shows the window (checked every two
 * seconds, on the thread that submits frames, which may call SDL). */
static void sp_present_follow_window(SpPresenter *p)
{
    int64_t now = g_get_monotonic_time();
    SDL_SysWMinfo wm;
    HMONITOR mon;
    IDXGIAdapter1 *a;
    IDXGIOutput *o;

    if (now - p->display_checked < 2 * G_USEC_PER_SEC) {
        return;
    }
    p->display_checked = now;
    SDL_VERSION(&wm.version);
    if (!p->dxgi || !SDL_GetWindowWMInfo(p->window, &wm) || wm.subsystem != SDL_SYSWM_WINDOWS) {
        return;
    }
    mon = MonitorFromWindow(wm.info.win.window, MONITOR_DEFAULTTONEAREST);
    if (mon == p->monitor) {
        return;
    }
    for (UINT i = 0; IDXGIFactory1_EnumAdapters1(p->dxgi, i, &a) == S_OK; i++) {
        for (UINT j = 0; IDXGIAdapter1_EnumOutputs(a, j, &o) == S_OK; j++) {
            DXGI_OUTPUT_DESC d;

            if (SUCCEEDED(IDXGIOutput_GetDesc(o, &d)) && d.Monitor == mon) {
                IDXGIOutput *old;
                g_autofree char *name = g_utf16_to_utf8((const gunichar2 *)d.DeviceName, -1,
                                                        NULL, NULL, NULL);

                qemu_mutex_lock(&p->lock);
                old = p->dxgi_output;
                p->dxgi_output = o;
                p->monitor = mon;
                qemu_mutex_unlock(&p->lock);
                if (old) {
                    IDXGIOutput_Release(old);
                }
                IDXGIAdapter1_Release(a);
                fprintf(stderr, "[SPACEBOX-PRESENT] the window is on display %s: "
                        "the vertical blank follows it\n", name ? name : "?");
                return;
            }
            IDXGIOutput_Release(o);
        }
        IDXGIAdapter1_Release(a);
    }
}

/* the DWM's refresh, 0 = not known */
static int64_t sp_display_refresh_us(SpPresenter *p)
{
    DWM_TIMING_INFO ti = { .cbSize = sizeof(ti) };

    if (SUCCEEDED(DwmGetCompositionTimingInfo(NULL, &ti)) && ti.qpcRefreshPeriod) {
        return sp_qpc_us(ti.qpcRefreshPeriod);
    }
    return 0;
}

/* the Mac's QoS class user-initiated: above the guest's work, below the
 * thread that waits for the vertical blank */
static void sp_present_thread_priority(void)
{
    SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_ABOVE_NORMAL);
}

static bool sp_display_open(SpPresenter *p)
{
    HRESULT hr = CreateDXGIFactory1(&IID_IDXGIFactory1, (void **)&p->dxgi);

    if (FAILED(hr)) {
        /* the thread still ticks, at the DWM's composition or the nominal refresh */
        fprintf(stderr, "[SPACEBOX-PRESENT-ERROR] DXGI factory: 0x%lx\n", (unsigned long)hr);
        p->dxgi = NULL;
    }
    return true;
}

static void sp_display_start(SpPresenter *p)
{
}

static void sp_display_run(SpPresenter *p)
{
    qemu_thread_create(&p->vblank, "sp-vblank", sp_present_vblank_thread, p, QEMU_THREAD_JOINABLE);
    fprintf(stderr, "[SPACEBOX-PRESENT] mode=thread slots=%d display=dxgi refresh_us=%lld nonblocking_producer=1\n",
            SP_SLOTS, (long long)sp_display_refresh_us(p));
}

static void sp_display_stop(SpPresenter *p)
{
    qatomic_set(&p->vblank_stop, true);
    qemu_thread_join(&p->vblank);
}

static void sp_display_release(SpPresenter *p)
{
    if (p->dxgi_output) {
        IDXGIOutput_Release(p->dxgi_output);
    }
    if (p->dxgi) {
        IDXGIFactory1_Release(p->dxgi);
    }
}
#endif

static int64_t sp_present_show_time(uint64_t now_host, uint64_t out_host,
                                    int64_t call, int64_t refresh_us,
                                    int64_t *vsync, int64_t *out)
{
    int64_t k;

    if (!now_host || !out_host || refresh_us <= 0) {
        *vsync = *out = 0;
        return 0;
    }
    *vsync = sp_present_host_us(now_host);
    *out = sp_present_host_us(out_host);
    k = call > *vsync ? (call - *vsync) / refresh_us : 0;
    return *out + k * refresh_us;
}

static bool sp_fence_done(GLsync fence)
{
    if (!fence) return true;
    GLenum r = glClientWaitSync(fence, 0, 0);
    return r == GL_ALREADY_SIGNALED || r == GL_CONDITION_SATISFIED;
}

/*
 * The record of video frames: the frame code in the frame about to be shown,
 * at the top of the video's rectangle (the control's video-rect, in the
 * scanout's pixels). The slot holds the scanout scaled to the window, bottom
 * row first. Three rows across the central half of the cells' height are read
 * back from the slot's texture (readfb); -1 when there is no rectangle or it
 * is not inside the frame.
 */
static int sp_present_code(SpSlot *s, bool *ok, int64_t *read_us)
{
    static uint8_t *buf;
    static size_t buf_size;
    const uint8_t *rows[3];
    int x, y, w, h, code;
    int64_t t0 = g_get_monotonic_time();

    *ok = false;
    *read_us = -1;
    if (!spacebox_video_rect(&x, &y, &w, &h) || !s->src_w || !s->src_h) {
        return -1;
    }
    double sx = (double)s->fb.width / s->src_w, sy = (double)s->fb.height / s->src_h;
    int left = (int)(x * sx), width = (int)(w * sx);
    double cell = w / (double)SP_CODE_CELLS;    /* in the scanout's pixels */

    if (width < SP_CODE_CELLS || left < 0 || left + width > s->fb.width ||
        (y + cell) * sy > s->fb.height) {
        return -1;
    }
    if (buf_size < (size_t)width * 4 * 3) {
        buf_size = (size_t)width * 4 * 3;
        buf = g_realloc(buf, buf_size);
    }
    for (int k = 0; k < 3; k++) {
        int row = (int)((y + cell * (3 + k) / 8) * sy);

        rows[k] = buf + (size_t)k * width * 4;
        glReadPixels(left, s->fb.height - 1 - row, width, 1, GL_RGBA,
                     GL_UNSIGNED_BYTE, (void *)rows[k]);
    }
    code = spacebox_frame_code_read(rows, 3, width, ok);
    *read_us = g_get_monotonic_time() - t0;
    return code;
}

/*
 * Two paced frames of a stream one refresh apart, where the frame after them
 * comes within the stream's usual spacing (and half a refresh) of the first,
 * are a frame of the stream and an update of the page's other drawing (an
 * animation of the page that plays the video, in its own rhythm), in either
 * order, the first of them on time (within the spacing of the frame before
 * it). The later one replaces the earlier one at the earlier one's time: it
 * was drawn later, so it shows the newer video frame and everything else.
 * Shown as two frames, the update took the place of a frame of the stream in
 * the spacing below and held the next video frame back by a refresh: 25 ms
 * where 16.7 ms was due, 2 to 4% of a real YouTube video's frames at 60/s
 * (the video task, turn G4c). A frame that the guest sent a refresh early,
 * with the next one two spacings after the first, is not replaced (the
 * spacing below holds it), nor is a late one that the next ones follow a
 * refresh apart to catch up (each of them is a video frame). Sets show_due
 * and superseded of the ready slots.
 */
static void sp_pair_updates(SpPresenter *p, int64_t refresh_us)
{
    SpSlot *ord[SP_SLOTS];
    int n = 0;

    for (int i = 0; i < SP_SLOTS; i++) {
        SpSlot *s = &p->slots[i];

        if (s->state != SP_READY) {
            continue;
        }
        s->show_due = s->due;
        s->superseded = false;
        int k = n++;
        while (k > 0 && ord[k - 1]->sequence > s->sequence) {
            ord[k] = ord[k - 1];
            k--;
        }
        ord[k] = s;
    }
    int64_t prev = p->shown_due;    /* the frame before the next one */

    for (int i = 0; i < n; i++) {
        SpSlot *f = ord[i], *g = i + 1 < n ? ord[i + 1] : NULL, *h = i + 2 < n ? ord[i + 2] : NULL;

        if (h && f->due && f->gap && g->due && g->gap && h->due &&
            (!prev || f->due - prev < f->gap + refresh_us / 2) &&
            g->due > f->due && g->due - f->due < f->gap - refresh_us / 2 &&
            h->due - f->due < f->gap + refresh_us / 2) {
            f->superseded = true;
            g->show_due = f->show_due;
            i++;    /* the frame that replaced one does not pair again */
            f = g;
        }
        prev = f->show_due;
    }
}

static bool sp_has_ready(SpPresenter *p)
{
    for (int i = 0; i < SP_SLOTS; i++) if (p->slots[i].state == SP_READY) return true;
    return false;
}

static void *sp_present_thread(void *opaque)
{
    SpPresenter *p = opaque;
    uint64_t seen = 0, record_discarded = 0;
    int shown = -1;
    GLuint readfb;
    int64_t last_stats = 0, last_control = 0;
    const char *scheduler = getenv("SPACEBOX_PRESENT_SCHEDULER");
    bool native = scheduler && !strcmp(scheduler, "native");
    const char *phase_env = getenv("SPACEBOX_PRESENT_PHASE_US");
    int phase_us = phase_env ? atoi(phase_env) : 0;
    const char *phase_file = getenv("SPACEBOX_PRESENT_PHASE_CONTROL");
    /* SPACEBOX_PRESENT_PROBE=<file>: one line per frame shown, "time_us r g b" */
    FILE *probe = getenv("SPACEBOX_PRESENT_PROBE") ? fopen(getenv("SPACEBOX_PRESENT_PROBE"), "w") : NULL;
    /* SPACEBOX_PRESENT_STATS=<seconds between lines> */
    const char *stats_env = getenv("SPACEBOX_PRESENT_STATS");
    int64_t stats_us = stats_env ? (int64_t)MAX(atoi(stats_env), 1) * 1000000 : 0;
    int64_t refresh_us = 8333, last_swap = 0, held_max = 0, phase_adj = 0;
    /* where between two refreshes a frame's time is kept: this fraction of a
     * refresh before the one it is shown at (SPACEBOX_PRESENT_PHASE_TARGET) */
    double phase_target = getenv("SPACEBOX_PRESENT_PHASE_TARGET") ? atof(getenv("SPACEBOX_PRESENT_PHASE_TARGET")) : 0.5;
    bool waited_for_due = false;
    uint64_t held[7] = { 0 };
    {
        int64_t nominal = sp_display_refresh_us(p);
        if (nominal > 0) {
            refresh_us = nominal;
        }
    }
    sp_present_thread_priority();
    if (SDL_GL_MakeCurrent(p->window, p->output)) {
        fprintf(stderr, "[SPACEBOX-PRESENT-ERROR] make_current: %s\n", SDL_GetError());
        return NULL;
    }
    const char *interval = getenv("SPACEBOX_PRESENT_INTERVAL");
    SDL_GL_SetSwapInterval(interval ? atoi(interval) : 1);
    glGenFramebuffers(1, &readfb);
    for (;;) {
        qemu_mutex_lock(&p->lock);
        while (!p->stop && (native ? !sp_has_ready(p) : p->tick == seen)) qemu_cond_wait(&p->wake, &p->lock);
        if (p->stop) { qemu_mutex_unlock(&p->lock); break; }
        seen = p->tick;
        qemu_mutex_unlock(&p->lock);
        int64_t check = g_get_monotonic_time();
        if (phase_file && check - last_control > 500000) {
            FILE *f = fopen(phase_file, "r");
            int next, scheduler_mode = -1;
            if (f) {
                int fields = fscanf(f, "%d %d", &next, &scheduler_mode);
                if (fields >= 1 && next >= 0 && next <= 7000) phase_us = next;
                if (fields >= 2 && (scheduler_mode == 0 || scheduler_mode == 1)) native = scheduler_mode == 1;
                fclose(f);
            }
            last_control = check;
        }
        if (phase_us > 0 && phase_us <= 7000) g_usleep(phase_us);
        qemu_mutex_lock(&p->lock);
        if (p->stop) { qemu_mutex_unlock(&p->lock); break; }
        int pick = -1, oldest = -1, newest = -1, n_due = 0;
        bool stream = true;
        int64_t pick_now = g_get_monotonic_time(), next_due = 0;
        sp_pair_updates(p, refresh_us);
        for (int i = 0; i < SP_SLOTS; i++) {
            SpSlot *s = &p->slots[i];
            if (s->state != SP_READY || s->superseded || !sp_fence_done(s->ready)) continue;
            int64_t due = s->show_due ? s->show_due + phase_adj : 0;
            if (due > pick_now) {
                if (!next_due || due < next_due) next_due = due;
                continue;
            }
            n_due++;
            if (!s->due || !s->gap) stream = false;
            if (oldest < 0 || s->sequence < p->slots[oldest].sequence) oldest = i;
            if (newest < 0 || s->sequence > p->slots[newest].sequence) newest = i;
        }
        if (n_due && (!stream || n_due >= 4)) {
            /* Frames without a time, or far behind: the newest one, now. */
            pick = newest;
        } else if (n_due) {
            /*
             * A paced stream: in order, and not sooner after the previous
             * frame than the stream's usual spacing in whole refreshes. The
             * swap lands on the refresh after the wake-up, so wake just after
             * the one before it (last_swap is a little after a refresh). When
             * a second frame's time has come as well, one refresh sooner, so
             * that being late is made up by a shorter frame instead of a
             * dropped one.
             */
            SpSlot *s = &p->slots[oldest];
            int64_t k = MAX((s->gap + refresh_us / 2) / refresh_us, 1);
            if (n_due >= 2 && k > 1) k--;
            int64_t earliest = last_swap ? last_swap + (k - 1) * refresh_us + refresh_us / 8 : 0;
            if (earliest > pick_now) {
                if (!next_due || earliest < next_due) next_due = earliest;
            } else {
                pick = oldest;
            }
        }
        if (pick < 0) {
            if (next_due) {
                waited_for_due = n_due == 0;
                qemu_cond_timedwait(&p->wake, &p->lock, (int)MAX((next_due - pick_now + 500) / 1000, 1));
            } else if (native) {
                qemu_cond_timedwait(&p->wake, &p->lock, 1);
            }
            qemu_mutex_unlock(&p->lock); continue;
        }
        int64_t pick_due = p->slots[pick].show_due ? p->slots[pick].show_due + phase_adj : 0;
        bool pick_waited = waited_for_due && pick_due;
        waited_for_due = false;
        SpSlot *s = &p->slots[pick];
        s->state = SP_READING;
        p->shown_due = s->show_due;
        if (s->ready) { glDeleteSync(s->ready); s->ready = NULL; }
        for (int i = 0; i < SP_SLOTS; i++) {
            if (i != pick && p->slots[i].state == SP_READY &&
                p->slots[i].sequence < s->sequence) {
                p->slots[i].state = SP_FREE;
                p->discarded++;
                p->superseded += p->slots[i].superseded;
            }
        }
        qemu_mutex_unlock(&p->lock);
        /* Only this thread touches the window's default framebuffer. */
        glBindFramebuffer(GL_READ_FRAMEBUFFER, readfb);
        glFramebufferTexture2D(GL_READ_FRAMEBUFFER, GL_COLOR_ATTACHMENT0,
                               GL_TEXTURE_2D, s->fb.texture, 0);
        glReadBuffer(GL_COLOR_ATTACHMENT0);
        bool recording = qatomic_read(&spacebox_video_record_on), code_ok = false;
        int64_t code_us = -1;
        int code = recording ? sp_present_code(s, &code_ok, &code_us) : -1;
        glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);
        glDrawBuffer(GL_BACK);
        glViewport(0, 0, s->fb.width, s->fb.height);
        glDisable(GL_SCISSOR_TEST);
        glBlitFramebuffer(0, 0, s->fb.width, s->fb.height,
                          0, 0, s->fb.width, s->fb.height,
                          GL_COLOR_BUFFER_BIT, GL_NEAREST);
        spacebox_capture_client(s->fb.width, s->fb.height);
        uint8_t probe_px[4] = { 0 };
        if (probe) {
            /* test aid: the colour at the centre of each frame shown */
            glBindFramebuffer(GL_READ_FRAMEBUFFER, readfb);
            glReadPixels(s->fb.width / 2, s->fb.height / 2, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, probe_px);
        }
        int64_t swap_call = g_get_monotonic_time(), vsync_us = 0, out_us = 0, show_us = 0;
        int64_t qpc_call = sp_present_qpc(), qpc_ret;
        uint64_t now_host = 0, out_host = 0;
        if (recording) {
            qemu_mutex_lock(&p->lock);
            now_host = p->tick_now_host;
            out_host = p->tick_out_host;
            qemu_mutex_unlock(&p->lock);
        }
        spacebox_swap(p->window, s->fb.width, s->fb.height);
        int64_t swapped = g_get_monotonic_time();
        qpc_ret = sp_present_qpc();
        if (recording) {
            show_us = sp_present_show_time(now_host, out_host, swap_call, refresh_us,
                                           &vsync_us, &out_us);
        }
        if (probe) {
            fprintf(probe, "%lld %u %u %u\n", (long long)g_get_monotonic_time(), probe_px[0], probe_px[1], probe_px[2]);
        }
        GLsync consumed = glFenceSync(GL_SYNC_GPU_COMMANDS_COMPLETE, 0);
        glFlush();
        if (pick_waited) {
            /*
             * The swap returned at a display refresh. Keep the frames' times in
             * the middle between two refreshes, so that small timing noise does
             * not move a frame to the refresh before or after.
             */
            int64_t x = g_get_monotonic_time() - pick_due;
            if (x >= 0 && x < 2 * refresh_us) {
                phase_adj += (x - (int64_t)(refresh_us * phase_target)) / 16;
                phase_adj = MAX(-refresh_us, MIN(phase_adj, refresh_us));
            }
        }
        {
            /* How long each guest frame stayed on screen, in display refreshes. */
            int64_t t = g_get_monotonic_time();
            if (stats_us && last_swap) {
                int64_t dt = t - last_swap;
                int ticks = (int)((dt + refresh_us / 2) / refresh_us);
                ticks = ticks < 1 ? 1 : ticks > 6 ? 6 : ticks;
                held[ticks]++;
                if (dt > held_max) held_max = dt;
            }
            last_swap = t;
        }
        qemu_mutex_lock(&p->lock);
        if (shown >= 0) p->slots[shown].state = SP_FREE;
        shown = pick;
        s->consumed = consumed;
        s->state = SP_DISPLAYED;
        p->presented++;
        uint64_t presented = p->presented, coalesced = p->discarded - record_discarded;
        record_discarded = p->discarded;
        int64_t now = g_get_monotonic_time();
        if (stats_us && now - last_stats > stats_us) {
            fprintf(stderr, "[SPACEBOX-PRESENT-STATS] us=%lld submitted=%llu presented=%llu coalesced=%llu busy=%llu"
                    " refresh_us=%lld held1=%llu held2=%llu held3=%llu held4=%llu held5=%llu held6plus=%llu held_max_ms=%.1f"
                    " superseded=%llu\n",
                    (long long)now, (unsigned long long)p->sequence,
                    (unsigned long long)p->presented, (unsigned long long)p->discarded,
                    (unsigned long long)p->busy, (long long)refresh_us,
                    (unsigned long long)held[1], (unsigned long long)held[2],
                    (unsigned long long)held[3], (unsigned long long)held[4],
                    (unsigned long long)held[5], (unsigned long long)held[6],
                    held_max / 1000.0, (unsigned long long)p->superseded);
            memset(held, 0, sizeof(held));
            held_max = 0;
            last_stats = now;
        }
        qemu_mutex_unlock(&p->lock);
        if (recording) {
            spacebox_video_record_swap(presented, swapped, code, code_ok, s->tickets,
                                       s->ntickets, coalesced, code_us, show_us,
                                       swap_call, vsync_us, out_us, pick_due,
                                       s->flush_dev, s->flush_read, qpc_call, qpc_ret);
        }
        int pause_ms = qatomic_xchg(&spacebox_present_pause_ms, 0);
        if (pause_ms) {
            /* the test pause (pause-present): nothing is shown for pause_ms */
            int64_t until = swapped + (int64_t)pause_ms * 1000, t;
            qemu_mutex_lock(&p->lock);
            while (!p->stop && (t = g_get_monotonic_time()) < until) {
                qemu_cond_timedwait(&p->wake, &p->lock, (int)MAX((until - t) / 1000, 1));
            }
            qemu_mutex_unlock(&p->lock);
            spacebox_video_record_paused(pause_ms, swapped, g_get_monotonic_time());
        }
    }
    glFinish();
    glDeleteFramebuffers(1, &readfb);
    SDL_GL_MakeCurrent(p->window, NULL);
    return NULL;
}

void sdl2_gl_present_init(struct sdl2_console *scon)
{
    const char *mode = getenv("SPACEBOX_PRESENT_MODE");
    if (!mode || strcmp(mode, "thread")) return;
    SDL_GL_SetAttribute(SDL_GL_SHARE_WITH_CURRENT_CONTEXT, 1);
    scon->renderctx = SDL_GL_CreateContext(scon->real_window);
    if (!scon->renderctx) {
        fprintf(stderr, "[SPACEBOX-PRESENT-ERROR] producer context: %s\n", SDL_GetError());
        SDL_GL_MakeCurrent(scon->real_window, scon->winctx);
        return;
    }
    SDL_GL_SetSwapInterval(0);
    SpPresenter *p = g_new0(SpPresenter, 1);
    p->window = scon->real_window;
    p->output = scon->winctx;
    qemu_mutex_init(&p->lock);
    qemu_cond_init(&p->wake);
    if (!sp_display_open(p)) {
        SDL_GL_DeleteContext(scon->renderctx); scon->renderctx = NULL;
        SDL_GL_MakeCurrent(scon->real_window, scon->winctx);
        qemu_cond_destroy(&p->wake); qemu_mutex_destroy(&p->lock); g_free(p);
        return;
    }
    scon->presenter = p;
    sp_present_follow_window(p);
    sp_display_start(p);
    qemu_thread_create(&p->thread, "sp-present", sp_present_thread, p, QEMU_THREAD_JOINABLE);
    sp_display_run(p);
}

static SpSlot *sp_present_acquire(struct sdl2_console *scon, int w, int h)
{
    SpPresenter *p = scon->presenter;
    SpSlot *pick = NULL;
    qemu_mutex_lock(&p->lock);
    for (int i = 0; i < SP_SLOTS; i++) {
        SpSlot *s = &p->slots[i];
        if (s->state == SP_FREE && sp_fence_done(s->consumed)) { pick = s; break; }
    }
    if (!pick) {
        for (int i = 0; i < SP_SLOTS; i++) {
            SpSlot *s = &p->slots[i];
            if (s->state == SP_READY && (!pick || s->sequence < pick->sequence)) pick = s;
        }
        if (pick) p->discarded++;
    }
    if (!pick) { p->busy++; qemu_mutex_unlock(&p->lock); return NULL; }
    pick->state = SP_WRITING;
    qemu_mutex_unlock(&p->lock);
    if (pick->ready) { glDeleteSync(pick->ready); pick->ready = NULL; }
    if (pick->consumed) { glDeleteSync(pick->consumed); pick->consumed = NULL; }
    if (pick->fb.width != w || pick->fb.height != h) {
        egl_fb_destroy(&pick->fb);
        egl_fb_setup_new_tex(&pick->fb, w, h);
    }
    return pick;
}

/* src_w x src_h: the size of the picture the frame was made from */
static void sp_present_submit(struct sdl2_console *scon, SpSlot *s, int src_w, int src_h)
{
    SpPresenter *p = scon->presenter;
    GLsync fence = glFenceSync(GL_SYNC_GPU_COMMANDS_COMPLETE, 0);
    glFlush();
    sp_present_follow_window(p);
    s->ntickets = MIN(spacebox_frame_ntickets, SP_REC_TICKETS);
    memcpy(s->tickets, spacebox_frame_tickets, s->ntickets * sizeof(s->tickets[0]));
    spacebox_frame_ntickets = 0;
    s->flush_dev = spacebox_frame_flush_dev;
    s->flush_read = spacebox_frame_flush_read;
    s->src_w = src_w;
    s->src_h = src_h;
    qemu_mutex_lock(&p->lock);
    s->ready = fence;
    s->sequence = ++p->sequence;
    s->submitted = g_get_monotonic_time();
    s->due = spacebox_frame_due_us;
    s->gap = spacebox_frame_gap_us;
    s->superseded = false;
    spacebox_frame_due_us = 0;
    spacebox_frame_gap_us = 0;
    s->state = SP_READY;
    qemu_cond_signal(&p->wake);
    qemu_mutex_unlock(&p->lock);
}

void sdl2_gl_present_destroy(struct sdl2_console *scon)
{
    SpPresenter *p = scon->presenter;
    if (!p) return;
    sp_display_stop(p);
    qemu_mutex_lock(&p->lock); p->stop = true; qemu_cond_signal(&p->wake); qemu_mutex_unlock(&p->lock);
    qemu_thread_join(&p->thread);
    SDL_GL_MakeCurrent(scon->real_window, scon->renderctx);
    glFinish();
    for (int i = 0; i < SP_SLOTS; i++) {
        if (p->slots[i].ready) glDeleteSync(p->slots[i].ready);
        if (p->slots[i].consumed) glDeleteSync(p->slots[i].consumed);
        egl_fb_destroy(&p->slots[i].fb);
    }
    sp_display_release(p);
    SDL_GL_MakeCurrent(scon->real_window, scon->winctx);
    SDL_GL_DeleteContext(scon->renderctx); scon->renderctx = NULL;
    qemu_cond_destroy(&p->wake); qemu_mutex_destroy(&p->lock);
    g_free(p); scon->presenter = NULL;
}
#else
void sdl2_gl_present_init(struct sdl2_console *scon) {}
void sdl2_gl_present_destroy(struct sdl2_console *scon) {}
#endif
