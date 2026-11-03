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

/*
 * "schedule FILE OUT DELAY_MS": the UI evals' schedule (dev/spacebox screen/ui/README.md, "The schedule") given
 * to this window without the OS: for a client where the OS's injection is not allowed. Each event comes at
 * start + DELAY_MS + t_ms on CLOCK_MONOTONIC (a timer of the main loop) and is then handed over at the next poll
 * of the window's events, as a person's event that comes at that time waits in the queue for that poll: a key
 * into SDL's queue (counted as this hook's own), a scroll as the window's event monitor gets a trackpad's
 * (spacebox-scroll.m). OUT gets the injector's lines (README, "The injector"): t_us the event's coming,
 * deliver_us its handing over, late_us how late the timer was; "via":"sink". The OS's path before the
 * application (the HID system, the window server, the application's event queue) is not in it.
 */
#if defined(CONFIG_DARWIN)
bool spacebox_scroll_schedule(double dy, unsigned long phase);
#endif

typedef struct {
    int id, sc;          /* sc: the SDL scancode of a key, 0 for a scroll */
    bool down, control;
    double dy;
    unsigned long phase;
    char key[8];
    int64_t due_us, came_us, late_us;
} SpSchedEv;

static struct {
    QEMUTimer *timer;
    FILE *out;
    SpSchedEv *ev;
    int n, next, given, pending0;
    int64_t t0_us, max_late_us;
} sp_sched;

static int64_t sp_mono_us(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1000000LL + ts.tv_nsec / 1000;
}

/* the value after "NAME": in one JSON line of the schedule (its writer's own form: no spaces) */
static const char *sp_jfield(const char *line, const char *name)
{
    char pat[32];
    snprintf(pat, sizeof(pat), "\"%s\":", name);
    const char *p = strstr(line, pat);
    return p ? p + strlen(pat) : NULL;
}

static bool sp_jstr(const char *line, const char *name, char *buf, size_t len)
{
    const char *p = sp_jfield(line, name);
    if (!p || *p != '"') return false;
    const char *e = strchr(p + 1, '"');
    if (!e || (size_t)(e - p - 1) >= len) return false;
    memcpy(buf, p + 1, e - p - 1);
    buf[e - p - 1] = 0;
    return true;
}

static void sp_sched_end(const char *why)
{
    if (sp_sched.out) {
        fprintf(sp_sched.out, "{\"k\":\"end\",\"t_us\":%" PRId64 ",\"given\":%d,\"of\":%d,\"max_late_us\":%" PRId64
                ",\"why\":\"%s\"}\n", sp_mono_us(), sp_sched.given, sp_sched.n, sp_sched.max_late_us, why);
        fclose(sp_sched.out);
        sp_sched.out = NULL;
    }
    g_free(sp_sched.ev);
    sp_sched.ev = NULL;
    sp_sched.n = sp_sched.next = sp_sched.pending0 = 0;
    if (sp_sched.timer) timer_del(sp_sched.timer);
    fprintf(stderr, "[SPACEBOX-INPUT-TEST] schedule %s: %d events given\n", why, sp_sched.given);
}

static void sp_sched_arm(void)
{
    if (sp_sched.next < sp_sched.n) {
        int64_t wait_ns = (sp_sched.ev[sp_sched.next].due_us - sp_mono_us()) * 1000;
        timer_mod_ns(sp_sched.timer, qemu_clock_get_ns(QEMU_CLOCK_REALTIME) + MAX(wait_ns, 0));
    }
}

/* the timer: each event whose time has come; it waits for the next poll */
static void sp_sched_tick(void *opaque)
{
    int64_t now = sp_mono_us();
    while (sp_sched.next < sp_sched.n && sp_sched.ev[sp_sched.next].due_us <= now) {
        SpSchedEv *e = &sp_sched.ev[sp_sched.next++];
        e->came_us = now;
        e->late_us = now - e->due_us;
        sp_sched.max_late_us = MAX(sp_sched.max_late_us, e->late_us);
    }
    sp_sched_arm();
}

/* the poll: the events that came since the last one, handed over in their order */
static void sp_sched_deliver(struct sdl2_console *scon)
{
    if (!sp_sched.ev) return;
    while (sp_sched.pending0 < sp_sched.next) {
        SpSchedEv *e = &sp_sched.ev[sp_sched.pending0++];
        int64_t at = sp_mono_us();
        bool ok;
        if (e->sc) {
            SDL_Event ev = {0};
            ev.type = e->down ? SDL_KEYDOWN : SDL_KEYUP;
            ev.key.windowID = SDL_GetWindowID(scon->real_window);
            ev.key.state = e->down ? SDL_PRESSED : SDL_RELEASED;
            ev.key.keysym.scancode = e->sc;
            ev.key.keysym.sym = SDL_GetKeyFromScancode(e->sc);
            ok = SDL_PushEvent(&ev) == 1;
            if (ok) spacebox_test_keys++;
        } else {
#if defined(CONFIG_DARWIN)
            ok = spacebox_scroll_schedule(e->dy, e->phase);
#else
            ok = false;
#endif
        }
        sp_sched.given += ok;
        if (e->sc) {
            fprintf(sp_sched.out, "{\"k\":\"inj\",\"id\":%d,\"t_us\":%" PRId64 ",\"type\":\"%s\",\"key\":\"%s\","
                    "\"down\":%s,\"late_us\":%" PRId64 ",\"deliver_us\":%" PRId64 ",\"tagged\":true,\"via\":\"sink\"%s}\n",
                    e->id, e->came_us, e->control ? "control" : "key", e->key, e->down ? "true" : "false",
                    e->late_us, at, ok ? "" : ",\"lost\":true");
        } else {
            fprintf(sp_sched.out, "{\"k\":\"inj\",\"id\":%d,\"t_us\":%" PRId64 ",\"type\":\"scroll\",\"dy\":%.9g,"
                    "\"late_us\":%" PRId64 ",\"deliver_us\":%" PRId64 ",\"tagged\":true,\"via\":\"sink\"%s}\n",
                    e->id, e->came_us, e->dy, e->late_us, at, ok ? "" : ",\"lost\":true");
        }
    }
    if (sp_sched.pending0 == sp_sched.n) {
        sp_sched_end("done");
    }
}

static void sp_sched_start(const char *line)
{
    char file[512], out[512];
    int delay;
    g_autofree char *text = NULL;

    if (sscanf(line, "%*s %511s %511s %d", file, out, &delay) != 3 || delay < 0) {
        fprintf(stderr, "[SPACEBOX-INPUT-TEST] schedule: not taken: %s", line);
        return;
    }
    if (sp_sched.ev) sp_sched_end("replaced");
    if (!g_file_get_contents(file, &text, NULL, NULL) || !(sp_sched.out = fopen(out, "w"))) {
        fprintf(stderr, "[SPACEBOX-INPUT-TEST] schedule: cannot read %s or write %s\n", file, out);
        return;
    }
    setvbuf(sp_sched.out, NULL, _IOLBF, 0);
    g_auto(GStrv) lines = g_strsplit(text, "\n", -1);
    sp_sched.ev = g_new0(SpSchedEv, g_strv_length(lines));
    sp_sched.n = sp_sched.next = sp_sched.given = sp_sched.pending0 = 0;
    sp_sched.max_late_us = 0;
    sp_sched.t0_us = sp_mono_us() + delay * 1000LL;
    for (int i = 0; lines[i]; i++) {
        const char *l = lines[i], *p;
        char type[16] = "", ph[16] = "";
        if (!strstr(l, "\"k\":\"ev\"") || !sp_jstr(l, "type", type, sizeof(type))) continue;
        SpSchedEv *e = &sp_sched.ev[sp_sched.n];
        if (!(p = sp_jfield(l, "id"))) continue;
        e->id = atoi(p);
        if (!(p = sp_jfield(l, "t_ms"))) continue;
        e->due_us = sp_sched.t0_us + (int64_t)(g_ascii_strtod(p, NULL) * 1000);
        if (!strcmp(type, "scroll")) {
            e->dy = (p = sp_jfield(l, "dy")) ? g_ascii_strtod(p, NULL) : 0;
            sp_jstr(l, "phase", ph, sizeof(ph));
            e->phase = !strcmp(ph, "begin") ? 1 : !strcmp(ph, "end") ? 8 : 4;
        } else {
            sp_jstr(l, "key", e->key, sizeof(e->key));
            e->down = (p = sp_jfield(l, "down")) && !strncmp(p, "true", 4);
            e->control = !strcmp(type, "control");
            if (strlen(e->key) == 1 && e->key[0] >= 'a' && e->key[0] <= 'z') e->sc = SDL_SCANCODE_A + (e->key[0] - 'a');
            else if (!strcmp(e->key, "F8")) e->sc = SDL_SCANCODE_F8;
            else if (!strcmp(e->key, "space")) e->sc = SDL_SCANCODE_SPACE;
            else {
                fprintf(stderr, "[SPACEBOX-INPUT-TEST] schedule: no scancode for key %s (event %d)\n", e->key, e->id);
                continue;
            }
        }
        sp_sched.n++;
    }
    fprintf(sp_sched.out, "{\"k\":\"hdr\",\"tool\":\"sink-schedule\",\"version\":1,\"clock\":\"CLOCK_MONOTONIC\","
            "\"t0_us\":%" PRId64 ",\"tag_magic\":\"0x53504556\",\"via\":\"sink\",\"events\":%d,\"schedule\":\"%s\"}\n",
            sp_sched.t0_us, sp_sched.n, file);
    if (!sp_sched.timer) sp_sched.timer = timer_new_ns(QEMU_CLOCK_REALTIME, sp_sched_tick, NULL);
    fprintf(stderr, "[SPACEBOX-INPUT-TEST] schedule: %d events from %s, start in %d ms\n", sp_sched.n, file, delay);
    if (!sp_sched.n) {
        sp_sched_end("empty");
        return;
    }
    sp_sched_arm();
}

/* Synthetic events in this SDL application's queue. Opt-in local validation;
 * does not post events to macOS or inspect any other application. */
static void spacebox_input_test(struct sdl2_console *scon)
{
    const char *path = getenv("SPACEBOX_INPUT_TEST");
    if (!path || !scon->real_window) return;
    sp_sched_deliver(scon);
    g_autofree char *text = NULL;
    if (!spacebox_hook_take(path, &text)) return;
    if (!strncmp(text, "schedule ", 9)) {
        sp_sched_start(text);
        return;
    }
    if (!strncmp(text, "schedule-stop", 13)) {
        if (sp_sched.ev) sp_sched_end("stopped");
        return;
    }
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
