/* Launcher command for this application's own visible window. */
static void spacebox_window_request(struct sdl2_console *scon)
{
    const char *path = getenv("SPACEBOX_WINDOW_REQUEST");
    if (!path || !scon->real_window || access(path, F_OK)) return;
    unlink(path);
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

/* Synthetic events in this SDL application's queue. Opt-in local validation;
 * does not post events to macOS or inspect any other application. */
static void spacebox_input_test(struct sdl2_console *scon)
{
    const char *path = getenv("SPACEBOX_INPUT_TEST");
    if (!path || !scon->real_window) return;
    FILE *f = fopen(path, "r");
    if (!f) return;
    char command[24];
    int a = 0, b = 0;
    int count = fscanf(f, "%23s %d %d", command, &a, &b);
    fclose(f);
    unlink(path);
    if (count < 1) return;
    static int test_x, test_y;
    SDL_Event ev = {0};
    Uint32 id = SDL_GetWindowID(scon->real_window);
    if (!strcmp(command, "key") && count == 3) {
        ev.type = b ? SDL_KEYDOWN : SDL_KEYUP;
        ev.key.windowID = id;
        ev.key.state = b ? SDL_PRESSED : SDL_RELEASED;
        ev.key.keysym.scancode = a;
        ev.key.keysym.sym = SDL_GetKeyFromScancode(a);
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
