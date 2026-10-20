/* The pictures of the user's toggle, in the renderer inside the guest
 * (hw/display/virtio-gpu-remote-picture.c encodes and sends them). While the
 * source asks for them, the scanout is read back after a frame is shown, at most
 * SPACEBOX_STREAM_RATE times a second by a token bucket of SPACEBOX_STREAM_BURST
 * tokens: a frame that finds no token is read when the next token comes, so the
 * last frame of a burst is sent. What is read is the scanout's texture (through
 * its framebuffer), not the window. */

bool spacebox_stream_on(void);
void spacebox_stream_take(uint8_t *px, int w, int h, int stride, bool bottom_up);

#define SPACEBOX_STREAM_RATE 30
#define SPACEBOX_STREAM_BURST 3

static struct {
    QEMUTimer *timer;
    struct sdl2_console *scon;
    double tokens;
    int64_t at;     /* when the tokens were counted, in microseconds */
} spacebox_stream = { .tokens = SPACEBOX_STREAM_BURST };

static void spacebox_stream_read(struct sdl2_console *scon)
{
    GLint read_fb, read_buffer, alignment, row_length, pack_buffer;
    int w = scon->w, h = scon->h;
    uint8_t *px;

    if (!scon->scanout_mode || !scon->guest_fb.framebuffer ||
        w <= 0 || h <= 0 || w > 8192 || h > 8192) {
        return;
    }
    SDL_GL_MakeCurrent(scon->real_window, scon->renderctx ? scon->renderctx : scon->winctx);
    glGetIntegerv(GL_READ_FRAMEBUFFER_BINDING, &read_fb);
    glGetIntegerv(GL_READ_BUFFER, &read_buffer);
    glGetIntegerv(GL_PACK_ALIGNMENT, &alignment);
    glGetIntegerv(GL_PACK_ROW_LENGTH, &row_length);
    glGetIntegerv(GL_PIXEL_PACK_BUFFER_BINDING, &pack_buffer);
    glBindFramebuffer(GL_READ_FRAMEBUFFER, scon->guest_fb.framebuffer);
    glReadBuffer(GL_COLOR_ATTACHMENT0);
    glBindBuffer(GL_PIXEL_PACK_BUFFER, 0);
    glPixelStorei(GL_PACK_ALIGNMENT, 4);
    glPixelStorei(GL_PACK_ROW_LENGTH, 0);
    px = g_malloc((size_t)w * h * 4);
    glReadPixels(scon->x, scon->y, w, h, GL_RGBA, GL_UNSIGNED_BYTE, px);
    glBindFramebuffer(GL_READ_FRAMEBUFFER, read_fb);
    glReadBuffer(read_buffer);
    glBindBuffer(GL_PIXEL_PACK_BUFFER, pack_buffer);
    glPixelStorei(GL_PACK_ALIGNMENT, alignment);
    glPixelStorei(GL_PACK_ROW_LENGTH, row_length);
    /* the texture's first row is the picture's top unless y0_top (egl_fb_blit
     * flips it into the window then) */
    spacebox_stream_take(px, w, h, w * 4, scon->y0_top);
}

/* Takes a token if one is there. */
static bool spacebox_stream_token(void)
{
    int64_t now = g_get_monotonic_time();

    spacebox_stream.tokens = MIN(SPACEBOX_STREAM_BURST,
                                 spacebox_stream.tokens +
                                 (now - spacebox_stream.at) * SPACEBOX_STREAM_RATE / 1e6);
    spacebox_stream.at = now;
    if (spacebox_stream.tokens < 1) {
        return false;
    }
    spacebox_stream.tokens -= 1;
    return true;
}

/* The next token has come: the frame shown last is read. */
static void spacebox_stream_tick(void *opaque)
{
    if (spacebox_stream_on() && spacebox_stream.scon && spacebox_stream_token()) {
        spacebox_stream_read(spacebox_stream.scon);
    }
}

/* A frame of the scanout was shown. */
static void spacebox_stream_frame(struct sdl2_console *scon)
{
    int64_t wait_us;

    if (!spacebox_stream_on()) {
        return;
    }
    spacebox_stream.scon = scon;
    if (!spacebox_stream.timer) {
        spacebox_stream.timer = timer_new_us(QEMU_CLOCK_REALTIME, spacebox_stream_tick, NULL);
        spacebox_stream.at = g_get_monotonic_time();
    }
    if (spacebox_stream_token()) {
        timer_del(spacebox_stream.timer);
        spacebox_stream_read(scon);
        return;
    }
    if (!timer_pending(spacebox_stream.timer)) {
        wait_us = (1 - spacebox_stream.tokens) * 1e6 / SPACEBOX_STREAM_RATE + 1;
        timer_mod(spacebox_stream.timer,
                  qemu_clock_get_us(QEMU_CLOCK_REALTIME) + wait_us);
    }
}
