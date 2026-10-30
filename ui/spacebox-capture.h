/* Optional measurement hook. Captures this client's GL back buffer only,
 * never the macOS desktop or other applications. Disabled without env paths.
 * On Windows the presenter (the client's way of showing frames: sink_cmd.py)
 * captures the guest's picture at the guest's own size (its frame as it is
 * shown, before it is scaled to the window: spacebox_capture_client_from), so
 * that a check compares it with the guest's pixels one to one; at a display
 * scale of 100% the window shows that picture at half its size (ui/sdl2.c).
 * On the Mac the window's back buffer is that size already (Retina).
 *
 * The request file is looked at every 100 ms, at a frame shown. Empty, it asks
 * for one picture of the next frame (SPACEBOX_CAPTURE_PPM). "series DIR N MS
 * SECS" asks for the next N frames shown, at least MS apart, for at most SECS
 * seconds, each as DIR/f-<wall clock in microseconds when it was read>.ppm, so
 * that a test sees when the picture changed (a frame is shown only when the
 * screen changes or the window needs it). */
#include "ui/spacebox-hook.h"

/* the picture of read framebuffer fb (0: the window's back buffer), width x
 * height from its origin, written top row first */
static void spacebox_capture_write(const char *path, GLuint fb, int width, int height)
{
    GLint read_fb, read_buffer, alignment, row_length, pack_buffer;
    GLenum before, after;
    unsigned char *pixels;
    FILE *f;

    before = glGetError();
    glGetIntegerv(GL_READ_FRAMEBUFFER_BINDING, &read_fb);
    glGetIntegerv(GL_READ_BUFFER, &read_buffer);
    glGetIntegerv(GL_PACK_ALIGNMENT, &alignment);
    glGetIntegerv(GL_PACK_ROW_LENGTH, &row_length);
    glGetIntegerv(GL_PIXEL_PACK_BUFFER_BINDING, &pack_buffer);
    glBindFramebuffer(GL_READ_FRAMEBUFFER, fb);
    glReadBuffer(fb ? GL_COLOR_ATTACHMENT0 : GL_BACK);
    glBindBuffer(GL_PIXEL_PACK_BUFFER, 0);
    glPixelStorei(GL_PACK_ALIGNMENT, 1);
    glPixelStorei(GL_PACK_ROW_LENGTH, 0);
    pixels = g_malloc((size_t)width * height * 3);
    glReadPixels(0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE, pixels);
    after = glGetError();
    glBindFramebuffer(GL_READ_FRAMEBUFFER, read_fb);
    glReadBuffer(read_buffer);
    glBindBuffer(GL_PIXEL_PACK_BUFFER, pack_buffer);
    glPixelStorei(GL_PACK_ALIGNMENT, alignment);
    glPixelStorei(GL_PACK_ROW_LENGTH, row_length);
    f = after == GL_NO_ERROR ? fopen(path, "wb") : NULL;
    if (f) {
        fprintf(f, "P6\n%d %d\n255\n", width, height);
        for (int y = height - 1; y >= 0; y--) {
            fwrite(pixels + (size_t)y * width * 3, 1, (size_t)width * 3, f);
        }
        fclose(f);
    }
    g_free(pixels);
    fprintf(stderr, "[SPACEBOX-CAPTURE] size=%dx%d gl_before=0x%x gl_read=0x%x saved=%d %s\n",
            width, height, before, after, !!f, path);
}

static void spacebox_capture_client_from(GLuint fb, int width, int height)
{
    static int64_t last_check;
    static struct {
        char *dir;
        int left, min_ms;
        int64_t until, last;
    } series;
    const char *request = getenv("SPACEBOX_CAPTURE_REQUEST");
    const char *output = getenv("SPACEBOX_CAPTURE_PPM");
    bool one = false;
    int64_t now;

    if (!request || !output) {
        return;
    }
    if (width <= 0 || height <= 0 || width > 8192 || height > 8192) {
        return;
    }
    now = g_get_monotonic_time();
    if (now - last_check >= 100000) {
        last_check = now;
        g_autofree char *text = NULL;

        if (!access(request, F_OK) && spacebox_hook_take(request, &text)) {
            char dir[1024];
            int n, ms, secs;

            if (text && sscanf(text, "series %1023s %d %d %d", dir, &n, &ms, &secs) == 4) {
                /* the frame shown now is the series' first */
                g_free(series.dir);
                series.dir = g_strdup(dir);
                series.left = n;
                series.min_ms = ms;
                series.until = now + (int64_t)secs * 1000000;
                series.last = 0;
                fprintf(stderr, "[SPACEBOX-CAPTURE] series %s: %d frames, %d ms apart, %d s\n",
                        dir, n, ms, secs);
            } else {
                one = true;
            }
        }
    }
    if (one) {
        spacebox_capture_write(output, fb, width, height);
    }
    if (series.dir) {
        if (series.left <= 0 || now > series.until) {
            fprintf(stderr, "[SPACEBOX-CAPTURE] series %s ended\n", series.dir);
            g_free(series.dir);
            series.dir = NULL;
        } else if (now - series.last >= (int64_t)series.min_ms * 1000) {
            g_autofree char *path = g_strdup_printf("%s/f-%" PRId64 ".ppm", series.dir,
                                                    g_get_real_time());

            series.last = now;
            series.left--;
            spacebox_capture_write(path, fb, width, height);
        }
    }
}

/* the window's back buffer, width x height */
static void spacebox_capture_client(int width, int height)
{
    spacebox_capture_client_from(0, width, height);
}
