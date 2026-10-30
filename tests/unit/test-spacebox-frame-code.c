/*
 * The frame code reader of the Mac's presenter (ui/spacebox-frame-code.h).
 *
 * Rows of the code drawn here (sharp, scaled with blur, with noise, shifted)
 * and, when SPACEBOX_FRAME_CODE_PPMS names a folder, the pictures in it:
 * binary PPM files whose names start with the frame number they carry
 * ("<number>.ppm" or "<number>-..."), each the whole video picture with the
 * code at its top (frames of a clip made with the code, decoded with ffmpeg).
 */

#include "qemu/osdep.h"
#include "../../ui/spacebox-frame-code.h"

#define WHITE 235
#define BLACK 16

static int parity(int code)
{
    return __builtin_popcount(code) & 1;
}

/* one row of the code: width pixels, RGBA */
static uint8_t *draw_row(int code, int width, bool good_parity)
{
    uint8_t *row = g_malloc0((size_t)width * 4);

    for (int x = 0; x < width; x++) {
        int c = x * SP_CODE_CELLS / width;
        int white;

        if (c == 0 || c == 19) {
            white = 1;
        } else if (c == 1) {
            white = 0;
        } else if (c == 18) {
            white = parity(code) ^ !good_parity;
        } else {
            white = code >> (17 - c) & 1;
        }
        memset(row + x * 4, white ? WHITE : BLACK, 3);
        row[x * 4 + 3] = 255;
    }
    return row;
}

/* a box blur of radius r (a picture scaled down and up) */
static void blur(uint8_t *row, int width, int r)
{
    uint8_t *copy = g_memdup2(row, (size_t)width * 4);

    for (int x = 0; x < width; x++) {
        int sum = 0, n = 0;

        for (int d = -r; d <= r; d++) {
            if (x + d >= 0 && x + d < width) {
                sum += copy[(x + d) * 4 + 1];
                n++;
            }
        }
        memset(row + x * 4, sum / n, 3);
    }
    g_free(copy);
}

static void noise(uint8_t *row, int width, int amplitude, uint32_t seed)
{
    for (int x = 0; x < width; x++) {
        int v;

        seed = seed * 1103515245 + 12345;
        v = row[x * 4 + 1] + (int)(seed >> 16) % (2 * amplitude + 1) - amplitude;
        memset(row + x * 4, MAX(0, MIN(255, v)), 3);
    }
}

static const int codes[] = { 0, 1, 2, 0x5555, 0xaaaa, 3599, 32768, 65535 };
static const int widths[] = { 20, 64, 640, 1280, 1919, 2560, 3840 };

static void check(const uint8_t *row, int width, int want, bool want_ok)
{
    const uint8_t *rows[3] = { row, row, row };
    bool ok;
    int code = spacebox_frame_code_read(rows, 3, width, &ok);

    g_assert_cmpint(code, ==, want);
    g_assert_cmpint(ok, ==, want_ok);
}

static void test_sharp(void)
{
    for (unsigned i = 0; i < G_N_ELEMENTS(codes); i++) {
        for (unsigned j = 0; j < G_N_ELEMENTS(widths); j++) {
            g_autofree uint8_t *row = draw_row(codes[i], widths[j], true);

            check(row, widths[j], codes[i], true);
        }
    }
}

static void test_blurred_noisy(void)
{
    for (unsigned i = 0; i < G_N_ELEMENTS(codes); i++) {
        for (unsigned j = 2; j < G_N_ELEMENTS(widths); j++) {
            g_autofree uint8_t *row = draw_row(codes[i], widths[j], true);

            /* a blur of a sixth of a cell, noise of +-60 */
            blur(row, widths[j], widths[j] / SP_CODE_CELLS / 6);
            noise(row, widths[j], 60, codes[i] + widths[j]);
            check(row, widths[j], codes[i], true);
        }
    }
}

static void test_shifted(void)
{
    /* the rectangle a few pixels off the video's edge (rounding of the
     * element's place): an eighth of a cell either way still reads */
    for (unsigned i = 0; i < G_N_ELEMENTS(codes); i++) {
        int width = 1920, shift = width / SP_CODE_CELLS / 8;
        g_autofree uint8_t *row = draw_row(codes[i], width + 2 * shift, true);
        const uint8_t *rows[1];
        bool ok;

        rows[0] = row;
        g_assert_cmpint(spacebox_frame_code_read(rows, 1, width, &ok), ==, codes[i]);
        g_assert_true(ok);
        rows[0] = row + 2 * shift * 4;
        g_assert_cmpint(spacebox_frame_code_read(rows, 1, width, &ok), ==, codes[i]);
        g_assert_true(ok);
    }
}

static void test_parity(void)
{
    g_autofree uint8_t *row = draw_row(0x1234, 1280, false);

    check(row, 1280, 0x1234, false);
}

static void test_no_code(void)
{
    g_autofree uint8_t *row = g_malloc((size_t)1280 * 4);

    memset(row, 128, (size_t)1280 * 4);
    check(row, 1280, -1, false);        /* grey: no markers */
    memset(row, 255, (size_t)1280 * 4);
    check(row, 1280, -1, false);        /* white: cell 1 is not black */
    memset(row, 0, (size_t)1280 * 4);
    check(row, 1280, -1, false);
    check(row, 19, -1, false);          /* narrower than the cells */
}

/* the pictures of SPACEBOX_FRAME_CODE_PPMS, read as the presenter reads them */
static void test_pictures(void)
{
    const char *dir = g_getenv("SPACEBOX_FRAME_CODE_PPMS");
    g_autoptr(GDir) d = NULL;
    const char *name;
    unsigned n = 0;

    if (!dir) {
        g_test_skip("SPACEBOX_FRAME_CODE_PPMS is not set");
        return;
    }
    d = g_dir_open(dir, 0, NULL);
    g_assert_nonnull(d);
    while ((name = g_dir_read_name(d))) {
        g_autofree char *path = g_build_filename(dir, name, NULL);
        g_autofree char *data = NULL;
        gsize len;
        int w, h, max, off, want = atoi(name), code;
        const uint8_t *rows[3];
        uint8_t *rgba[3];
        bool ok;

        if (!g_str_has_suffix(name, ".ppm")) {
            continue;
        }
        g_assert_true(g_file_get_contents(path, &data, &len, NULL));
        g_assert_cmpint(sscanf(data, "P6 %d %d %d%n", &w, &h, &max, &off), ==, 3);
        off++;
        for (int k = 0; k < 3; k++) {
            int y = (int)(w / (double)SP_CODE_CELLS * (3 + k) / 8);

            rgba[k] = g_malloc((size_t)w * 4);
            for (int x = 0; x < w; x++) {
                memcpy(rgba[k] + x * 4, data + off + ((size_t)y * w + x) * 3, 3);
                rgba[k][x * 4 + 3] = 255;
            }
            rows[k] = rgba[k];
        }
        code = spacebox_frame_code_read(rows, 3, w, &ok);
        if (code != want || !ok) {
            g_test_message("%s: read %d ok %d", name, code, ok);
        }
        g_assert_cmpint(code, ==, want);
        g_assert_true(ok);
        for (int k = 0; k < 3; k++) {
            g_free(rgba[k]);
        }
        n++;
    }
    g_test_message("%u pictures read", n);
    g_assert_cmpuint(n, >, 0);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    g_test_add_func("/spacebox/frame-code/sharp", test_sharp);
    g_test_add_func("/spacebox/frame-code/blurred-noisy", test_blurred_noisy);
    g_test_add_func("/spacebox/frame-code/shifted", test_shifted);
    g_test_add_func("/spacebox/frame-code/parity", test_parity);
    g_test_add_func("/spacebox/frame-code/no-code", test_no_code);
    g_test_add_func("/spacebox/frame-code/pictures", test_pictures);
    return g_test_run();
}
