/*
 * The UI evals' probe of the presenter, its part on pixels
 * (ui/spacebox-ui-probe-px.h): a strip of three rows of the frame code and a
 * region, drawn into a picture as glReadPixels gives it (RGBA, bottom row
 * first), sharp and scaled to half its size with a box filter (BF2's window
 * at a display scale of 100%); the region's checksum against zlib's CRC-32 of
 * the same pixels written the other way.
 */

#include "qemu/osdep.h"
#include "../../ui/spacebox-ui-probe-px.h"

#define W 320
#define CELL 16
#define ROWS 3
#define H (CELL * ROWS)

static int parity(int v)
{
    return __builtin_popcount(v) & 1;
}

/* the cell's colour: white (1) or black (0) */
static int cell_bit(int v, int c, bool good_parity)
{
    if (c == 0 || c == 19) {
        return 1;
    }
    if (c == 1) {
        return 0;
    }
    if (c == 18) {
        return parity(v) ^ !good_parity;
    }
    return v >> (17 - c) & 1;
}

/* the strip of values v[ROWS], W x H, RGBA, top row first */
static uint8_t *draw_strip(const int *v, bool good_parity)
{
    uint8_t *px = g_malloc((size_t)W * H * 4);

    for (int y = 0; y < H; y++) {
        for (int x = 0; x < W; x++) {
            uint8_t g = cell_bit(v[y / CELL], x / CELL, good_parity) ? 255 : 0;
            uint8_t *p = px + ((size_t)y * W + x) * 4;

            p[0] = g;
            p[1] = g;
            p[2] = g;
            p[3] = 255;
        }
    }
    return px;
}

/* top row first -> bottom row first (in place) */
static void flip(uint8_t *px, int w, int h)
{
    g_autofree uint8_t *row = g_malloc((size_t)w * 4);

    for (int y = 0; y < h / 2; y++) {
        uint8_t *a = px + (size_t)y * w * 4, *b = px + (size_t)(h - 1 - y) * w * 4;

        memcpy(row, a, (size_t)w * 4);
        memcpy(a, b, (size_t)w * 4);
        memcpy(b, row, (size_t)w * 4);
    }
}

/* half the size, each pixel the mean of 2 x 2 */
static uint8_t *half(const uint8_t *px, int w, int h)
{
    uint8_t *o = g_malloc((size_t)(w / 2) * (h / 2) * 4);

    for (int y = 0; y < h / 2; y++) {
        for (int x = 0; x < w / 2; x++) {
            for (int c = 0; c < 4; c++) {
                int s = px[((size_t)(2 * y) * w + 2 * x) * 4 + c] +
                        px[((size_t)(2 * y) * w + 2 * x + 1) * 4 + c] +
                        px[((size_t)(2 * y + 1) * w + 2 * x) * 4 + c] +
                        px[((size_t)(2 * y + 1) * w + 2 * x + 1) * 4 + c];

                o[((size_t)y * (w / 2) + x) * 4 + c] = (s + 2) / 4;
            }
        }
    }
    return o;
}

static void test_strip(void)
{
    static const int vals[][ROWS] = {
        { 0, 0, 0 }, { 1, 2, 3 }, { 65535, 0, 32768 }, { 12345, 678, 4321 },
    };

    for (size_t i = 0; i < G_N_ELEMENTS(vals); i++) {
        g_autofree uint8_t *px = draw_strip(vals[i], true);

        flip(px, W, H);
        for (int r = 0; r < ROWS; r++) {
            bool ok;
            int v = spacebox_ui_probe_strip_row(px, W, H, CELL, r, &ok);

            g_assert_cmpint(v, ==, vals[i][r]);
            g_assert_true(ok);
        }
        /* half the size: cells of 8 */
        g_autofree uint8_t *hp = half(px, W, H);
        for (int r = 0; r < ROWS; r++) {
            bool ok;
            int v = spacebox_ui_probe_strip_row(hp, W / 2, H / 2, CELL / 2.0, r, &ok);

            g_assert_cmpint(v, ==, vals[i][r]);
            g_assert_true(ok);
        }
    }
}

static void test_strip_bad(void)
{
    static const int v[ROWS] = { 77, 88, 99 };
    g_autofree uint8_t *px = draw_strip(v, false);
    bool ok;

    flip(px, W, H);
    /* parity wrong: the value is read, ok is false */
    g_assert_cmpint(spacebox_ui_probe_strip_row(px, W, H, CELL, 1, &ok), ==, 88);
    g_assert_false(ok);
    /* a row outside the picture */
    g_assert_cmpint(spacebox_ui_probe_strip_row(px, W, H, CELL, 3, &ok), ==, -1);
    g_assert_false(ok);
    /* all black: no markers */
    memset(px, 0, (size_t)W * H * 4);
    g_assert_cmpint(spacebox_ui_probe_strip_row(px, W, H, CELL, 0, &ok), ==, -1);
    g_assert_false(ok);
}

static void test_region(void)
{
    const int w = 37, h = 11;
    g_autofree uint8_t *px = g_malloc((size_t)w * h * 4);
    g_autofree uint8_t *out = g_malloc((size_t)w * h * 3);
    g_autofree uint8_t *want = g_malloc((size_t)w * h * 3);

    /* px bottom row first; want: top row first, RGB */
    for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++) {
            uint8_t *p = px + ((size_t)(h - 1 - y) * w + x) * 4;
            uint8_t *q = want + ((size_t)y * w + x) * 3;

            p[0] = q[0] = x * 7 + y;
            p[1] = q[1] = x ^ (y * 3);
            p[2] = q[2] = 255 - x - y;
            p[3] = 17;      /* alpha is dropped */
        }
    }
    uint32_t crc = spacebox_ui_probe_rgb(px, w, h, out);

    g_assert_cmpmem(out, (size_t)w * h * 3, want, (size_t)w * h * 3);
    g_assert_cmpuint(crc, ==, (uint32_t)crc32(0, want, (uInt)((size_t)w * h * 3)));
    /* one level in one pixel changes the checksum */
    px[((size_t)3 * w + 5) * 4 + 1] ^= 1;
    g_assert_cmpuint(spacebox_ui_probe_rgb(px, w, h, out), !=, crc);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    g_test_add_func("/spacebox-ui-probe/strip", test_strip);
    g_test_add_func("/spacebox-ui-probe/strip-bad", test_strip_bad);
    g_test_add_func("/spacebox-ui-probe/region", test_region);
    return g_test_run();
}
