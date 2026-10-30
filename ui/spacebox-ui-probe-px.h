/*
 * Spacebox: the UI evals' probe of the presenter, its part on pixels
 * (include/ui/spacebox-ui-probe.h; screen/ui/README.md of the spacebox
 * repository, "The strip" and "Region pixels and their checksum").
 *
 * The presenter reads a rectangle of a frame's texture back as RGBA pixels,
 * bottom row first (glReadPixels). The strip is several rows of the frame
 * code (ui/spacebox-frame-code.h), each one cell high, stacked from the top;
 * a region's checksum is the CRC-32 of zlib over its pixels as R, G, B, rows
 * from the top. Tested by tests/unit/test-spacebox-ui-probe.c.
 */

#ifndef UI_SPACEBOX_UI_PROBE_PX_H
#define UI_SPACEBOX_UI_PROBE_PX_H

#include <zlib.h>
#include "spacebox-frame-code.h"

/*
 * px: w x h RGBA pixels, bottom row first. The value of row r of the strip,
 * each row cell pixels high (in these pixels), read as the frame code is:
 * three lines across the central half of the row's height. -1 when the
 * markers do not hold or the row is not inside; *ok as the frame code's.
 */
static inline int spacebox_ui_probe_strip_row(const uint8_t *px, int w, int h,
                                              double cell, int r, bool *ok)
{
    const uint8_t *rows[3];

    *ok = false;
    for (int k = 0; k < 3; k++) {
        int y = (int)(cell * r + cell * (3 + k) / 8);   /* from the top */

        if (y < 0 || y >= h) {
            return -1;
        }
        rows[k] = px + (size_t)(h - 1 - y) * w * 4;
    }
    return spacebox_frame_code_read(rows, 3, w, ok);
}

/*
 * px: w x h RGBA pixels, bottom row first. out (w * h * 3 bytes): the
 * pixels as R, G, B, rows from the top. Returns their CRC-32.
 */
static inline uint32_t spacebox_ui_probe_rgb(const uint8_t *px, int w, int h,
                                             uint8_t *out)
{
    size_t n = (size_t)w * h * 3;

    for (int y = 0; y < h; y++) {
        const uint8_t *s = px + (size_t)(h - 1 - y) * w * 4;
        uint8_t *d = out + (size_t)y * w * 3;

        for (int x = 0; x < w; x++, s += 4, d += 3) {
            d[0] = s[0];
            d[1] = s[1];
            d[2] = s[2];
        }
    }
    return (uint32_t)crc32(crc32(0L, Z_NULL, 0), out, (uInt)n);
}

#endif
