/*
 * Spacebox: the frame code (version 1) of the test clips and streams, as the
 * presenter reads it in the picture it swaps (ui/spacebox-presenter.h).
 *
 * One row of 20 square cells across the top edge of the video's picture,
 * each cell W/20 of the picture's displayed width W: cells 0 and 19 white,
 * cell 1 black (the markers), cells 2 to 17 the 16 bits of the frame's number
 * (cell 2 the most significant, white = 1), cell 18 white when the number of
 * 1 bits is odd (the parity). A cell is read as the mean green of its central
 * quarter: white above 128.
 */

#ifndef UI_SPACEBOX_FRAME_CODE_H
#define UI_SPACEBOX_FRAME_CODE_H

#define SP_CODE_CELLS 20

/*
 * rows: nrows rows of width RGBA pixels (4 bytes each), the cells' row of the
 * picture from the left edge of the video, taken in the central half of the
 * cells' height. Returns the frame's number, or -1 when the markers do not
 * hold; *ok is true when the markers and the parity hold.
 */
static inline int spacebox_frame_code_read(const uint8_t *const *rows,
                                           int nrows, int width, bool *ok)
{
    int bits[SP_CODE_CELLS], code = 0, ones = 0;

    *ok = false;
    if (nrows < 1 || width < SP_CODE_CELLS) {
        return -1;
    }
    for (int c = 0; c < SP_CODE_CELLS; c++) {
        /* the central half of the cell's width */
        int x0 = (int)((c + 0.25) * width / SP_CODE_CELLS);
        int x1 = (int)((c + 0.75) * width / SP_CODE_CELLS);
        unsigned long sum = 0, n = 0;

        if (x1 <= x0) {
            x1 = x0 + 1;
        }
        for (int r = 0; r < nrows; r++) {
            for (int x = x0; x < x1; x++) {
                sum += rows[r][x * 4 + 1];
                n++;
            }
        }
        bits[c] = sum > 128 * n;
    }
    if (!bits[0] || bits[1] || !bits[19]) {
        return -1;
    }
    for (int c = 2; c < 18; c++) {
        code = code << 1 | bits[c];
        ones += bits[c];
    }
    *ok = bits[18] == (ones & 1);
    return code;
}

#endif
