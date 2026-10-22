/*
 * Spacebox remote GPU, sink: the record of the renderer's state, so that a
 * source can build it again in another renderer (the screen track's K4).
 *
 * The sink keeps, for each resource, its create arguments and two message ids
 * of the source's stream: the RESOURCE_CREATE that made it and the last
 * SP_WMARK that listed it. The source lists a resource in an SP_WMARK once
 * after each mark point of the connection (a new pairing, a state reset, a
 * snapshot of the source), before the first command after that point that may
 * write it, and not again until the next point; so the sink's `written` of a
 * resource is the first mark after the newest point, and a resource written
 * after N has written > N, which is all the keep test (created <= N &&
 * written <= N) needs. With them it answers:
 *
 *   SP_STATE_RESET N   destroy every context, the scanouts, the cursor, the
 *                      pending fences, and every resource but those created
 *                      and last written at or before N; the kept ones keep
 *                      their contents and their shadow of the guest backing
 *   SP_IMPORT          write bytes into a resource
 *   SP_EXPORT          read resources out, as SP_IMPORTs for another sink
 *
 * in order with the commands before them: they go through the command queue
 * (cmd->sp_op), so none overtakes a command still waiting there.
 *
 * SPACEBOX_GPU_REMOTE_CONTROL=<file>        sink, tests: read one line from the
 *                                           file every 100 ms (the file is then
 *                                           deleted): "detach" (send SP_DETACH),
 *                                           "clear" (set every render target and
 *                                           depth buffer to zero, then show the
 *                                           scanout again), "stats" (log the
 *                                           resources), "stall-export MS" (the
 *                                           next SP_EXPORT waits MS ms, with
 *                                           the sink's main loop held, before it
 *                                           runs: a renderer that stalls), "gpu
 *                                           off", "gpu on" (the user's toggle, as
 *                                           its hotkey Ctrl+Alt+A).
 */

#include "qemu/osdep.h"
#include "qemu/iov.h"
#include "qemu/error-report.h"
#include "qemu/main-loop.h"
#include "qemu/timer.h"
#include "hw/virtio/virtio.h"
#include "hw/virtio/virtio-gpu.h"
#include "ui/console.h"
#include "virtio-gpu-remote.h"
#include "virtio-gpu-remote-formats.h"

#include <math.h>
#include <virglrenderer.h>
#include <epoxy/gl.h>

/* gallium's pipe_texture_target, as the guest sends it */
enum {
    SP_T_BUFFER, SP_T_1D, SP_T_2D, SP_T_3D, SP_T_CUBE, SP_T_RECT,
    SP_T_1D_ARRAY, SP_T_2D_ARRAY, SP_T_CUBE_ARRAY,
};

#define SP_BIND_DEPTH_STENCIL (1u << 0)
#define SP_BIND_RENDER_TARGET (1u << 1)

typedef struct SpRes {
    uint32_t id, target, format, bind, width, height, depth, array_size;
    uint32_t last_level, nr_samples, flags;
    uint64_t created;   /* id of the message that created it */
    uint64_t written;   /* id of the last SP_WMARK that listed it (the first
                         * after the newest mark point), 0 = none */
} SpRes;

typedef struct SpOp {
    uint32_t type;
    uint64_t id, arg0;
    GByteArray *a, *b;
} SpOp;

static struct {
    VirtIOGPU *g;
    GHashTable *res;            /* resource id -> SpRes */
    GHashTable *ctx;            /* ids of the contexts that exist */
    GByteArray *next_wmark;     /* the last SP_WMARK: u64 id, then u32[] */
    char *control;
    QEMUTimer *control_timer;
    struct {
        uint64_t n, bytes, failed;
        int64_t t0, last;
    } imp;                      /* the imports since the last log line */
    bool ctx0_dirty;            /* kind-0 imports not yet finished in context 0 */
    unsigned ctx0_stale;        /* GL errors found waiting in context 0 */
    unsigned export_stall_ms;   /* the control's "stall-export MS", for the next export */
} sps;

/*
 * virglrenderer's context 0 before one of its transfers: GL errors that
 * earlier calls left there are cleared. vrend_transfer_send_readpixels() takes
 * any pending error as its own failure and then reads with glGetTexImage,
 * which does not turn a Y_0_TOP resource the guest's way up; and the creation
 * of a multisampled texture leaves one (vrend_resource_alloc_texture() also
 * calls glTexStorage2D on GL_TEXTURE_2D_MULTISAMPLE, GL_INVALID_ENUM).
 */
static void sp_ctx0_clean(void)
{
    unsigned n = 0;

    virgl_renderer_force_ctx_0();
    while (glGetError() != GL_NO_ERROR && n < 64) {
        n++;
    }
    sps.ctx0_stale += n;
}

/*
 * What ran in other GL contexts complete before the sink reads: each guest
 * context made current by an empty command stream (virglrenderer then
 * switches to it) and finished, then context 0 (its kind-0 imports). Without
 * it a read in context 0 or in the sink's own context can see a texture
 * before the writes of another context reach it (llvmpipe runs each
 * context's rendering on threads of its own).
 */
static void sp_state_settle(void)
{
    static const uint32_t nothing;
    GHashTableIter it;
    gpointer key;

    g_hash_table_iter_init(&it, sps.ctx);
    while (g_hash_table_iter_next(&it, &key, NULL)) {
        if (!virgl_renderer_submit_cmd((void *)&nothing, GPOINTER_TO_UINT(key), 0)) {
            glFinish();
        }
    }
    virgl_renderer_force_ctx_0();
    glFinish();
    sps.ctx0_dirty = false;
}

/* ---- the record ---------------------------------------------------------- */

static bool sp_is_depth(uint32_t f)
{
    return f == 16 || f == 17 || f == 18 || f == 21 ||           /* Z only */
           f == 19 || f == 20 || f == 126 || f == 328 ||         /* Z and S */
           f == 23 || f == 136 || f == 137 || f == 138;          /* S only */
}

static bool sp_has_stencil(uint32_t f)
{
    return sp_is_depth(f) && !(f == 16 || f == 17 || f == 18 || f == 21);
}

static bool sp_has_depth(uint32_t f)
{
    return sp_is_depth(f) && !(f == 23 || f == 136 || f == 137 || f == 138);
}

static void sp_level_dims(const SpRes *r, uint32_t level, uint32_t *w,
                          uint32_t *h, uint32_t *layers)
{
    *w = MAX(level < 32 ? r->width >> level : 0, 1);
    *h = r->target == SP_T_1D || r->target == SP_T_1D_ARRAY ? 1 :
         MAX(level < 32 ? r->height >> level : 0, 1);
    switch (r->target) {
    case SP_T_3D:
        *layers = MAX(level < 32 ? r->depth >> level : 0, 1);
        break;
    case SP_T_CUBE:
        *layers = 6;
        break;
    case SP_T_1D_ARRAY:
    case SP_T_2D_ARRAY:
    case SP_T_CUBE_ARRAY:
        *layers = MAX(r->array_size, 1);
        break;
    default:
        *layers = 1;
    }
}

/* Bytes of one level and layer in the guest layout, 0 = unknown format. */
static uint64_t sp_layer_size(const SpRes *r, uint32_t level, uint64_t *row,
                              uint32_t *rows)
{
    uint32_t w, h, layers, bw, bh, bits;

    if (r->format >= SP_FORMAT_COUNT || sp_format_blocks[r->format].bits < 8) {
        return 0;
    }
    bw = sp_format_blocks[r->format].bw;
    bh = sp_format_blocks[r->format].bh;
    bits = sp_format_blocks[r->format].bits;
    sp_level_dims(r, level, &w, &h, &layers);
    *row = (uint64_t)((w + bw - 1) / bw) * (bits / 8);
    *rows = (h + bh - 1) / bh;
    return *row * *rows;
}

static uint64_t sp_res_size(const SpRes *r)
{
    uint64_t sum = 0, row;
    uint32_t l, rows, w, h, layers;

    if (r->target == SP_T_BUFFER) {
        return r->width;
    }
    for (l = 0; l <= r->last_level && l < 32; l++) {
        sp_level_dims(r, l, &w, &h, &layers);
        sum += sp_layer_size(r, l, &row, &rows) * layers;
    }
    return sum * MAX(r->nr_samples, 1);
}

void sp_state_forget(void)
{
    if (sps.res) {
        g_hash_table_remove_all(sps.res);
        g_hash_table_remove_all(sps.ctx);
    }
    if (sps.next_wmark) {
        g_byte_array_unref(sps.next_wmark);
        sps.next_wmark = NULL;
    }
}

void sp_state_wmark(uint64_t id, const void *a, size_t len)
{
    if (sps.next_wmark) {
        g_byte_array_unref(sps.next_wmark);
    }
    sps.next_wmark = g_byte_array_sized_new(sizeof(id) + len);
    g_byte_array_append(sps.next_wmark, (const guint8 *)&id, sizeof(id));
    g_byte_array_append(sps.next_wmark, a, len);
}

void sp_state_attach(struct virtio_gpu_ctrl_command *cmd, uint64_t id)
{
    cmd->sp_id = id;
    cmd->sp_wmark = sps.next_wmark;
    sps.next_wmark = NULL;
}

/* Before a command of the source runs: what it creates, destroys or writes. */
static void sp_state_note(VirtIOGPU *g, struct virtio_gpu_ctrl_command *cmd)
{
    struct virtio_gpu_ctrl_hdr hdr;
    struct virtio_gpu_resource_create_3d c3;
    struct virtio_gpu_resource_create_2d c2;
    struct virtio_gpu_resource_unref un;
    size_t n = iov_size(cmd->elem.out_sg, cmd->elem.out_num);
    SpRes *r;

    if (iov_to_buf(cmd->elem.out_sg, cmd->elem.out_num, 0, &hdr,
                   sizeof(hdr)) != sizeof(hdr)) {
        return;
    }
    switch (hdr.type) {
    case VIRTIO_GPU_CMD_CTX_CREATE:
        g_hash_table_add(sps.ctx, GUINT_TO_POINTER(hdr.ctx_id));
        break;
    case VIRTIO_GPU_CMD_CTX_DESTROY:
        g_hash_table_remove(sps.ctx, GUINT_TO_POINTER(hdr.ctx_id));
        break;
    case VIRTIO_GPU_CMD_RESOURCE_CREATE_2D:
        if (n < sizeof(c2)) {
            break;
        }
        iov_to_buf(cmd->elem.out_sg, cmd->elem.out_num, 0, &c2, sizeof(c2));
        if (!c2.resource_id || virtio_gpu_find_resource(g, c2.resource_id)) {
            break;
        }
        /* as virgl_cmd_create_resource_2d() makes it */
        r = g_new0(SpRes, 1);
        *r = (SpRes){ .id = c2.resource_id, .target = SP_T_2D,
                      .format = c2.format, .bind = SP_BIND_RENDER_TARGET,
                      .width = c2.width, .height = c2.height, .depth = 1,
                      .array_size = 1,
                      .flags = VIRTIO_GPU_RESOURCE_FLAG_Y_0_TOP,
                      .created = cmd->sp_id };
        g_hash_table_replace(sps.res, GUINT_TO_POINTER(r->id), r);
        break;
    case VIRTIO_GPU_CMD_RESOURCE_CREATE_3D:
        if (n < sizeof(c3)) {
            break;
        }
        iov_to_buf(cmd->elem.out_sg, cmd->elem.out_num, 0, &c3, sizeof(c3));
        if (!c3.resource_id || virtio_gpu_find_resource(g, c3.resource_id)) {
            break;
        }
        r = g_new0(SpRes, 1);
        *r = (SpRes){ .id = c3.resource_id, .target = c3.target,
                      .format = c3.format, .bind = c3.bind,
                      .width = c3.width, .height = c3.height,
                      .depth = c3.depth, .array_size = c3.array_size,
                      .last_level = c3.last_level,
                      .nr_samples = c3.nr_samples, .flags = c3.flags,
                      .created = cmd->sp_id };
        g_hash_table_replace(sps.res, GUINT_TO_POINTER(r->id), r);
        break;
    case VIRTIO_GPU_CMD_RESOURCE_UNREF:
        if (n >= sizeof(un)) {
            iov_to_buf(cmd->elem.out_sg, cmd->elem.out_num, 0, &un, sizeof(un));
            if (virtio_gpu_find_resource(g, un.resource_id)) {
                g_hash_table_remove(sps.res, GUINT_TO_POINTER(un.resource_id));
            }
        }
        break;
    default:
        break;
    }
}

static void sp_state_written(GByteArray *wm)
{
    uint64_t id;
    size_t i;

    if (!wm || wm->len < sizeof(id)) {
        return;
    }
    memcpy(&id, wm->data, sizeof(id));
    for (i = sizeof(id); i + 4 <= wm->len; i += 4) {
        uint32_t res;
        SpRes *r;

        memcpy(&res, wm->data + i, 4);
        r = g_hash_table_lookup(sps.res, GUINT_TO_POINTER(res));
        if (r) {
            r->written = id;
        }
    }
}

/* ---- a GL context of the sink's own ---------------------------------------- */

/*
 * For what virglrenderer's own calls cannot do (contents beyond the guest
 * layout: depth bits exactly, stencil, each sample of a multisampled target,
 * and the test command "clear"): a context of the display, which shares its
 * objects with virglrenderer's contexts as theirs do with each other
 * (dpy_gl_ctx_create() shares with the display's context, see
 * virgl_create_context()). Afterwards virglrenderer is told that its context is
 * no longer current (virgl_renderer_force_ctx_0()).
 */
static struct {
    QEMUGLContext ctx;
    bool failed;
    GLuint fbo, vao;
} spgl;

static bool sp_gl_begin(VirtIOGPU *g)
{
    QemuConsole *con = g->parent_obj.scanout[0].con;

    if (spgl.failed || !con) {
        return false;
    }
    if (!spgl.ctx) {
        QEMUGLParams p = { .major_ver = 3, .minor_ver = 3 };

        virgl_renderer_force_ctx_0();
        spgl.ctx = dpy_gl_ctx_create(con, &p);
        if (!spgl.ctx) {
            spgl.failed = true;
            virgl_renderer_force_ctx_0();
            error_report("spacebox remote gpu: no GL context of the sink's own");
            return false;
        }
        dpy_gl_ctx_make_current(con, spgl.ctx);
        glGenFramebuffers(1, &spgl.fbo);
        glGenVertexArrays(1, &spgl.vao);
        fprintf(stderr, "[SPACEBOX-REMOTE] sink: GL context of its own: %s, %s\n",
                (const char *)glGetString(GL_VERSION),
                (const char *)glGetString(GL_RENDERER));
    }
    dpy_gl_ctx_make_current(con, spgl.ctx);
    while (glGetError() != GL_NO_ERROR) {
        /* errors of earlier work are not ours to report */
    }
    glBindFramebuffer(GL_FRAMEBUFFER, spgl.fbo);
    return true;
}

static void sp_gl_end(void)
{
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glFinish(); /* virglrenderer's contexts see what was written */
    virgl_renderer_force_ctx_0();
}

static GLenum sp_gl_attachment(uint32_t format)
{
    if (!sp_is_depth(format)) {
        return GL_COLOR_ATTACHMENT0;
    }
    if (!sp_has_stencil(format)) {
        return GL_DEPTH_ATTACHMENT;
    }
    return sp_has_depth(format) ? GL_DEPTH_STENCIL_ATTACHMENT : GL_STENCIL_ATTACHMENT;
}

/* The parts of one resource, each SpXfer then bytes, sent only when complete. */
typedef GPtrArray SpParts;

static GByteArray *sp_part_new(const SpXfer *x, size_t len, uint8_t **data)
{
    GByteArray *p = g_byte_array_sized_new(sizeof(*x) + len);

    g_byte_array_append(p, (const guint8 *)x, sizeof(*x));
    g_byte_array_set_size(p, sizeof(*x) + len);
    *data = p->data + sizeof(*x);
    return p;
}

/* ---- contents beyond the guest layout: kinds 1 to 4 ------------------------- */

/*
 * The kinds of SpXfer this sink makes besides 0. Their rows are in GL's order
 * (box y counts from GL's row 0), packed, and the same on every desktop GL
 * renderer of virglrenderer (the same format table):
 *
 *   1  a level and layer of a single-sample texture with depth, as
 *      glReadPixels() gives it: depth, or depth and stencil packed
 *      (UNSIGNED_INT_24_8, FLOAT_32_UNSIGNED_INT_24_8_REV; Z24X8 as
 *      UNSIGNED_INT with the low byte 0). Written back by drawing: the depth
 *      through gl_FragDepth and the stencil as kind 4 does, since Apple's GL
 *      keeps Z24 as a float and a glTexSubImage of it comes back off by one;
 *      with glTexSubImage only where the depth self-check finds gl_FragDepth
 *      not exact. Kind 0 is not used for these: virglrenderer rescales Z24X8
 *      through a float on both ways (vrend_scale_depth()) and loses the low 8
 *      bits of the depth.
 *   2  sample `sample` of layer box z of a multisampled color texture (2D or
 *      2D array): drawn by texelFetch() into a single-sample texture of the
 *      same internal format and read from there; written back by drawing from
 *      such a texture with the sample mask of that sample (glTexSubImage into
 *      a multisampled texture does not exist). Snorm formats are copied
 *      instead, since texelFetch() gives -1.0 for both -128 and -127: the
 *      layer is copied by glCopyImageSubData (bits, not values) into a
 *      multisampled texture of the signed integer format of the same size,
 *      read from there as integers and written back the same way round; the
 *      bytes are those of the snorm format's own type (GL_BYTE, GL_SHORT). A
 *      renderer without glCopyImageSubData (Apple's GL 4.1) goes through
 *      floats instead (the user's choice of 2026-10-17): texelFetch() into an
 *      RGBA32F texture, rounded to the format's integers, and back from such
 *      a texture; every value is kept but the lowest, which arrives as the
 *      lowest + 1 (-128 as -127, -32768 as -32767): the guest under the
 *      profile cannot make it by drawing or clearing
 *   3  sample `sample` of layer box z of a multisampled texture with depth:
 *      u32 per texel, the depth as an integer of the format's bits (Z16, Z24,
 *      Z32) or the bits of the float (Z32F); read as the float that
 *      texelFetch() gives, written back through gl_FragDepth, both converted
 *      here in double precision. The float conversions of a renderer need not
 *      be exact, so a self-check (sp_depth_selfcheck()) looks at them first.
 *      Where gl_FragDepth is exact but texelFetch() is not (llvmpipe's Z24
 *      read is off by one for about a third of the values), the depth is read
 *      by the depth test instead, which compares in the format's own integers:
 *      a pass for each bit, from the highest, asks whether the stored value is
 *      at least the bits found so far with this bit set. A renderer whose
 *      gl_FragDepth is not exact keeps no multisampled depth of that format
 *      (the resource is not exported), except Z32_UNORM: a float has 24
 *      bits, so no renderer keeps multisampled Z32 exactly, and it is kept
 *      with rounding (the user's choice of 2026-10-17): read by texelFetch(),
 *      written by gl_FragDepth, through the 24 bits of the float; the low bits
 *      of a value can change. Single-sample Z32_UNORM that a renderer does
 *      not hold exactly (Apple's GL keeps it as a float) is kept the same way,
 *      with the rounding of that renderer's glTexSubImage and glReadPixels
 *   4  the stencil, u8 per texel, of sample `sample` of layer box z of a
 *      multisampled texture, or of level `level` and layer box z of a
 *      single-sample stencil-only texture. Read without stencil texturing (the
 *      Mac's GL 4.1 has none): single-sample by glReadPixels(), multisampled
 *      by a pass for each bit with the stencil test EQUAL on that bit that
 *      adds the bit to an R8 target of the same samples; written back by a
 *      pass that sets the sample's stencil to 0 and a pass for each bit that
 *      sets the bit where it is 1 (stencil op REPLACE, write mask on that bit)
 */
enum { SP_K_GUEST, SP_K_DEPTH, SP_K_MS_COLOR, SP_K_MS_DEPTH, SP_K_STENCIL };

typedef struct SpFmt {
    GLenum ifmt, fmt, type;
    uint8_t bpp;
    char cls;           /* 'f' float or normalized, 'i' int, 'u' unsigned,
                         * 's' signed normalized (copied as `twin`) */
    GLenum twin;        /* 's': the signed integer format of the same size */
} SpFmt;

static const SpFmt sp_fmts[] = {
    { GL_RGBA8, GL_RGBA, GL_UNSIGNED_BYTE, 4, 'f' },
    { GL_SRGB8_ALPHA8, GL_RGBA, GL_UNSIGNED_BYTE, 4, 'f' },
    { GL_RGB8, GL_RGB, GL_UNSIGNED_BYTE, 3, 'f' },
    { GL_SRGB8, GL_RGB, GL_UNSIGNED_BYTE, 3, 'f' },
    { GL_RG8, GL_RG, GL_UNSIGNED_BYTE, 2, 'f' },
    { GL_R8, GL_RED, GL_UNSIGNED_BYTE, 1, 'f' },
    { GL_RGBA16, GL_RGBA, GL_UNSIGNED_SHORT, 8, 'f' },
    { GL_RG16, GL_RG, GL_UNSIGNED_SHORT, 4, 'f' },
    { GL_R16, GL_RED, GL_UNSIGNED_SHORT, 2, 'f' },
    { GL_RGB565, GL_RGB, GL_UNSIGNED_SHORT_5_6_5, 2, 'f' },
    { GL_RGB5_A1, GL_RGBA, GL_UNSIGNED_SHORT_5_5_5_1, 2, 'f' },
    { GL_RGBA4, GL_RGBA, GL_UNSIGNED_SHORT_4_4_4_4, 2, 'f' },
    { GL_R3_G3_B2, GL_RGB, GL_UNSIGNED_BYTE_3_3_2, 1, 'f' },
    { GL_RGB10_A2, GL_RGBA, GL_UNSIGNED_INT_2_10_10_10_REV, 4, 'f' },
    { GL_R11F_G11F_B10F, GL_RGB, GL_UNSIGNED_INT_10F_11F_11F_REV, 4, 'f' },
    { GL_RGBA16F, GL_RGBA, GL_HALF_FLOAT, 8, 'f' },
    { GL_RGB16F, GL_RGB, GL_HALF_FLOAT, 6, 'f' },
    { GL_RG16F, GL_RG, GL_HALF_FLOAT, 4, 'f' },
    { GL_R16F, GL_RED, GL_HALF_FLOAT, 2, 'f' },
    { GL_RGBA32F, GL_RGBA, GL_FLOAT, 16, 'f' },
    { GL_RGB32F, GL_RGB, GL_FLOAT, 12, 'f' },
    { GL_RG32F, GL_RG, GL_FLOAT, 8, 'f' },
    { GL_R32F, GL_RED, GL_FLOAT, 4, 'f' },
    { GL_RGB10_A2UI, GL_RGBA_INTEGER, GL_UNSIGNED_INT_2_10_10_10_REV, 4, 'u' },
    { GL_RGBA8UI, GL_RGBA_INTEGER, GL_UNSIGNED_BYTE, 4, 'u' },
    { GL_RG8UI, GL_RG_INTEGER, GL_UNSIGNED_BYTE, 2, 'u' },
    { GL_R8UI, GL_RED_INTEGER, GL_UNSIGNED_BYTE, 1, 'u' },
    { GL_RGBA16UI, GL_RGBA_INTEGER, GL_UNSIGNED_SHORT, 8, 'u' },
    { GL_RG16UI, GL_RG_INTEGER, GL_UNSIGNED_SHORT, 4, 'u' },
    { GL_R16UI, GL_RED_INTEGER, GL_UNSIGNED_SHORT, 2, 'u' },
    { GL_RGBA32UI, GL_RGBA_INTEGER, GL_UNSIGNED_INT, 16, 'u' },
    { GL_RG32UI, GL_RG_INTEGER, GL_UNSIGNED_INT, 8, 'u' },
    { GL_R32UI, GL_RED_INTEGER, GL_UNSIGNED_INT, 4, 'u' },
    { GL_RGBA8I, GL_RGBA_INTEGER, GL_BYTE, 4, 'i' },
    { GL_RG8I, GL_RG_INTEGER, GL_BYTE, 2, 'i' },
    { GL_R8I, GL_RED_INTEGER, GL_BYTE, 1, 'i' },
    { GL_RGBA16I, GL_RGBA_INTEGER, GL_SHORT, 8, 'i' },
    { GL_RG16I, GL_RG_INTEGER, GL_SHORT, 4, 'i' },
    { GL_R16I, GL_RED_INTEGER, GL_SHORT, 2, 'i' },
    { GL_RGBA32I, GL_RGBA_INTEGER, GL_INT, 16, 'i' },
    { GL_RG32I, GL_RG_INTEGER, GL_INT, 8, 'i' },
    { GL_R32I, GL_RED_INTEGER, GL_INT, 4, 'i' },
    { GL_RGBA8_SNORM, GL_RGBA, GL_BYTE, 4, 's', GL_RGBA8I },
    { GL_RG8_SNORM, GL_RG, GL_BYTE, 2, 's', GL_RG8I },
    { GL_R8_SNORM, GL_RED, GL_BYTE, 1, 's', GL_R8I },
    { GL_RGBA16_SNORM, GL_RGBA, GL_SHORT, 8, 's', GL_RGBA16I },
    { GL_RG16_SNORM, GL_RG, GL_SHORT, 4, 's', GL_RG16I },
    { GL_R16_SNORM, GL_RED, GL_SHORT, 2, 's', GL_R16I },
};

static const SpFmt sp_fmt_r32ui = { GL_R32UI, GL_RED_INTEGER, GL_UNSIGNED_INT, 4, 'u' };
static const SpFmt sp_fmt_r8ui = { GL_R8UI, GL_RED_INTEGER, GL_UNSIGNED_BYTE, 1, 'u' };

static const SpFmt *sp_fmt_of(GLint ifmt)
{
    unsigned i;

    for (i = 0; i < G_N_ELEMENTS(sp_fmts); i++) {
        if (sp_fmts[i].ifmt == (GLenum)ifmt) {
            return &sp_fmts[i];
        }
    }
    return NULL;
}

/* kind 1: how a single-sample texture with depth is read and written */
static bool sp_depth_fmt(uint32_t f, GLenum *fmt, GLenum *type, unsigned *bpp)
{
    switch (f) {
    case 16:    /* Z16_UNORM */
        *fmt = GL_DEPTH_COMPONENT, *type = GL_UNSIGNED_SHORT, *bpp = 2;
        return true;
    case 17:    /* Z32_UNORM */
    case 21:    /* Z24X8_UNORM */
        *fmt = GL_DEPTH_COMPONENT, *type = GL_UNSIGNED_INT, *bpp = 4;
        return true;
    case 18:    /* Z32_FLOAT */
        *fmt = GL_DEPTH_COMPONENT, *type = GL_FLOAT, *bpp = 4;
        return true;
    case 19:    /* Z24_UNORM_S8_UINT */
    case 20:    /* S8_UINT_Z24_UNORM */
        *fmt = GL_DEPTH_STENCIL, *type = GL_UNSIGNED_INT_24_8, *bpp = 4;
        return true;
    case 126:   /* Z32_FLOAT_S8X24_UINT */
        *fmt = GL_DEPTH_STENCIL, *type = GL_FLOAT_32_UNSIGNED_INT_24_8_REV, *bpp = 8;
        return true;
    }
    return false;
}

/* kind 3: the depth as an integer of scale; 0 = the float's bits, -1 = not kept */
static double sp_depth_scale(uint32_t f)
{
    switch (f) {
    case 16: return 65535.0;
    case 17: return 4294967295.0;
    case 19: case 20: case 21: return 16777215.0;
    case 18: case 126: return 0.0;
    }
    return -1.0;
}

static GLenum sp_gl_target(const SpRes *r)
{
    bool ms = r->nr_samples > 1;

    switch (r->target) {
    case SP_T_1D: return GL_TEXTURE_1D;
    case SP_T_3D: return GL_TEXTURE_3D;
    case SP_T_CUBE: return GL_TEXTURE_CUBE_MAP;
    case SP_T_RECT: return GL_TEXTURE_RECTANGLE;
    case SP_T_1D_ARRAY: return GL_TEXTURE_1D_ARRAY;
    case SP_T_2D_ARRAY: return ms ? GL_TEXTURE_2D_MULTISAMPLE_ARRAY : GL_TEXTURE_2D_ARRAY;
    case SP_T_CUBE_ARRAY: return GL_TEXTURE_CUBE_MAP_ARRAY;
    default: return ms ? GL_TEXTURE_2D_MULTISAMPLE : GL_TEXTURE_2D;
    }
}

/* One level and layer of a texture to the framebuffer's attachment att. */
static void sp_gl_attach(GLenum att, const SpRes *r, GLuint tex, uint32_t level,
                         uint32_t layer)
{
    GLenum t = sp_gl_target(r);

    switch (t) {
    case GL_TEXTURE_3D:
    case GL_TEXTURE_1D_ARRAY:
    case GL_TEXTURE_2D_ARRAY:
    case GL_TEXTURE_CUBE_MAP_ARRAY:
    case GL_TEXTURE_2D_MULTISAMPLE_ARRAY:
        glFramebufferTextureLayer(GL_FRAMEBUFFER, att, tex, level, layer);
        break;
    case GL_TEXTURE_CUBE_MAP:
        glFramebufferTexture2D(GL_FRAMEBUFFER, att,
                               GL_TEXTURE_CUBE_MAP_POSITIVE_X + layer, tex, level);
        break;
    case GL_TEXTURE_1D:
        glFramebufferTexture1D(GL_FRAMEBUFFER, att, t, tex, level);
        break;
    default:
        glFramebufferTexture2D(GL_FRAMEBUFFER, att, t, tex, level);
        break;
    }
}

static void sp_gl_detach_all(void)
{
    static const GLenum atts[] = {
        GL_COLOR_ATTACHMENT0, GL_DEPTH_ATTACHMENT, GL_STENCIL_ATTACHMENT,
    };
    unsigned i;

    for (i = 0; i < G_N_ELEMENTS(atts); i++) {
        glFramebufferTexture2D(GL_FRAMEBUFFER, atts[i], GL_TEXTURE_2D, 0, 0);
    }
}

/* The fixed state of the sink's own passes; each pass changes what it needs. */
static void sp_gl_defaults(void)
{
    glDisable(GL_BLEND);
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_STENCIL_TEST);
    glDisable(GL_SCISSOR_TEST);
    glDisable(GL_SAMPLE_MASK);
    glDisable(GL_CULL_FACE);
    glDisable(GL_SAMPLE_ALPHA_TO_COVERAGE);
    glDisable(GL_DITHER);
    glEnable(GL_FRAMEBUFFER_SRGB); /* sRGB: the decode of texelFetch() undone */
    glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
    glDepthMask(GL_TRUE);
    glStencilMask(0xff);
    glPixelStorei(GL_PACK_ALIGNMENT, 1);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glPixelStorei(GL_PACK_ROW_LENGTH, 0);
    glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);
    glActiveTexture(GL_TEXTURE0);
    glBindVertexArray(spgl.vao);
}

/* the programs, made when first needed */
enum {
    SP_P_FETCH_MS_F, SP_P_FETCH_MS_I, SP_P_FETCH_MS_U,  /* sample s of a MS texture */
    SP_P_DEPTH_OUT,
    SP_P_FETCH_MSA_F, SP_P_FETCH_MSA_I, SP_P_FETCH_MSA_U, /* the same of layer z */
    SP_P_DEPTH_OUTA,                                    /* of a MS array */
    SP_P_COPY_F, SP_P_COPY_I, SP_P_COPY_U,              /* a single-sample texture */
    SP_P_DEPTH_OUT1, SP_P_DEPTH_CMP, SP_P_DEPTH_IN, SP_P_CONST,
    SP_P_STENCIL_IN, SP_P_COUNT,
};

#define SP_P_ARRAY (SP_P_FETCH_MSA_F - SP_P_FETCH_MS_F)

static GLuint sp_prog[SP_P_COUNT];

static const char *const sp_prog_fs[SP_P_COUNT] = {
    [SP_P_FETCH_MS_F] = "uniform sampler2DMS t; uniform int s; out vec4 o;\n"
        "void main() { o = texelFetch(t, ivec2(gl_FragCoord.xy), s); }\n",
    [SP_P_FETCH_MS_I] = "uniform isampler2DMS t; uniform int s; out ivec4 o;\n"
        "void main() { o = texelFetch(t, ivec2(gl_FragCoord.xy), s); }\n",
    [SP_P_FETCH_MS_U] = "uniform usampler2DMS t; uniform int s; out uvec4 o;\n"
        "void main() { o = texelFetch(t, ivec2(gl_FragCoord.xy), s); }\n",
    [SP_P_FETCH_MSA_F] = "uniform sampler2DMSArray t; uniform int s, z; out vec4 o;\n"
        "void main() { o = texelFetch(t, ivec3(gl_FragCoord.xy, z), s); }\n",
    [SP_P_FETCH_MSA_I] = "uniform isampler2DMSArray t; uniform int s, z; out ivec4 o;\n"
        "void main() { o = texelFetch(t, ivec3(gl_FragCoord.xy, z), s); }\n",
    [SP_P_FETCH_MSA_U] = "uniform usampler2DMSArray t; uniform int s, z; out uvec4 o;\n"
        "void main() { o = texelFetch(t, ivec3(gl_FragCoord.xy, z), s); }\n",
    [SP_P_COPY_F] = "uniform sampler2D t; out vec4 o;\n"
        "void main() { o = texelFetch(t, ivec2(gl_FragCoord.xy), 0); }\n",
    [SP_P_COPY_I] = "uniform isampler2D t; out ivec4 o;\n"
        "void main() { o = texelFetch(t, ivec2(gl_FragCoord.xy), 0); }\n",
    [SP_P_COPY_U] = "uniform usampler2D t; out uvec4 o;\n"
        "void main() { o = texelFetch(t, ivec2(gl_FragCoord.xy), 0); }\n",
    [SP_P_DEPTH_OUT] = "uniform sampler2DMS t; uniform int s; out uvec4 o;\n"
        "void main() {\n"
        "  o = uvec4(floatBitsToUint(texelFetch(t, ivec2(gl_FragCoord.xy), s).r));\n"
        "}\n",
    [SP_P_DEPTH_OUTA] = "uniform sampler2DMSArray t; uniform int s, z; out uvec4 o;\n"
        "void main() {\n"
        "  o = uvec4(floatBitsToUint(texelFetch(t, ivec3(gl_FragCoord.xy, z), s).r));\n"
        "}\n",
    [SP_P_DEPTH_OUT1] = "uniform sampler2D t; out uvec4 o;\n"
        "void main() {\n"
        "  o = uvec4(floatBitsToUint(texelFetch(t, ivec2(gl_FragCoord.xy), 0).r));\n"
        "}\n",
    [SP_P_DEPTH_CMP] = "uniform usampler2D t; out vec4 o;\n"
        "void main() {\n"
        "  gl_FragDepth = uintBitsToFloat(texelFetch(t, ivec2(gl_FragCoord.xy), 0).r);\n"
        "  o = vec4(1.0);\n"
        "}\n",
    [SP_P_DEPTH_IN] = "uniform usampler2D t;\n"
        "void main() {\n"
        "  gl_FragDepth = uintBitsToFloat(texelFetch(t, ivec2(gl_FragCoord.xy), 0).r);\n"
        "}\n",
    [SP_P_CONST] = "uniform float v; out vec4 o;\n"
        "void main() { o = vec4(v); }\n",
    [SP_P_STENCIL_IN] = "uniform usampler2D t; uniform int bit;\n"
        "void main() {\n"
        "  uint v = texelFetch(t, ivec2(gl_FragCoord.xy), 0).r;\n"
        "  if (bit >= 0 && ((v >> uint(bit)) & 1u) == 0u) discard;\n"
        "}\n",
};

static GLuint sp_gl_shader(GLenum type, const char *body)
{
    const char *src[2] = { "#version 330 core\n", body };
    GLuint s = glCreateShader(type);
    GLint ok = 0;

    glShaderSource(s, 2, src, NULL);
    glCompileShader(s);
    glGetShaderiv(s, GL_COMPILE_STATUS, &ok);
    if (!ok) {
        char log[1024] = "";

        glGetShaderInfoLog(s, sizeof(log), NULL, log);
        fprintf(stderr, "[SPACEBOX-REMOTE] sink: a shader of its own does not "
                "compile: %s\n", log);
        glDeleteShader(s);
        return 0;
    }
    return s;
}

static GLuint sp_gl_prog(unsigned which)
{
    /* a triangle that covers the viewport, from gl_VertexID alone */
    static const char vs[] =
        "void main() {\n"
        "  gl_Position = vec4(float((gl_VertexID & 1) * 4 - 1),\n"
        "                     float((gl_VertexID & 2) * 2 - 1), 0.0, 1.0);\n"
        "}\n";
    GLuint v, f, p;
    GLint ok = 0;

    if (sp_prog[which]) {
        glUseProgram(sp_prog[which]);
        return sp_prog[which];
    }
    v = sp_gl_shader(GL_VERTEX_SHADER, vs);
    f = sp_gl_shader(GL_FRAGMENT_SHADER, sp_prog_fs[which]);
    if (!v || !f) {
        return 0;
    }
    p = glCreateProgram();
    glAttachShader(p, v);
    glAttachShader(p, f);
    glLinkProgram(p);
    glDeleteShader(v);
    glDeleteShader(f);
    glGetProgramiv(p, GL_LINK_STATUS, &ok);
    if (!ok) {
        glDeleteProgram(p);
        return 0;
    }
    glUseProgram(p);
    glUniform1i(glGetUniformLocation(p, "t"), 0);
    sp_prog[which] = p;
    return p;
}

/*
 * Program p (one of the four reads of a multisampled texture), or its twin for
 * an array when r is one, made current and told sample s and layer z.
 */
static GLuint sp_gl_prog_ms(unsigned p, const SpRes *r, GLint s, uint32_t z)
{
    GLuint prog;

    if (r->target == SP_T_2D_ARRAY) {
        p += SP_P_ARRAY;
    }
    prog = sp_gl_prog(p);
    if (prog) {
        glUniform1i(glGetUniformLocation(prog, "s"), s);
        glUniform1i(glGetUniformLocation(prog, "z"), z);
    }
    return prog;
}

static void sp_gl_draw(uint32_t w, uint32_t h, uint32_t y0, uint32_t rows)
{
    glViewport(0, 0, w, h);
    glEnable(GL_SCISSOR_TEST);
    glScissor(0, y0, w, rows);
    glDrawArrays(GL_TRIANGLES, 0, 3);
    glDisable(GL_SCISSOR_TEST);
}

static GLuint sp_gl_texture(GLenum ifmt, GLenum fmt, GLenum type, uint32_t w,
                            uint32_t h)
{
    GLuint t;

    glGenTextures(1, &t);
    glBindTexture(GL_TEXTURE_2D, t);
    glTexImage2D(GL_TEXTURE_2D, 0, ifmt, w, h, 0, fmt, type, NULL);
    /* complete for texelFetch() also with an integer format */
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAX_LEVEL, 0);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    return t;
}

static const char *sp_gl_error(const char *what)
{
    static char buf[96];
    GLenum e = glGetError();

    if (e == GL_NO_ERROR) {
        return NULL;
    }
    while (glGetError() != GL_NO_ERROR) {
    }
    snprintf(buf, sizeof(buf), "GL error 0x%x in %s", e, what);
    return buf;
}

static const char *sp_gl_complete(void)
{
    GLenum st = glCheckFramebufferStatus(GL_FRAMEBUFFER);
    static char buf[64];

    if (st == GL_FRAMEBUFFER_COMPLETE) {
        return NULL;
    }
    snprintf(buf, sizeof(buf), "framebuffer incomplete (0x%x)", st);
    return buf;
}

/* The layers of a multisampled texture: an array's, else one. */
static uint32_t sp_ms_layers(const SpRes *r)
{
    return r->target == SP_T_2D_ARRAY ? MAX(r->array_size, 1) : 1;
}

/* What a multisampled texture is in GL; NULL or why it is not kept. */
static const char *sp_gl_ms_info(const SpRes *r, GLuint tex, GLint *samples,
                                 GLint *ifmt, GLint *fixed)
{
    GLenum t = sp_gl_target(r);

    if ((t != GL_TEXTURE_2D_MULTISAMPLE && t != GL_TEXTURE_2D_MULTISAMPLE_ARRAY) ||
        (r->target == SP_T_2D && r->array_size > 1)) {
        return "a multisampled texture of a target this sink does not keep";
    }
    glBindTexture(t, tex);
    glGetTexLevelParameteriv(t, 0, GL_TEXTURE_SAMPLES, samples);
    glGetTexLevelParameteriv(t, 0, GL_TEXTURE_INTERNAL_FORMAT, ifmt);
    glGetTexLevelParameteriv(t, 0, GL_TEXTURE_FIXED_SAMPLE_LOCATIONS, fixed);
    glBindTexture(t, 0);
    if (sp_gl_error("the texture's parameters") || *samples < 2) {
        return "not a multisampled texture in the renderer";
    }
    return NULL;
}

/* a single-sample 2D texture of the sink's own, for sp_gl_attach() */
static const SpRes sp_res_2d = { .target = SP_T_2D };

static uint32_t sp_rows_per_part(uint32_t w, unsigned bpp)
{
    return MAX(SP_XFER_MAX / MAX((uint64_t)w * bpp, 1), 1);
}

/* Read rows of the framebuffer's read buffer into parts of `kind`. */
static void sp_gl_read_parts(SpParts *parts, const SpRes *r, uint32_t kind,
                             uint32_t level, uint32_t layer, uint32_t sample,
                             uint32_t w, uint32_t h, GLenum fmt, GLenum type,
                             unsigned bpp)
{
    uint32_t step = sp_rows_per_part(w, bpp), y;

    for (y = 0; y < h; y += step) {
        uint32_t n = MIN(step, h - y);
        SpXfer x = { .res = r->id, .kind = kind, .level = level,
                     .sample = sample, .box = { 0, y, layer, w, n, 1 } };
        uint8_t *data;

        g_ptr_array_add(parts, sp_part_new(&x, (size_t)w * n * bpp, &data));
        glReadPixels(0, y, w, n, fmt, type, data);
    }
}

static int sp_kind1_way(const SpRes *r, GLuint tex, uint32_t level,
                        const char **why);

/* kind 1 */
static const char *sp_export_depth(const SpRes *r, GLuint tex, SpParts *parts)
{
    GLenum fmt, type, att = sp_gl_attachment(r->format);
    unsigned bpp;
    uint32_t level, w, h, layers, z;
    const char *why;

    if (!sp_depth_fmt(r->format, &fmt, &type, &bpp)) {
        return "a depth format this sink does not read";
    }
    if (sp_kind1_way(r, tex, 0, &why) == -2) {
        return why;     /* what this renderer cannot write back, it does not hold exactly */
    }
    glDrawBuffer(GL_NONE);
    glReadBuffer(GL_NONE);
    for (level = 0; level <= r->last_level && level < 32; level++) {
        sp_level_dims(r, level, &w, &h, &layers);
        for (z = 0; z < layers; z++) {
            sp_gl_attach(att, r, tex, level, z);
            if ((why = sp_gl_complete())) {
                return why;
            }
            unsigned first = parts->len, k;

            sp_gl_read_parts(parts, r, SP_K_DEPTH, level, z, 0, w, h, fmt,
                             type, bpp);
            for (k = first; r->format == 21 && k < parts->len; k++) {
                /* Z24X8 as UNSIGNED_INT: the low byte is not the depth's
                 * (Mesa repeats high bits there, Apple's GL other ones) */
                GByteArray *p = parts->pdata[k];
                size_t i;

                for (i = sizeof(SpXfer); i + 4 <= p->len; i += 4) {
                    p->data[i] = 0;
                }
            }
        }
    }
    sp_gl_detach_all();
    return sp_gl_error("the read of depth");
}

static const char *sp_import_depth(const SpRes *r, GLuint tex, const SpXfer *x,
                                   GByteArray *b)
{
    GLenum fmt, type, t = sp_gl_target(r);
    unsigned bpp;
    uint32_t w = x->box[3], n = x->box[4], y = x->box[1], z = x->box[2];

    if (!sp_depth_fmt(r->format, &fmt, &type, &bpp)) {
        return "a depth format this sink does not write";
    }
    if (b->len < (uint64_t)w * n * bpp) {
        return "too few bytes";
    }
    glBindTexture(t, tex);
    switch (t) {
    case GL_TEXTURE_3D:
    case GL_TEXTURE_2D_ARRAY:
    case GL_TEXTURE_CUBE_MAP_ARRAY:
        glTexSubImage3D(t, x->level, 0, y, z, w, n, 1, fmt, type, b->data);
        break;
    case GL_TEXTURE_1D_ARRAY:
        glTexSubImage2D(t, x->level, 0, z, w, 1, fmt, type, b->data);
        break;
    case GL_TEXTURE_CUBE_MAP:
        glTexSubImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_X + z, x->level, 0, y, w, n,
                        fmt, type, b->data);
        break;
    case GL_TEXTURE_1D:
        glTexSubImage1D(t, x->level, 0, w, fmt, type, b->data);
        break;
    default:
        glTexSubImage2D(t, x->level, 0, y, w, n, fmt, type, b->data);
        break;
    }
    glBindTexture(t, 0);
    return sp_gl_error("the write of depth");
}

/* kind 4 of a single-sample stencil-only texture: each level and layer */
static const char *sp_export_stencil1(const SpRes *r, GLuint tex, SpParts *parts)
{
    uint32_t level, w, h, layers, z;
    const char *why;

    glDrawBuffer(GL_NONE);
    glReadBuffer(GL_NONE);
    for (level = 0; level <= r->last_level && level < 32; level++) {
        sp_level_dims(r, level, &w, &h, &layers);
        for (z = 0; z < layers; z++) {
            sp_gl_attach(GL_STENCIL_ATTACHMENT, r, tex, level, z);
            if ((why = sp_gl_complete())) {
                return why;
            }
            sp_gl_read_parts(parts, r, SP_K_STENCIL, level, z, 0, w, h,
                             GL_STENCIL_INDEX, GL_UNSIGNED_BYTE, 1);
        }
    }
    sp_gl_detach_all();
    return sp_gl_error("the read of stencil");
}

/* kind 3: between the depth as an integer and the float of the shaders */
static uint32_t sp_depth_of_float(uint32_t bits, double scale)
{
    float f;
    double d;

    if (!scale) {
        return bits;
    }
    memcpy(&f, &bits, 4);
    d = (double)f * scale;
    return !(d > 0) ? 0 : d >= scale ? (uint32_t)scale : (uint32_t)(d + 0.5);
}

static uint32_t sp_depth_to_float(uint32_t v, double scale)
{
    float f;
    uint32_t bits;

    if (!scale) {
        return v;
    }
    f = (float)(v / scale);
    memcpy(&bits, &f, 4);
    return bits;
}

/* One depth value out of a texel that glReadPixels() gave for kind 1. */
static uint32_t sp_depth_of_texel(uint32_t format, const uint8_t *p)
{
    uint32_t v;
    uint16_t v16;

    switch (format) {
    case 16:
        memcpy(&v16, p, 2);
        return v16;
    case 17:
    case 18:
    case 126:
        memcpy(&v, p, 4);
        return v;
    default:            /* the 24 bits of UNSIGNED_INT or UNSIGNED_INT_24_8 */
        memcpy(&v, p, 4);
        return v >> 8;
    }
}

/*
 * How this renderer keeps the depth of a format through kind 3: values of
 * the whole range written by gl_FragDepth into a single-sample texture of the
 * same internal format must come back from glReadPixels() (exact), and the
 * same values should come back from texelFetch(). Once per format and internal
 * format (Apple's GL gives GL_DEPTH_COMPONENT32 for Z24X8 as for Z32).
 * Returns SP_DEPTH_FETCH (both exact), SP_DEPTH_SEARCH (only the writes are:
 * read by the depth test), or SP_DEPTH_NO with *why.
 */
enum { SP_DEPTH_NO = -1, SP_DEPTH_FETCH, SP_DEPTH_SEARCH };

static int sp_depth_selfcheck(uint32_t format, GLint ifmt, const char **whyp)
{
    static struct {
        uint32_t format;
        GLint ifmt;
        int mode;
        char why[192];
    } seen[8];
    static unsigned nseen;
    const uint32_t w = 256, h = 256, n = w * h;
    double scale = sp_depth_scale(format);
    g_autofree uint32_t *want = g_new(uint32_t, n);
    g_autofree uint32_t *io = g_new(uint32_t, n);
    g_autofree uint8_t *texels = NULL;
    unsigned i, bad_w = 0, bad_r = 0, maxd = 0;
    GLenum fmt, type;
    unsigned bpp;
    GLuint dt, st;
    const char *why = NULL;
    char *res;

    int mode;

    for (i = 0; i < nseen; i++) {
        if (seen[i].format == format && seen[i].ifmt == ifmt) {
            *whyp = seen[i].why;
            return seen[i].mode;
        }
    }
    if (!sp_depth_fmt(format, &fmt, &type, &bpp) || scale < 0) {
        *whyp = "a depth format this sink does not keep";
        return SP_DEPTH_NO;
    }
    for (i = 0; i < n; i++) {
        if (!scale) {
            float f = (float)i / (n - 1);

            memcpy(&want[i], &f, 4);
        } else if (scale > 16777215) {
            want[i] = i << 16 | ((i * 40503u) & 0xffff);
        } else if (scale > 65535) {
            want[i] = MIN((uint64_t)i * 256 + ((i * 37) & 255), (uint32_t)scale);
        } else {
            want[i] = i;
        }
        io[i] = sp_depth_to_float(want[i], scale);
    }
    texels = g_malloc((size_t)n * bpp);
    glGenTextures(1, &dt);
    glBindTexture(GL_TEXTURE_2D, dt);
    glTexImage2D(GL_TEXTURE_2D, 0, ifmt, w, h, 0, fmt, type, NULL);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAX_LEVEL, 0);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    st = sp_gl_texture(GL_R32UI, GL_RED_INTEGER, GL_UNSIGNED_INT, w, h);
    glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, w, h, GL_RED_INTEGER, GL_UNSIGNED_INT, io);
    /* the write: gl_FragDepth, then glReadPixels */
    glFramebufferTexture2D(GL_FRAMEBUFFER, sp_gl_attachment(format), GL_TEXTURE_2D,
                           dt, 0);
    glDrawBuffer(GL_NONE);
    glReadBuffer(GL_NONE);
    if ((why = sp_gl_complete()) || !sp_gl_prog(SP_P_DEPTH_IN)) {
        why = why ? why : "no program";
        goto out;
    }
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_ALWAYS);
    sp_gl_draw(w, h, 0, h);
    glDisable(GL_DEPTH_TEST);
    glReadPixels(0, 0, w, h, fmt, type, texels);
    for (i = 0; i < n; i++) {
        uint32_t got = sp_depth_of_texel(format, texels + (size_t)i * bpp);

        if (got != want[i]) {
            bad_w++;
            maxd = MAX(maxd, (unsigned)ABS((int64_t)got - want[i]));
        }
    }
    sp_gl_detach_all();
    /* the read: texelFetch() of what glReadPixels() showed to be there */
    glBindTexture(GL_TEXTURE_2D, st);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, st, 0);
    glDrawBuffer(GL_COLOR_ATTACHMENT0);
    glReadBuffer(GL_COLOR_ATTACHMENT0);
    glBindTexture(GL_TEXTURE_2D, dt);
    if ((why = sp_gl_complete()) || !sp_gl_prog(SP_P_DEPTH_OUT1)) {
        why = why ? why : "no program";
        goto out;
    }
    sp_gl_draw(w, h, 0, h);
    glReadPixels(0, 0, w, h, GL_RED_INTEGER, GL_UNSIGNED_INT, io);
    for (i = 0; i < n; i++) {
        uint32_t there = sp_depth_of_texel(format, texels + (size_t)i * bpp);
        uint32_t got = sp_depth_of_float(io[i], scale);

        if (got != there) {
            bad_r++;
            maxd = MAX(maxd, (unsigned)ABS((int64_t)got - there));
        }
    }
    why = sp_gl_error("the depth self-check");
  out:
    sp_gl_detach_all();
    glBindTexture(GL_TEXTURE_2D, 0);
    glDeleteTextures(1, &dt);
    glDeleteTextures(1, &st);
    if (nseen < G_N_ELEMENTS(seen)) {
        res = seen[nseen].why;
        seen[nseen].format = format;
        seen[nseen].ifmt = ifmt;
    } else {
        static char spare[192];

        res = spare;
    }
    mode = why || bad_w ? SP_DEPTH_NO : bad_r ? SP_DEPTH_SEARCH : SP_DEPTH_FETCH;
    if (why) {
        snprintf(res, 192, "the depth self-check failed: %s", why);
    } else if (bad_w) {
        snprintf(res, 192, "this renderer does not keep multisampled depth "
                 "exactly (format %u: of %u values written by gl_FragDepth, "
                 "%u came out wrong, by up to %u)", format, n, bad_w, maxd);
    } else if (bad_r) {
        snprintf(res, 192, "gl_FragDepth exact; texelFetch() of %u of %u "
                 "values off by up to %u: read by the depth test", bad_r, n,
                 maxd);
    } else {
        snprintf(res, 192, "gl_FragDepth and texelFetch() exact");
    }
    if (nseen < G_N_ELEMENTS(seen)) {
        seen[nseen++].mode = mode;
    }
    fprintf(stderr, "[SPACEBOX-REMOTE] sink depth self-check of format %u (GL "
            "0x%x): %s\n", format, ifmt, res);
    *whyp = res;
    return mode;
}

/*
 * Sample s of a multisampled depth texture by the depth test (SP_DEPTH_SEARCH):
 * for each bit from the highest, a pass draws the found bits with this one set
 * as the fragment's depth, with the test LEQUAL and the sample mask on s, into
 * an R8 texture of the same samples; where it passes, the stored value has the
 * bit. Then sample s of the R8 texture says where.
 */
static const char *sp_depth_search(const SpRes *r, GLuint tex, GLint ns,
                                   GLint fixed, GLint s, uint32_t z,
                                   SpParts *parts)
{
    uint32_t w = MAX(r->width, 1), h = MAX(r->height, 1), n = w * h;
    double scale = sp_depth_scale(r->format);
    unsigned bits = scale > 16777215 ? 32 : scale > 65535 ? 24 : 16, bit, step, y;
    g_autofree uint32_t *found = g_new0(uint32_t, n);
    g_autofree uint32_t *io = g_new(uint32_t, n);
    g_autofree uint8_t *hit = g_malloc(n);
    const char *why = NULL;
    GLuint ms8, st, r8;
    size_t i;

    glGenTextures(1, &ms8);
    glBindTexture(GL_TEXTURE_2D_MULTISAMPLE, ms8);
    glTexImage2DMultisample(GL_TEXTURE_2D_MULTISAMPLE, ns, GL_R8, w, h, fixed);
    r8 = sp_gl_texture(GL_R8, GL_RED, GL_UNSIGNED_BYTE, w, h);
    st = sp_gl_texture(GL_R32UI, GL_RED_INTEGER, GL_UNSIGNED_INT, w, h);
    for (bit = bits; bit-- > 0;) {
        for (i = 0; i < n; i++) {
            io[i] = sp_depth_to_float(found[i] | 1u << bit, scale);
        }
        glBindTexture(GL_TEXTURE_2D, st);
        glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, w, h, GL_RED_INTEGER,
                        GL_UNSIGNED_INT, io);
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0,
                               GL_TEXTURE_2D_MULTISAMPLE, ms8, 0);
        sp_gl_attach(GL_DEPTH_ATTACHMENT, r, tex, 0, z);
        glDrawBuffer(GL_COLOR_ATTACHMENT0);
        if ((why = sp_gl_complete()) || !sp_gl_prog(SP_P_DEPTH_CMP)) {
            why = why ? why : "no program";
            break;
        }
        glClearBufferfv(GL_COLOR, 0, (const GLfloat[4]){ 0 });
        glEnable(GL_DEPTH_TEST);
        glDepthFunc(GL_LEQUAL);
        glDepthMask(GL_FALSE);
        glEnable(GL_SAMPLE_MASK);
        glSampleMaski(0, 1u << s);
        sp_gl_draw(w, h, 0, h);
        glDisable(GL_SAMPLE_MASK);
        glDisable(GL_DEPTH_TEST);
        glDepthMask(GL_TRUE);
        sp_gl_detach_all();
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0,
                               GL_TEXTURE_2D, r8, 0);
        glDrawBuffer(GL_COLOR_ATTACHMENT0);
        glReadBuffer(GL_COLOR_ATTACHMENT0);
        if ((why = sp_gl_complete()) ||
            !sp_gl_prog_ms(SP_P_FETCH_MS_F, &sp_res_2d, s, 0)) {
            why = why ? why : "no program";
            break;
        }
        glBindTexture(GL_TEXTURE_2D_MULTISAMPLE, ms8);
        sp_gl_draw(w, h, 0, h);
        glReadPixels(0, 0, w, h, GL_RED, GL_UNSIGNED_BYTE, hit);
        sp_gl_detach_all();
        for (i = 0; i < n; i++) {
            if (hit[i] >= 128) {
                found[i] |= 1u << bit;
            }
        }
    }
    sp_gl_detach_all();
    glBindTexture(GL_TEXTURE_2D, 0);
    glBindTexture(GL_TEXTURE_2D_MULTISAMPLE, 0);
    glDeleteTextures(1, &ms8);
    glDeleteTextures(1, &r8);
    glDeleteTextures(1, &st);
    if (why) {
        return why;
    }
    step = sp_rows_per_part(w, 4);
    for (y = 0; y < h; y += step) {
        uint32_t rows = MIN(step, h - y);
        SpXfer x = { .res = r->id, .kind = SP_K_MS_DEPTH, .sample = s,
                     .box = { 0, y, z, w, rows, 1 } };
        uint8_t *data;

        g_ptr_array_add(parts, sp_part_new(&x, (size_t)w * rows * 4, &data));
        memcpy(data, found + (size_t)y * w, (size_t)w * rows * 4);
    }
    return sp_gl_error("the depth search");
}

/* The stencil passes of kinds 1 and 4: st holds the values (R8UI), bound. */
static void sp_gl_stencil_passes(uint32_t w, uint32_t h, uint32_t y, uint32_t n)
{
    GLint loc = glGetUniformLocation(sp_prog[SP_P_STENCIL_IN], "bit");
    unsigned bit;

    glEnable(GL_STENCIL_TEST);
    glStencilOp(GL_KEEP, GL_KEEP, GL_REPLACE);
    /* the stencil to 0, then each bit that is 1 */
    glStencilFunc(GL_ALWAYS, 0, 0xff);
    glStencilMask(0xff);
    glUniform1i(loc, -1);
    sp_gl_draw(w, h, y, n);
    glStencilFunc(GL_ALWAYS, 0xff, 0xff);
    for (bit = 0; bit < 8; bit++) {
        glStencilMask(1u << bit);
        glUniform1i(loc, bit);
        sp_gl_draw(w, h, y, n);
    }
    glStencilMask(0xff);
    glDisable(GL_STENCIL_TEST);
}

/* kind 1 back by drawing: the depth through gl_FragDepth, then the stencil */
static const char *sp_import_depth_draw(const SpRes *r, GLuint tex,
                                        const SpXfer *x, GByteArray *b)
{
    uint32_t lw, lh, layers, w = x->box[3], n = x->box[4], y = x->box[1];
    double scale = sp_depth_scale(r->format);
    g_autofree uint32_t *v = NULL;
    g_autofree uint8_t *st8 = NULL;
    const char *why = NULL;
    GLenum fmt, type;
    unsigned bpp;
    GLuint st;
    size_t i, nt = (size_t)w * n;

    sp_level_dims(r, x->level, &lw, &lh, &layers);
    if (!sp_depth_fmt(r->format, &fmt, &type, &bpp) || scale < 0) {
        return "a depth format this sink does not write";
    }
    if (x->box[0] || w != lw || y + n > lh || x->box[2] >= layers) {
        return "a box this sink did not export";
    }
    if (b->len < nt * bpp) {
        return "too few bytes";
    }
    v = g_new(uint32_t, nt);
    st8 = g_malloc(nt);
    for (i = 0; i < nt; i++) {
        const uint8_t *t = b->data + i * bpp;

        v[i] = sp_depth_to_float(sp_depth_of_texel(r->format, t), scale);
        st8[i] = r->format == 126 ? t[4] : t[0];
    }
    st = sp_gl_texture(GL_R32UI, GL_RED_INTEGER, GL_UNSIGNED_INT, lw, lh);
    glTexSubImage2D(GL_TEXTURE_2D, 0, 0, y, w, n, GL_RED_INTEGER, GL_UNSIGNED_INT, v);
    sp_gl_attach(GL_DEPTH_ATTACHMENT, r, tex, x->level, x->box[2]);
    glDrawBuffer(GL_NONE);
    glReadBuffer(GL_NONE);
    if ((why = sp_gl_complete()) || !sp_gl_prog(SP_P_DEPTH_IN)) {
        why = why ? why : "no program";
        goto out;
    }
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_ALWAYS);
    glDepthMask(GL_TRUE);
    sp_gl_draw(lw, lh, y, n);
    glDisable(GL_DEPTH_TEST);
    sp_gl_detach_all();
    if (sp_has_stencil(r->format)) {
        glDeleteTextures(1, &st);
        st = sp_gl_texture(GL_R8UI, GL_RED_INTEGER, GL_UNSIGNED_BYTE, lw, lh);
        glTexSubImage2D(GL_TEXTURE_2D, 0, 0, y, w, n, GL_RED_INTEGER,
                        GL_UNSIGNED_BYTE, st8);
        sp_gl_attach(GL_STENCIL_ATTACHMENT, r, tex, x->level, x->box[2]);
        if ((why = sp_gl_complete()) || !sp_gl_prog(SP_P_STENCIL_IN)) {
            why = why ? why : "no program";
            goto out;
        }
        sp_gl_stencil_passes(lw, lh, y, n);
    }
  out:
    sp_gl_detach_all();
    glBindTexture(GL_TEXTURE_2D, 0);
    glDeleteTextures(1, &st);
    return why ? why : sp_gl_error("the write of depth");
}

/* A texel of kind 1 for the check below: value i of n over the format's range. */
static void sp_depth_texel(uint32_t format, uint32_t i, uint32_t n, uint8_t *p)
{
    uint32_t v = i << 16 | ((i * 40503u) & 0xffff);     /* 32 bits */
    float f = (float)i / (n - 1);
    uint16_t v16 = i;

    switch (format) {
    case 16:
        memcpy(p, &v16, 2);
        break;
    case 18:
        memcpy(p, &f, 4);
        break;
    case 126:
        memcpy(p, &f, 4);
        memset(p + 4, 0, 4);
        p[4] = i * 37;
        break;
    case 17:
        memcpy(p, &v, 4);
        break;
    default:            /* the 24 bits of the depth, and the stencil or 0 */
        v = (v >> 8) << 8 | (format == 21 ? 0 : (i * 37) & 255);
        memcpy(p, &v, 4);
        break;
    }
}

/*
 * Kind 1 written by glTexSubImage (where gl_FragDepth is not exact): values
 * over the whole range must come back from glReadPixels() as they were
 * written, or the format is not kept (NULL, or why not). Apple's GL appears to
 * keep Z32_UNORM as a float: a 32-bit value written there comes back off by
 * one. Once per format and internal format.
 */
static const char *sp_depth_texcheck(uint32_t format, GLint ifmt)
{
    static struct {
        uint32_t format;
        GLint ifmt;
        bool bad;
        char why[192];
    } seen[8];
    static unsigned nseen;
    const uint32_t w = 256, h = 256, n = w * h;
    g_autofree uint8_t *want = NULL, *got = NULL;
    unsigned i, bad = 0, maxd = 0;
    const char *why = NULL;
    GLenum fmt, type;
    unsigned bpp;
    GLuint t;

    for (i = 0; i < nseen; i++) {
        if (seen[i].format == format && seen[i].ifmt == ifmt) {
            return seen[i].bad ? seen[i].why : NULL;
        }
    }
    if (!sp_depth_fmt(format, &fmt, &type, &bpp)) {
        return "a depth format this sink does not keep";
    }
    want = g_malloc((size_t)n * bpp);
    got = g_malloc0((size_t)n * bpp);
    for (i = 0; i < n; i++) {
        sp_depth_texel(format, i, n, want + (size_t)i * bpp);
    }
    glGenTextures(1, &t);
    glBindTexture(GL_TEXTURE_2D, t);
    glTexImage2D(GL_TEXTURE_2D, 0, ifmt, w, h, 0, fmt, type, NULL);
    glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, w, h, fmt, type, want);
    glBindTexture(GL_TEXTURE_2D, 0);
    glFramebufferTexture2D(GL_FRAMEBUFFER, sp_gl_attachment(format), GL_TEXTURE_2D, t, 0);
    glDrawBuffer(GL_NONE);
    glReadBuffer(GL_NONE);
    if (!(why = sp_gl_complete())) {
        glReadPixels(0, 0, w, h, fmt, type, got);
        why = sp_gl_error("the glTexSubImage check");
    }
    sp_gl_detach_all();
    glDeleteTextures(1, &t);
    for (i = 0; !why && i < n; i++) {
        const uint8_t *a = want + (size_t)i * bpp, *b = got + (size_t)i * bpp;

        uint32_t da = sp_depth_of_texel(format, a), db = sp_depth_of_texel(format, b);

        if (da != db ||
            (sp_has_stencil(format) && (format == 126 ? a[4] != b[4] : a[0] != b[0]))) {
            bad++;
            maxd = MAX(maxd, (unsigned)ABS((int64_t)da - db));
        }
    }
    if (nseen < G_N_ELEMENTS(seen)) {
        seen[nseen].format = format;
        seen[nseen].ifmt = ifmt;
        seen[nseen].bad = why || bad;
        if (why) {
            snprintf(seen[nseen].why, 192, "the glTexSubImage check failed: %s", why);
        } else {
            snprintf(seen[nseen].why, 192, "this renderer does not keep depth format "
                     "%u exactly (of %u values written by glTexSubImage, %u came back "
                     "different, by up to %u)", format, n, bad, maxd);
        }
        why = seen[nseen].bad ? seen[nseen].why : NULL;
        fprintf(stderr, "[SPACEBOX-REMOTE] sink glTexSubImage check of format %u (GL "
                "0x%x): %s\n", format, ifmt, why ? why : "exact");
        nseen++;
    } else if (bad && !why) {
        why = "this renderer does not keep this depth format exactly";
    }
    return why;
}

/*
 * Z32_UNORM where this renderer cannot keep it exactly: kept with rounding,
 * as the user chose on 2026-10-17 (see the kinds above); logged once per way.
 */
static void sp_depth_rounded(uint32_t format, bool ms, const char *why)
{
    static bool said[2];

    if (!said[ms]) {
        said[ms] = true;
        fprintf(stderr, "[SPACEBOX-REMOTE] sink keeps %s depth format %u with "
                "rounding (%s)\n", ms ? "multisampled" : "single-sample", format,
                why);
    }
}

/*
 * How kind 1 is written into r: by drawing (SP_DEPTH_FETCH or SP_DEPTH_SEARCH),
 * by glTexSubImage where gl_FragDepth is not exact (SP_DEPTH_NO), or not at all
 * (-2, *why) where neither is.
 */
static int sp_kind1_way(const SpRes *r, GLuint tex, uint32_t level, const char **why)
{
    GLenum t = sp_gl_target(r);
    GLint ifmt = 0;
    int mode;

    glBindTexture(t, tex);
    glGetTexLevelParameteriv(t == GL_TEXTURE_CUBE_MAP ?
                             GL_TEXTURE_CUBE_MAP_POSITIVE_X : t, level,
                             GL_TEXTURE_INTERNAL_FORMAT, &ifmt);
    glBindTexture(t, 0);
    mode = sp_depth_selfcheck(r->format, ifmt, why);
    if (mode == SP_DEPTH_NO && (*why = sp_depth_texcheck(r->format, ifmt))) {
        if (r->format == 17) {
            /* Z32_UNORM: kept with rounding (see the kinds above) */
            sp_depth_rounded(r->format, false, *why);
            return SP_DEPTH_NO;
        }
        return -2;
    }
    return mode;
}

static const char *sp_import_kind1(const SpRes *r, GLuint tex, const SpXfer *x,
                                   GByteArray *b)
{
    const char *why;

    switch (sp_kind1_way(r, tex, x->level, &why)) {
    case -2:
        return why;
    case SP_DEPTH_NO:
        return sp_import_depth(r, tex, x, b);
    default:
        return sp_import_depth_draw(r, tex, x, b);
    }
}

/* kind 2 of a snorm format: see the kinds above */
static bool sp_gl_copy_image(void)
{
    static int has = -1;

    if (has < 0) {
        has = epoxy_gl_version() >= 43 || epoxy_has_gl_extension("GL_ARB_copy_image");
        if (!has) {
            fprintf(stderr, "[SPACEBOX-REMOTE] sink keeps multisampled snorm through "
                    "floats (no glCopyImageSubData): the lowest value arrives as the "
                    "lowest + 1\n");
        }
    }
    return has;
}

/* the components and the largest value of a snorm format's own type */
static unsigned sp_snorm_comps(const SpFmt *f, double *mx)
{
    *mx = f->type == GL_BYTE ? 127.0 : 32767.0;
    return f->type == GL_BYTE ? f->bpp : f->bpp / 2;
}

/* kind 2 of a snorm format through floats (no glCopyImageSubData) */
static const char *sp_export_ms_snorm_float(const SpRes *r, GLuint tex, GLint ns,
                                            const SpFmt *f, SpParts *parts)
{
    uint32_t w = MAX(r->width, 1), h = MAX(r->height, 1), z, y;
    uint32_t step = sp_rows_per_part(w, f->bpp);
    g_autofree float *px = g_new(float, (size_t)w * h * 4);
    double mx;
    unsigned comps = sp_snorm_comps(f, &mx);
    GLenum t = sp_gl_target(r);
    const char *why;
    GLuint st;
    GLint s;

    st = sp_gl_texture(GL_RGBA32F, GL_RGBA, GL_FLOAT, w, h);
    glBindTexture(GL_TEXTURE_2D, 0);
    sp_gl_attach(GL_COLOR_ATTACHMENT0, &sp_res_2d, st, 0, 0);
    glDrawBuffer(GL_COLOR_ATTACHMENT0);
    glReadBuffer(GL_COLOR_ATTACHMENT0);
    why = sp_gl_complete();
    for (z = 0; !why && z < sp_ms_layers(r); z++) {
        for (s = 0; !why && s < ns; s++) {
            glBindTexture(t, tex);
            if (!sp_gl_prog_ms(SP_P_FETCH_MS_F, r, s, z)) {
                why = "no program";
                break;
            }
            sp_gl_draw(w, h, 0, h);
            glReadPixels(0, 0, w, h, GL_RGBA, GL_FLOAT, px);
            for (y = 0; y < h; y += step) {
                uint32_t n = MIN(step, h - y);
                SpXfer x = { .res = r->id, .kind = SP_K_MS_COLOR, .sample = s,
                             .box = { 0, y, z, w, n, 1 } };
                uint8_t *data;
                size_t i;
                unsigned c;

                g_ptr_array_add(parts, sp_part_new(&x, (size_t)w * n * f->bpp, &data));
                for (i = 0; i < (size_t)w * n; i++) {
                    const float *p = px + ((size_t)y * w + i) * 4;

                    for (c = 0; c < comps; c++) {
                        double v = MAX(MIN(rint(p[c] * mx), mx), -mx);

                        if (f->type == GL_BYTE) {
                            data[i * comps + c] = (uint8_t)(int8_t)v;
                        } else {
                            int16_t v16 = (int16_t)v;

                            memcpy(data + (i * comps + c) * 2, &v16, 2);
                        }
                    }
                }
            }
        }
        why = why ? why : sp_gl_error("the read of samples through floats");
    }
    sp_gl_detach_all();
    glBindTexture(t, 0);
    glBindTexture(GL_TEXTURE_2D, 0);
    glDeleteTextures(1, &st);
    return why;
}

/* A multisampled texture of `twin` (the samples and locations of the target). */
static GLuint sp_gl_ms_twin(const SpFmt *f, GLint ns, GLint fixed, uint32_t w,
                            uint32_t h)
{
    GLuint t;

    glGenTextures(1, &t);
    glBindTexture(GL_TEXTURE_2D_MULTISAMPLE, t);
    glTexImage2DMultisample(GL_TEXTURE_2D_MULTISAMPLE, ns, f->twin, w, h, fixed);
    glBindTexture(GL_TEXTURE_2D_MULTISAMPLE, 0);
    return t;
}

static const char *sp_export_ms_snorm(const SpRes *r, GLuint tex, GLint ns,
                                      GLint fixed, const SpFmt *f, SpParts *parts)
{
    const SpFmt *in = sp_fmt_of(f->twin);
    uint32_t w = MAX(r->width, 1), h = MAX(r->height, 1), z;
    const char *why;
    GLuint ms, st;
    GLint s;

    if (!sp_gl_copy_image()) {
        return sp_export_ms_snorm_float(r, tex, ns, f, parts);
    }
    ms = sp_gl_ms_twin(f, ns, fixed, w, h);
    st = sp_gl_texture(in->ifmt, in->fmt, in->type, w, h);
    glBindTexture(GL_TEXTURE_2D, 0);
    sp_gl_attach(GL_COLOR_ATTACHMENT0, &sp_res_2d, st, 0, 0);
    glDrawBuffer(GL_COLOR_ATTACHMENT0);
    glReadBuffer(GL_COLOR_ATTACHMENT0);
    why = sp_gl_complete();
    for (z = 0; !why && z < sp_ms_layers(r); z++) {
        glCopyImageSubData(tex, sp_gl_target(r), 0, 0, 0, z,
                           ms, GL_TEXTURE_2D_MULTISAMPLE, 0, 0, 0, 0, w, h, 1);
        for (s = 0; s < ns; s++) {
            glBindTexture(GL_TEXTURE_2D_MULTISAMPLE, ms);
            if (!sp_gl_prog_ms(SP_P_FETCH_MS_I, &sp_res_2d, s, 0)) {
                why = "no program";
                break;
            }
            sp_gl_draw(w, h, 0, h);
            sp_gl_read_parts(parts, r, SP_K_MS_COLOR, 0, z, s, w, h, in->fmt,
                             in->type, in->bpp);
        }
        why = why ? why : sp_gl_error("the copy of samples");
    }
    sp_gl_detach_all();
    glBindTexture(GL_TEXTURE_2D_MULTISAMPLE, 0);
    glDeleteTextures(1, &ms);
    glDeleteTextures(1, &st);
    return why;
}

static const char *sp_import_ms_snorm(const SpRes *r, GLuint tex, GLint ns,
                                      GLint fixed, const SpFmt *f,
                                      const SpXfer *x, GByteArray *b)
{
    const SpFmt *in = sp_fmt_of(f->twin);
    uint32_t w = MAX(r->width, 1), h = MAX(r->height, 1);
    uint32_t y = x->box[1], n = x->box[4], z = x->box[2];
    const char *why;
    GLuint ms, st;

    if (b->len < (uint64_t)w * n * in->bpp) {
        return "too few bytes";
    }
    ms = sp_gl_ms_twin(f, ns, fixed, w, h);
    st = sp_gl_texture(in->ifmt, in->fmt, in->type, w, h);
    glTexSubImage2D(GL_TEXTURE_2D, 0, 0, y, w, n, in->fmt, in->type, b->data);
    /* the layer as it is, this sample's rows drawn into it, and back */
    glCopyImageSubData(tex, sp_gl_target(r), 0, 0, 0, z,
                       ms, GL_TEXTURE_2D_MULTISAMPLE, 0, 0, 0, 0, w, h, 1);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0,
                           GL_TEXTURE_2D_MULTISAMPLE, ms, 0);
    glDrawBuffer(GL_COLOR_ATTACHMENT0);
    if (!(why = sp_gl_complete())) {
        if (!sp_gl_prog(SP_P_COPY_I)) {
            why = "no program";
        } else {
            glEnable(GL_SAMPLE_MASK);
            glSampleMaski(0, 1u << x->sample);
            sp_gl_draw(w, h, y, n);
            glDisable(GL_SAMPLE_MASK);
        }
    }
    sp_gl_detach_all();
    if (!why) {
        glCopyImageSubData(ms, GL_TEXTURE_2D_MULTISAMPLE, 0, 0, 0, 0,
                           tex, sp_gl_target(r), 0, 0, 0, z, w, h, 1);
    }
    glBindTexture(GL_TEXTURE_2D, 0);
    glDeleteTextures(1, &ms);
    glDeleteTextures(1, &st);
    return why ? why : sp_gl_error("the copy of samples");
}

/* kinds 2, 3 and 4 of a multisampled texture, each layer of an array */
static const char *sp_export_ms(const SpRes *r, GLuint tex, SpParts *parts)
{
    GLint ns = 0, ifmt = 0, fixed = 0, swz[4];
    uint32_t w = MAX(r->width, 1), h = MAX(r->height, 1), z;
    uint32_t layers = sp_ms_layers(r);
    GLenum t = sp_gl_target(r);
    const SpFmt *f = NULL;
    const char *why;
    GLuint st = 0, ms8 = 0;
    GLint s;
    int mode = SP_DEPTH_NO;
    bool has_swz;

    if ((why = sp_gl_ms_info(r, tex, &ns, &ifmt, &fixed))) {
        return why;
    }
    if (!sp_is_depth(r->format)) {
        f = sp_fmt_of(ifmt);
        if (!f) {
            return "an internal format this sink does not copy";
        }
        if (f->cls == 's') {
            return sp_export_ms_snorm(r, tex, ns, fixed, f, parts);
        }
    }
    glBindTexture(t, tex);
    /* texelFetch() applies the texture's swizzle (virglrenderer sets one for
     * some formats); Apple's GL 4.1 has none on multisampled textures */
    glGetTexParameteriv(t, GL_TEXTURE_SWIZZLE_RGBA, swz);
    has_swz = glGetError() == GL_NO_ERROR;
    if (has_swz) {
        glTexParameteriv(t, GL_TEXTURE_SWIZZLE_RGBA,
                         (const GLint[]){ GL_RED, GL_GREEN, GL_BLUE, GL_ALPHA });
    }
    if (f) {
        unsigned p = f->cls == 'i' ? SP_P_FETCH_MS_I :
                     f->cls == 'u' ? SP_P_FETCH_MS_U : SP_P_FETCH_MS_F;

        st = sp_gl_texture(f->ifmt, f->fmt, f->type, w, h);
        glBindTexture(GL_TEXTURE_2D, 0);
        sp_gl_attach(GL_COLOR_ATTACHMENT0, &sp_res_2d, st, 0, 0);
        glDrawBuffer(GL_COLOR_ATTACHMENT0);
        glReadBuffer(GL_COLOR_ATTACHMENT0);
        if ((why = sp_gl_complete())) {
            goto out;
        }
        for (z = 0; z < layers; z++) {
            for (s = 0; s < ns; s++) {
                glBindTexture(t, tex);
                if (!sp_gl_prog_ms(p, r, s, z)) {
                    why = "no program";
                    goto out;
                }
                sp_gl_draw(w, h, 0, h);
                sp_gl_read_parts(parts, r, SP_K_MS_COLOR, 0, z, s, w, h, f->fmt,
                                 f->type, f->bpp);
            }
        }
        goto out;
    }
    if (sp_has_depth(r->format)) {
        double scale = sp_depth_scale(r->format);

        if (scale < 0) {
            why = "a depth format this sink does not keep exactly";
            goto out;
        }
        mode = sp_depth_selfcheck(r->format, ifmt, &why);
        if (mode == SP_DEPTH_NO && r->format == 17) {
            sp_depth_rounded(r->format, true, why);
            mode = SP_DEPTH_FETCH;
        }
        if (mode == SP_DEPTH_NO) {
            goto out;
        }
        why = NULL;
        for (z = 0; mode == SP_DEPTH_SEARCH && z < layers && !why; z++) {
            for (s = 0; s < ns && !why; s++) {
                why = sp_depth_search(r, tex, ns, fixed, s, z, parts);
            }
        }
        if (why) {
            goto out;
        }
    }
    if (sp_has_depth(r->format) && mode == SP_DEPTH_FETCH) {
        double scale = sp_depth_scale(r->format);

        st = sp_gl_texture(GL_R32UI, GL_RED_INTEGER, GL_UNSIGNED_INT, w, h);
        glBindTexture(GL_TEXTURE_2D, 0);
        sp_gl_attach(GL_COLOR_ATTACHMENT0, &sp_res_2d, st, 0, 0);
        glDrawBuffer(GL_COLOR_ATTACHMENT0);
        glReadBuffer(GL_COLOR_ATTACHMENT0);
        if ((why = sp_gl_complete())) {
            goto out;
        }
        for (z = 0; z < layers; z++) {
            for (s = 0; s < ns; s++) {
                unsigned first = parts->len, k;

                glBindTexture(t, tex);
                if (!sp_gl_prog_ms(SP_P_DEPTH_OUT, r, s, z)) {
                    why = "no program";
                    goto out;
                }
                sp_gl_draw(w, h, 0, h);
                sp_gl_read_parts(parts, r, SP_K_MS_DEPTH, 0, z, s, w, h,
                                 GL_RED_INTEGER, GL_UNSIGNED_INT, 4);
                for (k = first; k < parts->len; k++) {
                    GByteArray *p = parts->pdata[k];
                    uint32_t *v = (uint32_t *)(p->data + sizeof(SpXfer));
                    size_t i, nv = (p->len - sizeof(SpXfer)) / 4;

                    for (i = 0; i < nv; i++) {
                        v[i] = sp_depth_of_float(v[i], scale);
                    }
                }
            }
        }
        sp_gl_detach_all();
    }
    if (sp_has_stencil(r->format)) {
        GLint loc;
        unsigned bit;

        /* the stencil, bit by bit, into an R8 texture of the same samples;
         * then each sample of that, as kind 2 does */
        glGenTextures(1, &ms8);
        glBindTexture(GL_TEXTURE_2D_MULTISAMPLE, ms8);
        glTexImage2DMultisample(GL_TEXTURE_2D_MULTISAMPLE, ns, GL_R8, w, h, fixed);
        if (st) {
            glDeleteTextures(1, &st);
        }
        st = sp_gl_texture(GL_R8, GL_RED, GL_UNSIGNED_BYTE, w, h);
        glBindTexture(GL_TEXTURE_2D, 0);
        for (z = 0; z < layers; z++) {
            glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0,
                                   GL_TEXTURE_2D_MULTISAMPLE, ms8, 0);
            sp_gl_attach(GL_STENCIL_ATTACHMENT, r, tex, 0, z);
            glDrawBuffer(GL_COLOR_ATTACHMENT0);
            if ((why = sp_gl_complete()) || !sp_gl_prog(SP_P_CONST)) {
                why = why ? why : "no program";
                goto out;
            }
            loc = glGetUniformLocation(sp_prog[SP_P_CONST], "v");
            glClearBufferfv(GL_COLOR, 0, (const GLfloat[4]){ 0 });
            glEnable(GL_STENCIL_TEST);
            glStencilMask(0);
            glStencilOp(GL_KEEP, GL_KEEP, GL_KEEP);
            glEnable(GL_BLEND);
            glBlendFunc(GL_ONE, GL_ONE);
            glBlendEquation(GL_FUNC_ADD);
            for (bit = 0; bit < 8; bit++) {
                glStencilFunc(GL_EQUAL, 1u << bit, 1u << bit);
                glUniform1f(loc, (1u << bit) / 255.0f);
                sp_gl_draw(w, h, 0, h);
            }
            glDisable(GL_BLEND);
            glDisable(GL_STENCIL_TEST);
            glStencilMask(0xff);
            sp_gl_detach_all();
            sp_gl_attach(GL_COLOR_ATTACHMENT0, &sp_res_2d, st, 0, 0);
            glDrawBuffer(GL_COLOR_ATTACHMENT0);
            glReadBuffer(GL_COLOR_ATTACHMENT0);
            if ((why = sp_gl_complete())) {
                goto out;
            }
            for (s = 0; s < ns; s++) {
                glBindTexture(GL_TEXTURE_2D_MULTISAMPLE, ms8);
                if (!sp_gl_prog_ms(SP_P_FETCH_MS_F, &sp_res_2d, s, 0)) {
                    why = "no program";
                    goto out;
                }
                sp_gl_draw(w, h, 0, h);
                sp_gl_read_parts(parts, r, SP_K_STENCIL, 0, z, s, w, h, GL_RED,
                                 GL_UNSIGNED_BYTE, 1);
            }
            sp_gl_detach_all();
        }
    }
  out:
    sp_gl_detach_all();
    glBindTexture(t, tex);
    if (has_swz) {
        glTexParameteriv(t, GL_TEXTURE_SWIZZLE_RGBA, swz);
    }
    glBindTexture(t, 0);
    glBindTexture(GL_TEXTURE_2D_MULTISAMPLE, 0);
    glBindTexture(GL_TEXTURE_2D, 0);
    if (st) {
        glDeleteTextures(1, &st);
    }
    if (ms8) {
        glDeleteTextures(1, &ms8);
    }
    return why ? why : sp_gl_error("the read of samples");
}

/*
 * kinds 2, 3 and 4 back (and kind 4 of a single-sample stencil-only texture):
 * draw rows of a staging texture into the target's level and layer
 */
static const char *sp_import_ms(const SpRes *r, GLuint tex, const SpXfer *x,
                                GByteArray *b)
{
    GLint ns = 0, ifmt = 0, fixed = 0;
    uint32_t w, h, layers;
    uint32_t y = x->box[1], n = x->box[4];
    const SpFmt *f = NULL;
    const char *why = NULL;
    GLuint st = 0;
    unsigned bpp, p;

    if (r->nr_samples > 1) {
        if ((why = sp_gl_ms_info(r, tex, &ns, &ifmt, &fixed))) {
            return why;
        }
        if (x->sample >= (uint32_t)ns) {
            return "no such sample here";
        }
        w = MAX(r->width, 1);
        h = MAX(r->height, 1);
        layers = x->level ? 0 : sp_ms_layers(r);
    } else {
        sp_level_dims(r, x->level, &w, &h, &layers);
    }
    if (x->box[3] != w || y + n > h || x->box[0] || x->box[2] >= layers ||
        x->level > r->last_level) {
        return "a box this sink did not export";
    }
    switch (x->kind) {
    case SP_K_MS_COLOR:
        f = sp_fmt_of(ifmt);
        if (!f) {
            return "an internal format this sink does not copy";
        }
        if (f->cls == 's' && sp_gl_copy_image()) {
            return sp_import_ms_snorm(r, tex, ns, fixed, f, x, b);
        }
        bpp = f->bpp;
        p = f->cls == 'i' ? SP_P_COPY_I : f->cls == 'u' ? SP_P_COPY_U : SP_P_COPY_F;
        break;
    case SP_K_MS_DEPTH:
        if (!sp_has_depth(r->format) || sp_depth_scale(r->format) < 0) {
            return "not a depth format this sink keeps";
        }
        if (sp_depth_selfcheck(r->format, ifmt, &why) == SP_DEPTH_NO) {
            if (r->format != 17) {
                return why;
            }
            sp_depth_rounded(r->format, true, why);
        }
        why = NULL;
        f = &sp_fmt_r32ui;
        bpp = 4;
        p = SP_P_DEPTH_IN;
        break;
    default:
        if (!sp_has_stencil(r->format)) {
            return "not a format with stencil";
        }
        f = &sp_fmt_r8ui;
        bpp = 1;
        p = SP_P_STENCIL_IN;
        break;
    }
    if (b->len < (uint64_t)w * n * bpp) {
        return "too few bytes";
    }
    if (f->cls == 's') {
        /* snorm through floats: the bytes as c / MAX, the lowest as -1.0 */
        g_autofree float *v = g_new0(float, (size_t)w * n * 4);
        double mx;
        unsigned comps = sp_snorm_comps(f, &mx), c;
        size_t i;

        for (i = 0; i < (size_t)w * n; i++) {
            for (c = 0; c < comps; c++) {
                double e;

                if (f->type == GL_BYTE) {
                    e = (int8_t)b->data[i * comps + c];
                } else {
                    int16_t v16;

                    memcpy(&v16, b->data + (i * comps + c) * 2, 2);
                    e = v16;
                }
                v[i * 4 + c] = (float)MAX(e / mx, -1.0);
            }
        }
        st = sp_gl_texture(GL_RGBA32F, GL_RGBA, GL_FLOAT, w, h);
        glTexSubImage2D(GL_TEXTURE_2D, 0, 0, y, w, n, GL_RGBA, GL_FLOAT, v);
    } else if (x->kind == SP_K_MS_DEPTH) {
        g_autofree uint32_t *v = g_memdup2(b->data, (size_t)w * n * 4);
        double scale = sp_depth_scale(r->format);
        size_t i;

        for (i = 0; i < (size_t)w * n; i++) {
            v[i] = sp_depth_to_float(v[i], scale);
        }
        st = sp_gl_texture(f->ifmt, f->fmt, f->type, w, h);
        glTexSubImage2D(GL_TEXTURE_2D, 0, 0, y, w, n, f->fmt, f->type, v);
    } else {
        st = sp_gl_texture(f->ifmt, f->fmt, f->type, w, h);
        glTexSubImage2D(GL_TEXTURE_2D, 0, 0, y, w, n, f->fmt, f->type, b->data);
    }
    if (!sp_gl_prog(p)) {
        why = "no program";
        goto out;
    }
    if (r->nr_samples > 1) {
        glEnable(GL_SAMPLE_MASK);
        glSampleMaski(0, 1u << x->sample);
    }
    switch (x->kind) {
    case SP_K_MS_COLOR:
        sp_gl_attach(GL_COLOR_ATTACHMENT0, r, tex, 0, x->box[2]);
        glDrawBuffer(GL_COLOR_ATTACHMENT0);
        if (!(why = sp_gl_complete())) {
            sp_gl_draw(w, h, y, n);
        }
        break;
    case SP_K_MS_DEPTH:
        sp_gl_attach(GL_DEPTH_ATTACHMENT, r, tex, 0, x->box[2]);
        glDrawBuffer(GL_NONE);
        if (!(why = sp_gl_complete())) {
            glEnable(GL_DEPTH_TEST);
            glDepthFunc(GL_ALWAYS);
            glDepthMask(GL_TRUE);
            sp_gl_draw(w, h, y, n);
            glDisable(GL_DEPTH_TEST);
        }
        break;
    default:
        sp_gl_attach(GL_STENCIL_ATTACHMENT, r, tex, x->level, x->box[2]);
        glDrawBuffer(GL_NONE);
        if (!(why = sp_gl_complete())) {
            sp_gl_stencil_passes(w, h, y, n);
        }
        break;
    }
    glDisable(GL_SAMPLE_MASK);
  out:
    sp_gl_detach_all();
    glBindTexture(GL_TEXTURE_2D, 0);
    glDeleteTextures(1, &st);
    return why ? why : sp_gl_error("the write of samples");
}

/* ---- the test command "clear" -------------------------------------------- */

/*
 * Every resource the GPU can render to, every level, layer and sample, to
 * zero, behind the source's back (the check S2: what a redraw brings back);
 * then the scanout is shown again.
 */
static void sp_state_clear(VirtIOGPU *g)
{
    GHashTableIter it;
    gpointer value;
    unsigned done = 0, failed = 0, i;

    sp_state_settle();
    if (!sp_gl_begin(g)) {
        fprintf(stderr, "[SPACEBOX-REMOTE] sink clear: not done (no GL context)\n");
        return;
    }
    sp_gl_defaults();
    g_hash_table_iter_init(&it, sps.res);
    while (g_hash_table_iter_next(&it, NULL, &value)) {
        SpRes *r = value;
        struct virgl_renderer_resource_info info = { 0 };
        GLenum att = sp_gl_attachment(r->format);
        uint32_t l;
        bool ok = true;

        if (r->target == SP_T_BUFFER ||
            !(r->bind & (SP_BIND_RENDER_TARGET | SP_BIND_DEPTH_STENCIL)) ||
            virgl_renderer_resource_get_info(r->id, &info) || !info.tex_id) {
            continue;
        }
        for (l = 0; l <= r->last_level && l < 32; l++) {
            static const GLfloat zf[4];
            static const GLint zi[4];
            static const GLuint zu[4];
            GLint type = GL_FLOAT;

            /* a layered attachment: a clear reaches every layer */
            glFramebufferTexture(GL_FRAMEBUFFER, att, info.tex_id, l);
            glDrawBuffer(att == GL_COLOR_ATTACHMENT0 ? GL_COLOR_ATTACHMENT0 : GL_NONE);
            glReadBuffer(GL_NONE);
            if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
                ok = false;
            } else if (att == GL_COLOR_ATTACHMENT0) {
                glGetFramebufferAttachmentParameteriv(GL_FRAMEBUFFER, att,
                    GL_FRAMEBUFFER_ATTACHMENT_COMPONENT_TYPE, &type);
                if (type == GL_INT) {
                    glClearBufferiv(GL_COLOR, 0, zi);
                } else if (type == GL_UNSIGNED_INT) {
                    glClearBufferuiv(GL_COLOR, 0, zu);
                } else {
                    glClearBufferfv(GL_COLOR, 0, zf);
                }
            } else if (att == GL_DEPTH_STENCIL_ATTACHMENT) {
                glClearBufferfi(GL_DEPTH_STENCIL, 0, 0.0f, 0);
            } else if (att == GL_DEPTH_ATTACHMENT) {
                glClearBufferfv(GL_DEPTH, 0, zf);
            } else {
                glClearBufferiv(GL_STENCIL, 0, zi);
            }
            glFramebufferTexture(GL_FRAMEBUFFER, att, 0, 0);
        }
        if (ok && glGetError() == GL_NO_ERROR) {
            done++;
        } else {
            failed++;
            fprintf(stderr, "[SPACEBOX-REMOTE] sink clear: resource %u (format %u, "
                    "target %u) not cleared\n", r->id, r->format, r->target);
        }
    }
    sp_gl_end();
    for (i = 0; i < g->parent_obj.conf.max_outputs; i++) {
        struct virtio_gpu_scanout *s = &g->parent_obj.scanout[i];

        if (s->con && s->resource_id) {
            dpy_gl_update(s->con, 0, 0, qemu_console_get_width(s->con, 0),
                          qemu_console_get_height(s->con, 0));
        }
    }
    fprintf(stderr, "[SPACEBOX-REMOTE] sink clear: %u resources set to zero, %u not; "
            "the scanout shown again\n", done, failed);
}

/* ---- the test command "stats" -------------------------------------------- */

static gint sp_res_cmp(gconstpointer a, gconstpointer b)
{
    const SpRes *x = *(SpRes *const *)a, *y = *(SpRes *const *)b;

    return x->id < y->id ? -1 : x->id > y->id;
}

static void sp_state_stats(void)
{
    g_autoptr(GPtrArray) all = g_ptr_array_new();
    GHashTableIter it;
    gpointer value;
    uint64_t sum = 0;
    unsigned i;

    g_hash_table_iter_init(&it, sps.res);
    while (g_hash_table_iter_next(&it, NULL, &value)) {
        g_ptr_array_add(all, value);
    }
    g_ptr_array_sort(all, sp_res_cmp);
    for (i = 0; i < all->len; i++) {
        SpRes *r = all->pdata[i];
        uint64_t size = sp_res_size(r);
        uint32_t w, h, layers;

        sp_level_dims(r, 0, &w, &h, &layers);
        sum += size;
        fprintf(stderr, "[SPACEBOX-REMOTE] sink resource id=%u target=%u format=%u "
                "size=%ux%ux%u levels=%u layers=%u samples=%u bind=0x%x "
                "created=%" PRIu64 " written=%" PRIu64 " bytes=%" PRIu64 "\n",
                r->id, r->target, r->format, r->width, r->height, r->depth,
                r->last_level + 1, layers, r->nr_samples, r->bind, r->created,
                r->written, size);
    }
    fprintf(stderr, "[SPACEBOX-REMOTE] sink resources=%u bytes=%" PRIu64
            " contexts=%u\n", all->len, sum, g_hash_table_size(sps.ctx));
}

/* ---- SP_STATE_RESET -------------------------------------------------------- */

static void sp_state_reset(VirtIOGPU *g, uint64_t n)
{
    VirtIOGPUClass *vgc = VIRTIO_GPU_GET_CLASS(g);
    struct virtio_gpu_simple_resource *res, *tmp;
    struct virtio_gpu_ctrl_command *cmd;
    g_autoptr(GArray) kept = g_array_new(false, false, sizeof(uint32_t));
    g_autoptr(GList) ctxs = g_hash_table_get_keys(sps.ctx);
    unsigned dropped = 0, nctx = 0, nfence = 0, i;
    int64_t t0 = g_get_monotonic_time();
    GList *l;

    /* contexts first: their sub-contexts, objects and attachments go with them */
    for (l = ctxs; l; l = l->next) {
        virgl_renderer_context_destroy(GPOINTER_TO_UINT(l->data));
        nctx++;
    }
    g_hash_table_remove_all(sps.ctx);
    /* fences of commands that ran: their answers are not wanted any more */
    while ((cmd = QTAILQ_FIRST(&g->fenceq))) {
        QTAILQ_REMOVE(&g->fenceq, cmd, next);
        g->inflight--;
        g_free(cmd->sp_snap);
        g_free(cmd);
        nfence++;
    }
    virtio_gpu_virgl_reset_scanout(g);
    for (i = 0; i < g->parent_obj.conf.max_outputs; i++) {
        struct virtio_gpu_scanout *s = &g->parent_obj.scanout[i];

        if (s->con) {
            dpy_mouse_set(s->con, s->cursor.pos.x, s->cursor.pos.y, false);
        }
        s->resource_id = 0;
        s->width = s->height = 0;
        s->x = s->y = 0;
        memset(&s->cursor, 0, sizeof(s->cursor));
        if (s->current_cursor) {
            cursor_unref(s->current_cursor);
            s->current_cursor = NULL;
        }
    }
    g->parent_obj.enable = 0;
    QTAILQ_FOREACH_SAFE(res, &g->reslist, next, tmp) {
        uint32_t id = res->resource_id;
        SpRes *r = g_hash_table_lookup(sps.res, GUINT_TO_POINTER(id));
        Error *err = NULL;

        if (n && r && r->created <= n && r->written <= n) {
            g_array_append_val(kept, id);
            continue;
        }
        g_hash_table_remove(sps.res, GUINT_TO_POINTER(id));
        vgc->resource_destroy(g, res, &err);
        if (err) {
            error_report_err(err);
        }
        dropped++;
    }
    sp_sink_send(SP_STATE_KEPT, kept->data, kept->len * sizeof(uint32_t),
                 NULL, 0, n, 0, 0);
    fprintf(stderr, "[SPACEBOX-REMOTE] sink state reset to message %" PRIu64
            ": kept %u resources, dropped %u, destroyed %u contexts, dropped %u "
            "fences, %.1f ms\n", n, kept->len, dropped, nctx, nfence,
            (g_get_monotonic_time() - t0) / 1000.0);
}

/* ---- SP_IMPORT -------------------------------------------------------------- */

static void sp_imports_log(void)
{
    if (!sps.imp.n) {
        return;
    }
    fprintf(stderr, "[SPACEBOX-REMOTE] sink imports: %" PRIu64 " messages, %"
            PRIu64 " bytes, %" PRIu64 " failed, over %.1f ms\n", sps.imp.n,
            sps.imp.bytes, sps.imp.failed,
            (sps.imp.last - sps.imp.t0) / 1000.0);
    memset(&sps.imp, 0, sizeof(sps.imp));
}

/* Any other message from the source ends a burst of imports (for the log). */
void sp_state_ctrl_seen(void)
{
    sp_imports_log();
}

static const char *sp_import_kind0(SpRes *r, const SpXfer *x, GByteArray *b)
{
    struct iovec iov = { .iov_base = b->data, .iov_len = b->len };
    struct virtio_gpu_box box = { x->box[0], x->box[1], x->box[2],
                                  x->box[3], x->box[4], x->box[5] };

    sp_ctx0_clean();
    sps.ctx0_dirty = true;
    if (virgl_renderer_transfer_write_iov(x->res, 0, x->level, x->stride,
                                          x->layer_stride,
                                          (struct virgl_box *)&box, 0,
                                          &iov, 1)) {
        return "the renderer refused the write";
    }
    return NULL;
}

static void sp_state_import(VirtIOGPU *g, SpOp *op)
{
    struct virgl_renderer_resource_info info = { 0 };
    SpXfer x;
    SpRes *r;
    const char *why;

    if (!sps.imp.n) {
        sps.imp.t0 = g_get_monotonic_time();
    }
    sps.imp.n++;
    sps.imp.bytes += op->b ? op->b->len : 0;
    sps.imp.last = g_get_monotonic_time();
    if (!op->a || op->a->len < sizeof(x)) {
        why = "short header";
        memset(&x, 0, sizeof(x));
        goto fail;
    }
    memcpy(&x, op->a->data, sizeof(x));
    r = g_hash_table_lookup(sps.res, GUINT_TO_POINTER(x.res));
    if (!r || !virtio_gpu_find_resource(g, x.res)) {
        why = "no such resource";
        goto fail;
    }
    if (x.kind == SP_K_GUEST) {
        why = sp_import_kind0(r, &x, op->b);
    } else if (x.kind > SP_K_STENCIL) {
        why = "unknown kind";
    } else if (virgl_renderer_resource_get_info(x.res, &info) || !info.tex_id) {
        why = "no texture in the renderer";
    } else if (!op->b) {
        why = "no bytes";
    } else if (!sp_gl_begin(g)) {
        why = "no GL context of the sink's own";
    } else {
        sp_gl_defaults();
        why = x.kind == SP_K_DEPTH ? sp_import_kind1(r, info.tex_id, &x, op->b) :
              sp_import_ms(r, info.tex_id, &x, op->b);
        sp_gl_end();
    }
    if (!why) {
        return;
    }
  fail:
    sps.imp.failed++;
    fprintf(stderr, "[SPACEBOX-REMOTE] sink import into resource %u (kind %u, "
            "level %u, sample %u, box %u,%u,%u %ux%ux%u) failed: %s\n",
            x.res, x.kind, x.level, x.sample, x.box[0], x.box[1], x.box[2],
            x.box[3], x.box[4], x.box[5], why);
}

/* ---- SP_EXPORT ------------------------------------------------------------- */

static const char *sp_export_kind0(SpRes *r, SpParts *parts)
{
    g_autofree uint8_t *buf = NULL;
    uint64_t row, layer;
    uint32_t level, rows, w, h, layers, z, y, bh;

    if (r->target == SP_T_BUFFER) {
        uint64_t off;

        for (off = 0; off < r->width; off += SP_XFER_MAX) {
            uint32_t len = MIN(r->width - off, SP_XFER_MAX);
            SpXfer x = { .res = r->id, .box = { off, 0, 0, len, 1, 1 } };
            struct virtio_gpu_box box = { off, 0, 0, len, 1, 1 };
            struct iovec iov;
            uint8_t *data;

            g_ptr_array_add(parts, sp_part_new(&x, len, &data));
            iov = (struct iovec){ data, len };
            sp_ctx0_clean();
            if (virgl_renderer_transfer_read_iov(r->id, 0, 0, 0, 0,
                                                 (struct virgl_box *)&box, 0,
                                                 &iov, 1)) {
                return "the renderer refused the read";
            }
        }
        return NULL;
    }
    if (r->format >= SP_FORMAT_COUNT || !sp_format_blocks[r->format].bits) {
        return "a format of unknown layout";
    }
    bh = sp_format_blocks[r->format].bh;
    for (level = 0; level <= r->last_level && level < 32; level++) {
        layer = sp_layer_size(r, level, &row, &rows);
        sp_level_dims(r, level, &w, &h, &layers);
        if (!layer) {
            return "a format of unknown layout";
        }
        /*
         * Formats the renderer cannot draw to are read with glGetTexImage,
         * which wants room for the whole level (all layers) even for a box.
         */
        g_free(buf);
        buf = g_try_malloc(layer * layers);
        if (!buf) {
            return "out of memory";
        }
        for (z = 0; z < layers; z++) {
            uint32_t step = MAX(SP_XFER_MAX / row, 1);

            for (y = 0; y < rows; y += step) {
                uint32_t n = MIN(step, rows - y);
                uint32_t py = y * bh, ph = MIN(n * bh, h - py);
                SpXfer x = { .res = r->id, .level = level,
                             .box = { 0, py, z, w, ph, 1 } };
                struct virtio_gpu_box box = { 0, py, z, w, ph, 1 };
                struct iovec iov = { buf, layer * layers };
                uint8_t *data;

                sp_ctx0_clean();
                if (virgl_renderer_transfer_read_iov(r->id, 0, level, 0, 0,
                                                     (struct virgl_box *)&box,
                                                     0, &iov, 1)) {
                    return "the renderer refused the read";
                }
                g_ptr_array_add(parts, sp_part_new(&x, n * row, &data));
                memcpy(data, buf, n * row);
            }
        }
    }
    return NULL;
}

static const char *sp_export_res(VirtIOGPU *g, SpRes *r, SpParts *parts)
{
    struct virgl_renderer_resource_info info = { 0 };
    const char *why;

    if (r->target == SP_T_BUFFER ||
        (r->nr_samples <= 1 && !sp_is_depth(r->format))) {
        return sp_export_kind0(r, parts);
    }
    if (virgl_renderer_resource_get_info(r->id, &info) || !info.tex_id) {
        return "no texture in the renderer";
    }
    if (!sp_gl_begin(g)) {
        return "no GL context of the sink's own";
    }
    sp_gl_defaults();
    if (r->nr_samples > 1) {
        why = sp_export_ms(r, info.tex_id, parts);
    } else if (sp_has_depth(r->format)) {
        why = sp_export_depth(r, info.tex_id, parts);
    } else {
        why = sp_export_stencil1(r, info.tex_id, parts);
    }
    sp_gl_end();
    return why;
}

static void sp_state_export(VirtIOGPU *g, SpOp *op)
{
    g_autoptr(GArray) skipped = g_array_new(false, false, sizeof(uint32_t));
    uint64_t msgs = 0, bytes = 0;
    unsigned asked = 0, done = 0, i;
    int64_t t0 = g_get_monotonic_time();
    uint64_t req = op->arg0;
    size_t k;

    if (sps.export_stall_ms) {
        fprintf(stderr, "[SPACEBOX-REMOTE] sink export %#" PRIx64 ": stalled "
                "%u ms (control)\n", req, sps.export_stall_ms);
        g_usleep(sps.export_stall_ms * 1000ull);
        sps.export_stall_ms = 0;
        t0 = g_get_monotonic_time();
    }
    sp_state_settle();
    sps.ctx0_stale = 0;
    for (k = 0; op->a && k + 4 <= op->a->len; k += 4) {
        uint32_t id;
        SpRes *r;
        const char *why;
        g_autoptr(GPtrArray) parts =
            g_ptr_array_new_with_free_func((GDestroyNotify)g_byte_array_unref);

        memcpy(&id, op->a->data + k, 4);
        asked++;
        r = g_hash_table_lookup(sps.res, GUINT_TO_POINTER(id));
        if (!r || !virtio_gpu_find_resource(g, id)) {
            why = "no such resource";
        } else {
            virgl_renderer_force_ctx_0();
            why = sp_export_res(g, r, parts);
        }
        if (why) {
            g_array_append_val(skipped, id);
            fprintf(stderr, "[SPACEBOX-REMOTE] sink export: resource %u not "
                    "exported: %s\n", id, why);
            continue;
        }
        for (i = 0; i < parts->len; i++) {
            GByteArray *p = parts->pdata[i];

            sp_sink_send(SP_EXPORT_DATA, p->data, sizeof(SpXfer),
                         p->data + sizeof(SpXfer), p->len - sizeof(SpXfer),
                         0, 0, req);
            msgs++;
            bytes += p->len - sizeof(SpXfer);
        }
        done++;
    }
    sp_sink_send(SP_EXPORT_DONE, skipped->data, skipped->len * sizeof(uint32_t),
                 NULL, 0, msgs, bytes, req);
    fprintf(stderr, "[SPACEBOX-REMOTE] sink export: %u resources asked, %u "
            "exported in %" PRIu64 " messages, %" PRIu64 " bytes, %u not "
            "exported, %.1f ms; %u stale GL errors of context 0 cleared; "
            "request %#" PRIx64 "\n",
            asked, done, msgs, bytes, skipped->len,
            (g_get_monotonic_time() - t0) / 1000.0, sps.ctx0_stale, req);
}

/* ---- in order with the commands ------------------------------------------- */

void sp_state_queue(VirtIOGPU *g, uint32_t type, uint64_t id, uint64_t arg0,
                    const void *a, size_t a_len, const void *b, size_t b_len)
{
    struct virtio_gpu_ctrl_command *cmd = g_new0(struct virtio_gpu_ctrl_command, 1);
    SpOp *op = g_new0(SpOp, 1);

    if (type != SP_IMPORT) {
        sp_imports_log();
    }
    op->type = type;
    op->id = id;
    op->arg0 = arg0;
    op->a = g_byte_array_sized_new(a_len);
    g_byte_array_append(op->a, a, a_len);
    if (b_len) {
        op->b = g_byte_array_sized_new(b_len);
        g_byte_array_append(op->b, b, b_len);
    }
    cmd->sp_remote = true;
    cmd->sp_op = op;
    QTAILQ_INSERT_TAIL(&g->cmdq, cmd, next);
    virtio_gpu_process_cmdq(g);
}

bool sp_sink_before_cmd(VirtIOGPU *g, struct virtio_gpu_ctrl_command *cmd)
{
    GByteArray *wm = cmd->sp_wmark;
    SpOp *op = cmd->sp_op;

    cmd->sp_wmark = NULL;
    if (sps.ctx0_dirty && (!op || op->type != SP_IMPORT)) {
        /* the imports complete before what comes after them reads them */
        virgl_renderer_force_ctx_0();
        glFinish();
        sps.ctx0_dirty = false;
    }
    if (!op) {
        sp_state_note(g, cmd);
        sp_state_written(wm);
        if (wm) {
            g_byte_array_unref(wm);
        }
        return false;
    }
    cmd->sp_op = NULL;
    switch (op->type) {
    case SP_STATE_RESET:
        sp_state_reset(g, op->arg0);
        break;
    case SP_IMPORT:
        sp_state_import(g, op);
        break;
    case SP_EXPORT:
        sp_state_export(g, op);
        break;
    }
    g_byte_array_unref(op->a);
    if (op->b) {
        g_byte_array_unref(op->b);
    }
    g_free(op);
    cmd->finished = true;
    return true;
}

/* ---- the control file ------------------------------------------------------ */

static void sp_control_tick(void *opaque)
{
    VirtIOGPU *g = sps.g;
    g_autofree char *text = NULL;

    if (sps.imp.n && g_get_monotonic_time() - sps.imp.last > 200000) {
        sp_imports_log();
    }
    timer_mod(sps.control_timer, qemu_clock_get_ms(QEMU_CLOCK_REALTIME) + 100);
    if (!sps.control || !g_file_get_contents(sps.control, &text, NULL, NULL)) {
        return;
    }
    unlink(sps.control);
    g_strstrip(text);
    if (strchr(text, '\n')) {
        *strchr(text, '\n') = 0;
    }
    fprintf(stderr, "[SPACEBOX-REMOTE] sink control: %s\n", text);
    if (!strcmp(text, "detach")) {
        sp_sink_detach();
    } else if (!strcmp(text, "clear")) {
        if (VIRTIO_GPU_GL(g)->renderer_state == RS_INITED) {
            sp_state_clear(g);
        }
    } else if (!strcmp(text, "stats")) {
        sp_state_stats();
    } else if (!strcmp(text, "gpu off") || !strcmp(text, "gpu on")) {
        /* the user's toggle, as the hotkey (virtio-gpu-remote.c) */
        sp_sink_gpu(!strcmp(text, "gpu on"));
    } else if (!strncmp(text, "stall-export ", 13)) {
        sps.export_stall_ms = MIN(strtoul(text + 13, NULL, 10), 60000);
        fprintf(stderr, "[SPACEBOX-REMOTE] sink control: the next export "
                "stalls %u ms\n", sps.export_stall_ms);
    } else {
        fprintf(stderr, "[SPACEBOX-REMOTE] sink control: unknown command\n");
    }
}

void sp_state_realize(VirtIOGPU *g)
{
    const char *p = getenv("SPACEBOX_GPU_REMOTE_CONTROL");

    sps.g = g;
    sps.res = g_hash_table_new_full(g_direct_hash, g_direct_equal, NULL, g_free);
    sps.ctx = g_hash_table_new(g_direct_hash, g_direct_equal);
    sps.control_timer = timer_new_ms(QEMU_CLOCK_REALTIME, sp_control_tick, NULL);
    if (p && *p) {
        sps.control = g_strdup(p);
        unlink(p);
    }
    timer_mod(sps.control_timer, qemu_clock_get_ms(QEMU_CLOCK_REALTIME) + 100);
}
