/*
 * Spacebox: the sink's hook files (the input test, the capture request, the
 * scroll fixture, the window request, the control file). A writer puts the
 * whole request in place at once (a rename over the file); the reader takes
 * its text and removes the file.
 *
 * On Windows a rename over a file can be a removal and then a new file
 * (MoveFileEx with replace is one step; a writer that removes first, or
 * writes in place, is two), and a file that another process holds open
 * cannot be removed. There the text is taken only when it is complete (it
 * ends in a newline, or it has stayed the same for a second: an empty
 * request, or one without a newline) and only when the file could be
 * removed, so that a request is neither taken half written nor taken twice.
 * Elsewhere it is taken as it is.
 */

#ifndef UI_SPACEBOX_HOOK_H
#define UI_SPACEBOX_HOOK_H

/* true and the file's text (g_free it) when a request was taken */
static inline bool spacebox_hook_take(const char *path, char **text)
{
    gchar *t = NULL;
    gsize len = 0;

    *text = NULL;
    if (!g_file_get_contents(path, &t, &len, NULL)) {
        return false;
    }
#ifdef _WIN32
    {
        /* each hook's text seen incomplete, after the time it was first seen */
        static GHashTable *partial;
        int64_t now = g_get_monotonic_time();

        if (!partial) {
            partial = g_hash_table_new_full(g_str_hash, g_str_equal, g_free, g_free);
        }
        if (!len || t[len - 1] != '\n') {
            const char *last = g_hash_table_lookup(partial, path);

            if (!last || strcmp(last + sizeof(int64_t), t)) {
                char *seen = g_malloc(sizeof(int64_t) + len + 1);

                memcpy(seen, &now, sizeof(now));
                memcpy(seen + sizeof(int64_t), t, len + 1);
                g_hash_table_insert(partial, g_strdup(path), seen);
                g_free(t);
                return false;
            }
            if (now - *(const int64_t *)last < G_USEC_PER_SEC) {
                g_free(t);
                return false;
            }
        }
        g_hash_table_remove(partial, path);
        if (unlink(path) != 0) {
            g_free(t);
            return false;
        }
    }
#else
    unlink(path);
#endif
    *text = t;
    return true;
}

#endif
