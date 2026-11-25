/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Stub <linux/ipc_logging.h> for 6.x port.
 *
 * Downstream Qualcomm trees ship a custom IPC logging facility used by
 * IPA / SPS to maintain in-kernel ring-buffered debug logs accessible via
 * debugfs. Mainline never had this header / facility.
 *
 * Both ipa_common_i.h and spsi.h do an unconditional `#include
 * <linux/ipc_logging.h>` at the top, so we ship this stub at the same path
 * to satisfy the include. All actual logging continues via pr_debug /
 * pr_err and trace_printk.
 *
 * NOTE: If your kernel tree already has <linux/ipc_logging.h>, do NOT
 * install this stub — let the existing one win.
 */

#ifndef _LINUX_IPC_LOGGING_H
#define _LINUX_IPC_LOGGING_H

#include <linux/types.h>
#include <linux/printk.h>

/*
 * The vendor source uses these three functions:
 *
 *   ipc_log_context_create(num_pages, modname, user_version)
 *   ipc_log_context_destroy(ctxt)
 *   ipc_log_string(ctxt, fmt, ...)
 *
 * We stub all three. ipc_log_context_create returns NULL so the calling
 * driver code follows its "no context, just continue" path, and the macros
 * in ipa_common_i.h / spsi.h that gate ipc_log_string on a non-NULL ctxt
 * become no-ops.
 */

static inline void *
ipc_log_context_create(int max_num_pages, const char *modname,
		       uint16_t user_version)
{
	(void)max_num_pages;
	(void)modname;
	(void)user_version;
	return NULL;
}

static inline int
ipc_log_context_destroy(void *ctxt)
{
	(void)ctxt;
	return 0;
}

/* Variadic. Silently swallow everything. */
#define ipc_log_string(ctxt, fmt, ...)	do { } while (0)

#endif /* _LINUX_IPC_LOGGING_H */
