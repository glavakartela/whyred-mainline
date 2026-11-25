/* SPDX-License-Identifier: GPL-2.0 */
/*
 * <linux/msm-bus-board.h> shim for the IPA v2 6.x port.
 *
 * Provides the MSM_BUS_MASTER_* / MSM_BUS_SLAVE_* IDs the vendor IPA v2
 * driver writes into the src/dst of its static msm_bus_vectors[] tables.
 * These IDs ARE read: msm_bus_compat.c (interconnect-backed, see msm-bus.h)
 * maps them in map_src_dst_to_icc_name() to a named DT interconnect path
 * ("ipa-mem" / "ipa-imem"). The numbers are arbitrary but must match the
 * case labels there; only IPA/BAM_DMA x EBI_CH0/OCIMEM are mapped, any
 * other pair is skipped with a warning.
 */

#ifndef _LINUX_MSM_BUS_BOARD_H
#define _LINUX_MSM_BUS_BOARD_H

/* Master IDs used by IPA v2 source */
#define MSM_BUS_MASTER_IPA		90
#define MSM_BUS_MASTER_BAM_DMA		91

/* Slave IDs used by IPA v2 source */
#define MSM_BUS_SLAVE_EBI_CH0		512
#define MSM_BUS_SLAVE_OCIMEM		513

#endif /* _LINUX_MSM_BUS_BOARD_H */
