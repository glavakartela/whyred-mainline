/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Stub <net/rmnet_config.h> for the IPA v2 port to Linux 6.x.
 *
 * The vendor downstream tree shipped a private <net/rmnet_config.h>.
 * Most of what it provided (RMNET_IOCTL_INGRESS/EGRESS_FORMAT_* flags)
 * is actually duplicated in <linux/msm_rmnet.h> -- which the user has
 * already copied into include/uapi/linux/. So this stub only needs to
 * provide what's NOT in msm_rmnet.h:
 *
 *   RMNET_MAP_GET_CD_BIT(skb) -- QMAP "Command/Data" bit accessor.
 *
 * Defining the flag macros here too would generate "macro redefined"
 * warnings against the values in msm_rmnet.h.
 */

#ifndef _NET_RMNET_CONFIG_H_
#define _NET_RMNET_CONFIG_H_

#include <linux/skbuff.h>

/*
 * QMAP header CD (Command/Data) bit accessor.
 *
 * In the QMAP protocol used between the modem and the IPA, the most
 * significant bit of the first byte of a packet distinguishes a
 * command frame (CD=1) from a data frame (CD=0). Used in rmnet_ipa.c's
 * TX flow-control path to skip qdisc-stop logic for command frames.
 */
#define RMNET_MAP_GET_CD_BIT(skb)	(((skb)->data[0] & 0x80) >> 7)

#endif /* _NET_RMNET_CONFIG_H_ */
