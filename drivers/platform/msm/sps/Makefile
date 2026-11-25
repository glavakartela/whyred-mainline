# SPDX-License-Identifier: GPL-2.0-only
#
# Qualcomm SPS / BAM driver. Provides the SPS public API consumed by IPA v2.
#

obj-$(CONFIG_SPS) += msm_sps.o

msm_sps-y := \
	sps.o \
	sps_rm.o \
	sps_mem.o \
	sps_map.o \
	sps_bam.o \
	bam.o

msm_sps-$(CONFIG_SPS_SUPPORT_BAMDMA) += sps_dma.o

ccflags-y += -I$(srctree)/drivers/platform/msm/sps
ccflags-y += -include $(srctree)/drivers/platform/msm/ipa/ipa_v2/ipa_compat.h
