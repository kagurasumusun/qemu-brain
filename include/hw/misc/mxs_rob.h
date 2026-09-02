/*
 * Freescale i.MX28 register-only peripherals (HSADC/SPDIF/DRAM/FlexCAN/ENET).
 * See hw/misc/mxs_rob.c for what is and isn't modelled.
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or later.
 */
#ifndef HW_MISC_MXS_ROB_H
#define HW_MISC_MXS_ROB_H

#include "hw/core/sysbus.h"

#define TYPE_MXS_HSADC     "mxs-hsadc"
#define TYPE_MXS_SPDIF     "mxs-spdif"
#define TYPE_MXS_DRAM      "mxs-dram"
#define TYPE_MXS_FLEXCAN   "mxs-flexcan"
#define TYPE_MXS_ENET      "mxs-enet"
#define TYPE_MXS_ENET_SWI  "mxs-enet-swi"
#define TYPE_MXS_AUDIOOUT  "mxs-audioout"

#endif /* HW_MISC_MXS_ROB_H */
