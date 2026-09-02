/*
 * Freescale i.MX28 Default First-Level Page Table (DFLPT).
 * See hw/misc/mxs_dflpt.c.
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or later.
 */
#ifndef HW_MISC_MXS_DFLPT_H
#define HW_MISC_MXS_DFLPT_H

#include "hw/core/sysbus.h"

#define TYPE_MXS_DFLPT "mxs-dflpt"

/* link the DIGCTL block that holds the HW_DIGCTL_MPTEn_LOC registers */
void mxs_dflpt_set_digctl(DeviceState *dflpt, DeviceState *digctl);

#endif /* HW_MISC_MXS_DFLPT_H */
