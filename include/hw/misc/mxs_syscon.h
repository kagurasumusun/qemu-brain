/*
 * Freescale i.MX28 (MXS) system control blocks (CLKCTRL/POWER/DIGCTL/...).
 * Small public accessors used by peer device models.
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or later.
 */
#ifndef HW_MISC_MXS_SYSCON_H
#define HW_MISC_MXS_SYSCON_H

#include "hw/core/sysbus.h"

/* HW_DIGCTL_MPTE<n>_LOC raw value (DIS|SPAN|LOC), n = 0..15. */
uint32_t mxs_digctl_mpte_loc(DeviceState *digctl, unsigned n);

#endif /* HW_MISC_MXS_SYSCON_H */
