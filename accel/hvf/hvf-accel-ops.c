/*
 * Copyright 2008 IBM Corporation
 *           2008 Red Hat, Inc.
 * Copyright 2011 Intel Corporation
 * Copyright 2016 Veertu, Inc.
 * Copyright 2017 The Android Open Source Project
 *
 * QEMU Hypervisor.framework support
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of version 2 of the GNU General Public
 * License as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, see <http://www.gnu.org/licenses/>.
 *
 * This file contain code under public domain from the hvdos project:
 * https://github.com/mist64/hvdos
 *
 * Parts Copyright (c) 2011 NetApp, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY NETAPP, INC ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL NETAPP, INC OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#include "qemu/osdep.h"
#include "qemu/guest-random.h"
#include "qemu/main-loop.h"
#include "qemu/queue.h"
#include "gdbstub/enums.h"
<<<<<<< qemu-11.0.3-brain
#include "exec/cpu-common.h"
#include "hw/core/cpu.h"
#include "accel/accel-cpu-ops.h"
||||||| qemu-10.0.12
#include "hw/boards.h"
#include "system/accel-ops.h"
=======
#include "hw/boards.h"
#include "qapi/error.h"
#include "qapi/visitor.h"
#include "system/accel-ops.h"
>>>>>>> qemu-10.0.12-utm
#include "system/cpus.h"
#include "system/hvf.h"
#include "system/hvf_int.h"
<<<<<<< qemu-11.0.3-brain
#include <mach/mach_time.h>
||||||| qemu-10.0.12
#include "system/runstate.h"
#include "qemu/guest-random.h"
=======
#include "system/runstate.h"
#include "qemu/guest-random.h"
#include "hw/boards.h"
>>>>>>> qemu-10.0.12-utm

HVFState *hvf_state;
bool hvf_tso_mode = 0;
uint32_t hvf_ipa_granule_size = 0;

/* Memory slots */

<<<<<<< qemu-11.0.3-brain
||||||| qemu-10.0.12
hvf_slot *hvf_find_overlap_slot(uint64_t start, uint64_t size)
{
    hvf_slot *slot;
    int x;
    for (x = 0; x < hvf_state->num_slots; ++x) {
        slot = &hvf_state->slots[x];
        if (slot->size && start < (slot->start + slot->size) &&
            (start + size) > slot->start) {
            return slot;
        }
    }
    return NULL;
}

struct mac_slot {
    int present;
    uint64_t size;
    uint64_t gpa_start;
    uint64_t gva;
};

struct mac_slot mac_slots[32];

static int do_hvf_set_memory(hvf_slot *slot, hv_memory_flags_t flags)
{
    struct mac_slot *macslot;
    hv_return_t ret;

    macslot = &mac_slots[slot->slot_id];

    if (macslot->present) {
        if (macslot->size != slot->size) {
            macslot->present = 0;
            ret = hv_vm_unmap(macslot->gpa_start, macslot->size);
            assert_hvf_ok(ret);
        }
    }

    if (!slot->size) {
        return 0;
    }

    macslot->present = 1;
    macslot->gpa_start = slot->start;
    macslot->size = slot->size;
    ret = hv_vm_map(slot->mem, slot->start, slot->size, flags);
    assert_hvf_ok(ret);
    return 0;
}

static void hvf_set_phys_mem(MemoryRegionSection *section, bool add)
{
    hvf_slot *mem;
    MemoryRegion *area = section->mr;
    bool writable = !area->readonly && !area->rom_device;
    hv_memory_flags_t flags;
    uint64_t page_size = qemu_real_host_page_size();

    if (!memory_region_is_ram(area)) {
        if (writable) {
            return;
        } else if (!memory_region_is_romd(area)) {
            /*
             * If the memory device is not in romd_mode, then we actually want
             * to remove the hvf memory slot so all accesses will trap.
             */
             add = false;
        }
    }

    if (!QEMU_IS_ALIGNED(int128_get64(section->size), page_size) ||
        !QEMU_IS_ALIGNED(section->offset_within_address_space, page_size)) {
        /* Not page aligned, so we can not map as RAM */
        add = false;
    }

    mem = hvf_find_overlap_slot(
            section->offset_within_address_space,
            int128_get64(section->size));

    if (mem && add) {
        if (mem->size == int128_get64(section->size) &&
            mem->start == section->offset_within_address_space &&
            mem->mem == (memory_region_get_ram_ptr(area) +
            section->offset_within_region)) {
            return; /* Same region was attempted to register, go away. */
        }
    }

    /* Region needs to be reset. set the size to 0 and remap it. */
    if (mem) {
        mem->size = 0;
        if (do_hvf_set_memory(mem, 0)) {
            error_report("Failed to reset overlapping slot");
            abort();
        }
    }

    if (!add) {
        return;
    }

    if (area->readonly ||
        (!memory_region_is_ram(area) && memory_region_is_romd(area))) {
        flags = HV_MEMORY_READ | HV_MEMORY_EXEC;
    } else {
        flags = HV_MEMORY_READ | HV_MEMORY_WRITE | HV_MEMORY_EXEC;
    }

    /* Now make a new slot. */
    int x;

    for (x = 0; x < hvf_state->num_slots; ++x) {
        mem = &hvf_state->slots[x];
        if (!mem->size) {
            break;
        }
    }

    if (x == hvf_state->num_slots) {
        error_report("No free slots");
        abort();
    }

    mem->size = int128_get64(section->size);
    mem->mem = memory_region_get_ram_ptr(area) + section->offset_within_region;
    mem->start = section->offset_within_address_space;
    mem->region = area;

    if (do_hvf_set_memory(mem, flags)) {
        error_report("Error registering new memory slot");
        abort();
    }
}

=======
/* Default num of memslots to be allocated when VM starts */
#define  HVF_MEMSLOTS_NUM_ALLOC_DEFAULT                      32

static bool hvf_slots_grow(HVFState *state, unsigned int num_slots_new)
{
    unsigned int i, cur = state->num_slots;
    hvf_slot *slots;
    hvf_mac_slot *mac_slots;

    assert(num_slots_new > cur);
    if (cur == 0) {
        slots = g_new0(hvf_slot, num_slots_new);
        if (!slots) {
            return false;
        }
        mac_slots = g_new0(hvf_mac_slot, num_slots_new);
        if (!mac_slots) {
            g_free(slots);
            return false;
        }
    } else {
        slots = g_renew(hvf_slot, state->slots, num_slots_new);
        if (!slots) {
            return false;
        }
        mac_slots = g_renew(hvf_mac_slot, state->mac_slots, num_slots_new);
        if (!mac_slots) {
            /* save allocated slots but not new size */
            state->slots = slots;
            return false;
        }
        /*
         * g_renew() doesn't initialize extended buffers, however hvf
         * memslots require fields to be zero-initialized. E.g. pointers,
         * memory_size field, etc.
         */
        memset(&slots[cur], 0x0, sizeof(slots[0]) * (num_slots_new - cur));
        memset(&mac_slots[cur], 0x0, sizeof(mac_slots[0]) * (num_slots_new - cur));
    }

    for (i = cur; i < num_slots_new; i++) {
        slots[i].slot_id = i;
    }

    state->slots = slots;
    state->mac_slots = mac_slots;
    state->num_slots = num_slots_new;

    return true;
}

hvf_slot *hvf_find_overlap_slot(uint64_t start, uint64_t size)
{
    hvf_slot *slot;
    int x;
    for (x = 0; x < hvf_state->num_slots; ++x) {
        slot = &hvf_state->slots[x];
        if (slot->size && start < (slot->start + slot->size) &&
            (start + size) > slot->start) {
            return slot;
        }
    }
    return NULL;
}

static int do_hvf_set_memory(hvf_slot *slot, hv_memory_flags_t flags)
{
    hvf_mac_slot *macslot;
    hv_return_t ret;

    macslot = &hvf_state->mac_slots[slot->slot_id];

    if (macslot->present) {
        if (macslot->size != slot->size) {
            macslot->present = 0;
            ret = hv_vm_unmap(macslot->gpa_start, macslot->size);
            assert_hvf_ok(ret);
        }
    }

    if (!slot->size) {
        return 0;
    }

    macslot->present = 1;
    macslot->gpa_start = slot->start;
    macslot->size = slot->size;
    ret = hv_vm_map(slot->mem, slot->start, slot->size, flags);
    assert_hvf_ok(ret);
    return 0;
}

static void hvf_set_phys_mem(MemoryRegionSection *section, bool add)
{
    hvf_slot *mem;
    MemoryRegion *area = section->mr;
    bool writable = !area->readonly && !area->rom_device;
    hv_memory_flags_t flags;
    uint64_t page_size = qemu_real_host_page_size();

    if (!memory_region_is_ram(area)) {
        if (writable) {
            return;
        } else if (!memory_region_is_romd(area)) {
            /*
             * If the memory device is not in romd_mode, then we actually want
             * to remove the hvf memory slot so all accesses will trap.
             */
             add = false;
        }
    }

    if (hvf_ipa_granule_size) {
        page_size = hvf_ipa_granule_size;
    }

    if (!QEMU_IS_ALIGNED(int128_get64(section->size), page_size) ||
        !QEMU_IS_ALIGNED(section->offset_within_address_space, page_size)) {
        if (add) {
            warn_report("Cannot add 0x%016llX:0x%016llX because it is not "
                        "aligned to page size (0x%X)",
                        section->offset_within_address_space,
                        section->offset_within_address_space +
                        int128_get64(section->size),
                        (uint32_t)page_size);
        }
        /* Not page aligned, so we can not map as RAM */
        add = false;
    }

    mem = hvf_find_overlap_slot(
            section->offset_within_address_space,
            int128_get64(section->size));

    if (mem && add) {
        if (mem->size == int128_get64(section->size) &&
            mem->start == section->offset_within_address_space &&
            mem->mem == (memory_region_get_ram_ptr(area) +
            section->offset_within_region)) {
            return; /* Same region was attempted to register, go away. */
        }
    }

    /* Region needs to be reset. set the size to 0 and remap it. */
    if (mem) {
        mem->size = 0;
        if (do_hvf_set_memory(mem, 0)) {
            error_report("Failed to reset overlapping slot");
            abort();
        }
    }

    if (!add) {
        return;
    }

    if (area->readonly ||
        (!memory_region_is_ram(area) && memory_region_is_romd(area))) {
        flags = HV_MEMORY_READ | HV_MEMORY_EXEC;
    } else {
        flags = HV_MEMORY_READ | HV_MEMORY_WRITE | HV_MEMORY_EXEC;
    }

    /* Now make a new slot. */
    int x;

    for (x = 0; x < hvf_state->num_slots; ++x) {
        mem = &hvf_state->slots[x];
        if (!mem->size) {
            break;
        }
    }

    if (x == hvf_state->num_slots) {
        if (!hvf_slots_grow(hvf_state, hvf_state->num_slots * 2)) {
            error_report("Cannot allocate any more slots");
            abort();
        }
        mem = &hvf_state->slots[x];
    }

    mem->size = int128_get64(section->size);
    mem->mem = memory_region_get_ram_ptr(area) + section->offset_within_region;
    mem->start = section->offset_within_address_space;
    mem->region = area;

    if (do_hvf_set_memory(mem, flags)) {
        error_report("Error registering new memory slot");
        abort();
    }
}

>>>>>>> qemu-10.0.12-utm
static void do_hvf_cpu_synchronize_state(CPUState *cpu, run_on_cpu_data arg)
{
    if (!cpu->vcpu_dirty) {
        hvf_arch_get_registers(cpu);
        cpu->vcpu_dirty = true;
    }
}

static void hvf_cpu_synchronize_state(CPUState *cpu)
{
    if (!cpu->vcpu_dirty) {
        run_on_cpu(cpu, do_hvf_cpu_synchronize_state, RUN_ON_CPU_NULL);
    }
}

static void do_hvf_cpu_synchronize_set_dirty(CPUState *cpu,
                                             run_on_cpu_data arg)
{
    /* QEMU state is the reference, push it to HVF now and on next entry */
    cpu->vcpu_dirty = true;
}

static void hvf_cpu_synchronize_post_reset(CPUState *cpu)
{
    run_on_cpu(cpu, do_hvf_cpu_synchronize_set_dirty, RUN_ON_CPU_NULL);
}

static void hvf_cpu_synchronize_post_init(CPUState *cpu)
{
    run_on_cpu(cpu, do_hvf_cpu_synchronize_set_dirty, RUN_ON_CPU_NULL);
}

static void hvf_cpu_synchronize_pre_loadvm(CPUState *cpu)
{
    run_on_cpu(cpu, do_hvf_cpu_synchronize_set_dirty, RUN_ON_CPU_NULL);
}

static void dummy_signal(int sig)
{
}

<<<<<<< qemu-11.0.3-brain
static void do_hvf_get_vcpu_exec_time(CPUState *cpu, run_on_cpu_data arg)
||||||| qemu-10.0.12
bool hvf_allowed;

static int hvf_accel_init(MachineState *ms)
{
    int x;
    hv_return_t ret;
    HVFState *s;
    int pa_range = 36;
    MachineClass *mc = MACHINE_GET_CLASS(ms);

    if (mc->hvf_get_physical_address_range) {
        pa_range = mc->hvf_get_physical_address_range(ms);
        if (pa_range < 0) {
            return -EINVAL;
        }
    }

    ret = hvf_arch_vm_create(ms, (uint32_t)pa_range);
    assert_hvf_ok(ret);

    s = g_new0(HVFState, 1);

    s->num_slots = ARRAY_SIZE(s->slots);
    for (x = 0; x < s->num_slots; ++x) {
        s->slots[x].size = 0;
        s->slots[x].slot_id = x;
    }

    QTAILQ_INIT(&s->hvf_sw_breakpoints);

    hvf_state = s;
    memory_listener_register(&hvf_memory_listener, &address_space_memory);

    return hvf_arch_init();
}

static inline int hvf_gdbstub_sstep_flags(void)
{
    return SSTEP_ENABLE | SSTEP_NOIRQ;
}

static void hvf_accel_class_init(ObjectClass *oc, void *data)
{
    AccelClass *ac = ACCEL_CLASS(oc);
    ac->name = "HVF";
    ac->init_machine = hvf_accel_init;
    ac->allowed = &hvf_allowed;
    ac->gdbstub_supported_sstep_flags = hvf_gdbstub_sstep_flags;
}

static const TypeInfo hvf_accel_type = {
    .name = TYPE_HVF_ACCEL,
    .parent = TYPE_ACCEL,
    .class_init = hvf_accel_class_init,
};

static void hvf_type_init(void)
=======
bool hvf_allowed;

static int hvf_accel_init(MachineState *ms)
{
    int x;
    hv_return_t ret;
    HVFState *s;
    int pa_range = 36;
    MachineClass *mc = MACHINE_GET_CLASS(ms);
    bool success;

    if (mc->hvf_get_physical_address_range) {
        pa_range = mc->hvf_get_physical_address_range(ms);
        if (pa_range < 0) {
            return -EINVAL;
        }
    }

    ret = hvf_arch_vm_create(ms, (uint32_t)pa_range, hvf_ipa_granule_size);
    assert_hvf_ok(ret);

    s = g_new0(HVFState, 1);

    success = hvf_slots_grow(s, HVF_MEMSLOTS_NUM_ALLOC_DEFAULT);
    assert(success);

    QTAILQ_INIT(&s->hvf_sw_breakpoints);

    hvf_state = s;
    memory_listener_register(&hvf_memory_listener, &address_space_memory);

    return hvf_arch_init();
}

#if defined(__aarch64__)

static bool hvf_get_tso(Object *obj, Error **errp)
{
    return hvf_tso_mode;
}

static void hvf_set_tso(Object *obj, bool value, Error **errp)
{
    hvf_tso_mode = value;
}

#endif

static inline int hvf_gdbstub_sstep_flags(void)
{
    return SSTEP_ENABLE | SSTEP_NOIRQ;
}

static void hvf_get_ipa_granule_size(Object *obj, Visitor *v,
                                    const char *name, void *opaque,
                                    Error **errp)
{
    HVFState *s = HVF_STATE(obj);
    uint32_t value = hvf_ipa_granule_size;

    visit_type_uint32(v, name, &value, errp);
}

static void hvf_set_ipa_granule_size(Object *obj, Visitor *v,
                                     const char *name, void *opaque,
                                     Error **errp)
{
    HVFState *s = HVF_STATE(obj);
    uint32_t value;

    if (!visit_type_uint32(v, name, &value, errp)) {
        return;
    }
    if (value & (value - 1)) {
        error_setg(errp, "ipa-granule-size must be a power of two.");
        return;
    }

    hvf_ipa_granule_size = value;
}

static void hvf_accel_class_init(ObjectClass *oc, void *data)
{
    AccelClass *ac = ACCEL_CLASS(oc);
    ac->name = "HVF";
    ac->init_machine = hvf_accel_init;
    ac->allowed = &hvf_allowed;
    ac->gdbstub_supported_sstep_flags = hvf_gdbstub_sstep_flags;

#if defined(__aarch64__)
    object_class_property_add_bool(oc, "tso",
        hvf_get_tso, hvf_set_tso);
    object_class_property_set_description(oc, "tso",
        "Set on/off to enable/disable total store ordering mode");
#endif

    object_class_property_add(oc, "ipa-granule-size", "uint32",
        hvf_get_ipa_granule_size, hvf_set_ipa_granule_size,
        NULL, NULL);
    object_class_property_set_description(oc, "ipa-granule-size",
        "Size of a single guest page");
}

static const TypeInfo hvf_accel_type = {
    .name = TYPE_HVF_ACCEL,
    .parent = TYPE_ACCEL,
    .class_init = hvf_accel_class_init,
};

static void hvf_type_init(void)
>>>>>>> qemu-10.0.12-utm
{
    int r = hv_vcpu_get_exec_time(cpu->accel->fd, arg.host_ptr);
    assert_hvf_ok(r);
}

static void hvf_vcpu_destroy(CPUState *cpu)
{
    hv_return_t ret = hv_vcpu_destroy(cpu->accel->fd);
    assert_hvf_ok(ret);

    hvf_arch_vcpu_destroy(cpu);
    g_free(cpu->accel);
    cpu->accel = NULL;
}

static int hvf_init_vcpu(CPUState *cpu)
{
    int r;

    cpu->accel = g_new0(AccelCPUState, 1);

    /* init cpu signals */
    struct sigaction sigact;

    memset(&sigact, 0, sizeof(sigact));
    sigact.sa_handler = dummy_signal;
    sigaction(SIG_IPI, &sigact, NULL);

#ifdef __aarch64__
    cpu->accel->guest_debug_enabled = false;

    r = hv_vcpu_create(&cpu->accel->fd,
                       (hv_vcpu_exit_t **)&cpu->accel->exit, NULL);
#else
    r = hv_vcpu_create(&cpu->accel->fd, HV_VCPU_DEFAULT);
#endif
    assert_hvf_ok(r);
    cpu->vcpu_dirty = true;

    return hvf_arch_init_vcpu(cpu);
}

/*
 * The HVF-specific vCPU thread function. This one should only run when the host
 * CPU supports the VMX "unrestricted guest" feature.
 */
static void *hvf_cpu_thread_fn(void *arg)
{
    CPUState *cpu = arg;

    int r;

    assert(hvf_enabled());

    rcu_register_thread();

    bql_lock();
    qemu_thread_get_self(cpu->thread);

    cpu->thread_id = qemu_get_thread_id();
    current_cpu = cpu;

    hvf_init_vcpu(cpu);

    /* signal CPU creation */
    cpu_thread_signal_created(cpu);
    qemu_guest_random_seed_thread_part2(cpu->random_seed);

    do {
        qemu_process_cpu_events(cpu);
        if (cpu_can_run(cpu)) {
            r = hvf_arch_vcpu_exec(cpu);
            if (r == EXCP_DEBUG) {
                cpu_handle_guest_debug(cpu);
            }
        }
    } while (!cpu->unplug || cpu_can_run(cpu));

    hvf_vcpu_destroy(cpu);
    cpu_thread_signal_destroyed(cpu);
    bql_unlock();
    rcu_unregister_thread();
    return NULL;
}

static void hvf_start_vcpu_thread(CPUState *cpu)
{
    char thread_name[VCPU_THREAD_NAME_SIZE];

    /*
     * HVF currently does not support TCG, and only runs in
     * unrestricted-guest mode.
     */
    assert(hvf_enabled());

    snprintf(thread_name, VCPU_THREAD_NAME_SIZE, "CPU %d/HVF",
             cpu->cpu_index);
    qemu_thread_create(cpu->thread, thread_name, hvf_cpu_thread_fn,
                       cpu, QEMU_THREAD_JOINABLE);
}

struct hvf_sw_breakpoint *hvf_find_sw_breakpoint(CPUState *cpu, vaddr pc)
{
    struct hvf_sw_breakpoint *bp;

    QTAILQ_FOREACH(bp, &hvf_state->hvf_sw_breakpoints, entry) {
        if (bp->pc == pc) {
            return bp;
        }
    }
    return NULL;
}

int hvf_sw_breakpoints_active(CPUState *cpu)
{
    return !QTAILQ_EMPTY(&hvf_state->hvf_sw_breakpoints);
}

static void do_hvf_update_guest_debug(CPUState *cpu, run_on_cpu_data arg)
{
    hvf_arch_update_guest_debug(cpu);
}

int hvf_update_guest_debug(CPUState *cpu)
{
    run_on_cpu(cpu, do_hvf_update_guest_debug, RUN_ON_CPU_NULL);
    return 0;
}

static int hvf_insert_breakpoint(CPUState *cpu, int type, vaddr addr, vaddr len)
{
    struct hvf_sw_breakpoint *bp;
    int err;

    if (type == GDB_BREAKPOINT_SW) {
        bp = hvf_find_sw_breakpoint(cpu, addr);
        if (bp) {
            bp->use_count++;
            return 0;
        }

        bp = g_new(struct hvf_sw_breakpoint, 1);
        bp->pc = addr;
        bp->use_count = 1;
        err = hvf_arch_insert_sw_breakpoint(cpu, bp);
        if (err) {
            g_free(bp);
            return err;
        }

        QTAILQ_INSERT_HEAD(&hvf_state->hvf_sw_breakpoints, bp, entry);
    } else {
        err = hvf_arch_insert_hw_breakpoint(addr, len, type);
        if (err) {
            return err;
        }
    }

    CPU_FOREACH(cpu) {
        err = hvf_update_guest_debug(cpu);
        if (err) {
            return err;
        }
    }
    return 0;
}

static int hvf_remove_breakpoint(CPUState *cpu, int type, vaddr addr, vaddr len)
{
    struct hvf_sw_breakpoint *bp;
    int err;

    if (type == GDB_BREAKPOINT_SW) {
        bp = hvf_find_sw_breakpoint(cpu, addr);
        if (!bp) {
            return -ENOENT;
        }

        if (bp->use_count > 1) {
            bp->use_count--;
            return 0;
        }

        err = hvf_arch_remove_sw_breakpoint(cpu, bp);
        if (err) {
            return err;
        }

        QTAILQ_REMOVE(&hvf_state->hvf_sw_breakpoints, bp, entry);
        g_free(bp);
    } else {
        err = hvf_arch_remove_hw_breakpoint(addr, len, type);
        if (err) {
            return err;
        }
    }

    CPU_FOREACH(cpu) {
        err = hvf_update_guest_debug(cpu);
        if (err) {
            return err;
        }
    }
    return 0;
}

static void hvf_remove_all_breakpoints(CPUState *cpu)
{
    struct hvf_sw_breakpoint *bp, *next;
    CPUState *tmpcpu;

    QTAILQ_FOREACH_SAFE(bp, &hvf_state->hvf_sw_breakpoints, entry, next) {
        if (hvf_arch_remove_sw_breakpoint(cpu, bp) != 0) {
            /* Try harder to find a CPU that currently sees the breakpoint. */
            CPU_FOREACH(tmpcpu)
            {
                if (hvf_arch_remove_sw_breakpoint(tmpcpu, bp) == 0) {
                    break;
                }
            }
        }
        QTAILQ_REMOVE(&hvf_state->hvf_sw_breakpoints, bp, entry);
        g_free(bp);
    }
    hvf_arch_remove_all_hw_breakpoints();

    CPU_FOREACH(cpu) {
        hvf_update_guest_debug(cpu);
    }
}

static void hvf_get_vcpu_stats(CPUState *cpu, GString *buf)
{
    uint64_t time_mach; /* units of mach_absolute_time() */

    run_on_cpu(cpu, do_hvf_get_vcpu_exec_time, RUN_ON_CPU_HOST_PTR(&time_mach));

    mach_timebase_info_data_t timebase;
    mach_timebase_info(&timebase);
    uint64_t time_ns = time_mach * timebase.numer / timebase.denom;

    g_string_append_printf(buf, "HVF cumulative execution time: %llu.%.3llus\n",
                                 time_ns / 1000000000,
                                (time_ns % 1000000000) / 1000000);
}

static void hvf_accel_ops_class_init(ObjectClass *oc, const void *data)
{
    AccelOpsClass *ops = ACCEL_OPS_CLASS(oc);

    ops->cpu_target_realize = hvf_arch_cpu_realize;

    ops->create_vcpu_thread = hvf_start_vcpu_thread;
    ops->kick_vcpu_thread = hvf_kick_vcpu_thread;
    ops->handle_interrupt = generic_handle_interrupt;

    ops->synchronize_post_reset = hvf_cpu_synchronize_post_reset;
    ops->synchronize_post_init = hvf_cpu_synchronize_post_init;
    ops->synchronize_state = hvf_cpu_synchronize_state;
    ops->synchronize_pre_loadvm = hvf_cpu_synchronize_pre_loadvm;

    ops->insert_breakpoint = hvf_insert_breakpoint;
    ops->remove_breakpoint = hvf_remove_breakpoint;
    ops->remove_all_breakpoints = hvf_remove_all_breakpoints;
    ops->update_guest_debug = hvf_update_guest_debug;
    ops->supports_guest_debug = hvf_arch_supports_guest_debug;

    ops->get_vcpu_stats = hvf_get_vcpu_stats;
};

static const TypeInfo hvf_accel_ops_type = {
    .name = ACCEL_OPS_NAME("hvf"),

    .parent = TYPE_ACCEL_OPS,
    .class_init = hvf_accel_ops_class_init,
    .abstract = true,
};

static void hvf_accel_ops_register_types(void)
{
    type_register_static(&hvf_accel_ops_type);
}

type_init(hvf_accel_ops_register_types);
