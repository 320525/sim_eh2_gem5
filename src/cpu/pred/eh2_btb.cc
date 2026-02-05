/*
 * Copyright (c) 2022-2023 The University of Edinburgh
 * All rights reserved
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
 *
 * Copyright (c) 2004-2005 The Regents of The University of Michigan
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "cpu/pred/eh2_btb.hh"

#include "base/intmath.hh"
#include "base/trace.hh"
#include "debug/BTB.hh"

namespace gem5
{

namespace branch_prediction
{

eh2BTB::eh2BTB(const eh2BTBParams &p)
    : BranchTargetBuffer(p),
        numEntries(p.BTB_SIZE),
        tagBits(p.BTB_BTAG_SIZE),
        instShiftAmt(p.instShiftAmt),
        log2NumThreads(floorLog2(p.numThreads)),
        BTB_INDEX1_HI(p.BTB_INDEX1_HI),
        BTB_INDEX1_LO(p.BTB_INDEX1_LO),
        BTB_INDEX2_HI(p.BTB_INDEX2_HI),
        BTB_INDEX2_LO(p.BTB_INDEX2_LO),
        BTB_INDEX3_HI(p.BTB_INDEX3_HI),
        BTB_INDEX3_LO(p.BTB_INDEX3_LO),
        BTB_BTAG_SIZE(p.BTB_BTAG_SIZE)
{
    DPRINTF(BTB, "BTB: Creating BTB object.\n");                     //只有SConscript中DebugFlag('BTB')在顶层被定义时才会打印

    if (!isPowerOf2(numEntries)) {
        fatal("BTB entries is not a power of 2!");
    }

    btb.resize(numEntries);

    for (unsigned i = 0; i < numEntries; ++i) {
        btb[i].valid = false;
    }

    idxMask = numEntries - 1;

    tagMask = (1 << tagBits) - 1;

    tagShiftAmt = instShiftAmt + floorLog2(numEntries);
}

void
eh2BTB::memInvalidate()
{
    for (unsigned i = 0; i < numEntries; ++i) {
        btb[i].valid = false;
    }
}

inline
unsigned
eh2BTB::getIndex(Addr instPC, ThreadID tid)
{
    // Need to shift PC over by the word offset.
    unsigned part1 = (instPC >> (BTB_INDEX1_LO + 1)) & ((1ULL << (BTB_INDEX1_HI - BTB_INDEX1_LO)) - 1);
    unsigned part2 = (instPC >> (BTB_INDEX2_LO)) & ((1ULL << (BTB_INDEX2_HI - BTB_INDEX2_LO)) - 1);
    unsigned part3 = (instPC >> (BTB_INDEX3_LO)) & ((1ULL << (BTB_INDEX3_HI - BTB_INDEX3_LO)) - 1);
    unsigned xor_upper = part1 ^ part2 ^ part3;
    unsigned lsb = (instPC >> 3) & 1;
    return (xor_upper << 1) | lsb;
}

inline
Addr
eh2BTB::getTag(Addr instPC)
{
    unsigned long long mask = (1ULL << BTB_BTAG_SIZE) - 1;
    unsigned part1 = (instPC >> (BTB_ADDR_HI + BTB_BTAG_SIZE + 1)) & mask;
    unsigned part2 = (instPC >> (BTB_ADDR_HI + 1)) & mask;
    return part1 ^ part2;
}

eh2BTB::BTBEntry *
eh2BTB::findEntry(Addr instPC, ThreadID tid)
{
    unsigned btb_idx = getIndex(instPC, tid);
    Addr inst_tag = getTag(instPC);

    assert(btb_idx < numEntries);

    if (btb[btb_idx].valid
        && inst_tag == btb[btb_idx].tag
        && btb[btb_idx].tid == tid) {
        return &btb[btb_idx];                      //valid为1且tag相同tid相同 返回该条目指针 否则为空
    }

    return nullptr;
}

bool
eh2BTB::valid(ThreadID tid, Addr instPC)
{
    BTBEntry *entry = findEntry(instPC, tid);

    return entry != nullptr;
}

// @todo Create some sort of return struct that has both whether or not the
// address is valid, and also the address.  For now will just use addr = 0 to
// represent invalid entry.
const PCStateBase *
eh2BTB::lookup(ThreadID tid, Addr instPC, BranchType type)
{
    stats.lookups[type]++;

    BTBEntry *entry = findEntry(instPC, tid);

    if (entry) {
        return entry->target.get();                       //返回的是PCStateBase指针
    }
    stats.misses[type]++;
    return nullptr;                                        //未命中返回空指针
}

const StaticInstPtr
eh2BTB::getInst(ThreadID tid, Addr instPC)
{
    BTBEntry *entry = findEntry(instPC, tid);

    if (entry) {
        return entry->inst;
    }
    return nullptr;
}

void
eh2BTB::update(ThreadID tid, Addr instPC,
                    const PCStateBase &target,
                    BranchType type, StaticInstPtr inst)
{
    unsigned btb_idx = getIndex(instPC, tid);

    assert(btb_idx < numEntries);

    stats.updates[type]++;

    btb[btb_idx].tid = tid;
    btb[btb_idx].valid = true;
    set(btb[btb_idx].target, target);
    btb[btb_idx].tag = getTag(instPC);
    btb[btb_idx].inst = inst;
}

} // namespace branch_prediction
} // namespace gem5
