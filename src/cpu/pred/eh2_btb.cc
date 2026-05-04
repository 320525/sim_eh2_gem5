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

#include <algorithm>
#include <cassert>

#include "base/intmath.hh"
#include "base/trace.hh"
#include "debug/BTB.hh"

namespace gem5
{

namespace branch_prediction
{
// RTL mapping: Verilog bit slicing helper used across hash equations.
// Function: extract inclusive [hi:lo] bits from an address.
unsigned
eh2BTB::extractBits(Addr w, unsigned hi, unsigned lo) const
{
    assert(hi >= lo && hi < 32);
    const unsigned n = hi - lo + 1;
    const uint32_t mask = n >= 32 ? ~0u : ((1u << n) - 1u);
    return static_cast<unsigned>((w >> lo) & mask);
}

// RTL mapping: design/lib/eh2_lib.sv::eh2_btb_addr_hash.
// Function: compute EH2 BTB set index hash.
unsigned
eh2BTB::addrHash(Addr instPC) const
{
    // RTL: eh2_btb_addr_hash (!BTB_FOLD2_INDEX_HASH, !BTB_USE_SRAM)
    // assign hash[BTB_ADDR_HI:BTB_ADDR_LO] =
    //   pc[BTB_INDEX1_HI:BTB_INDEX1_LO] ^
    //   pc[BTB_INDEX2_HI:BTB_INDEX2_LO] ^
    //   pc[BTB_INDEX3_HI:BTB_INDEX3_LO];
    const unsigned s1 = extractBits(instPC, idx1_hi, idx1_lo);
    const unsigned s2 = extractBits(instPC, idx2_hi, idx2_lo);
    const unsigned s3 = extractBits(instPC, idx3_hi, idx3_lo);
    const unsigned x = s1 ^ s2 ^ s3;
    return x & (numSets - 1);
}

// RTL mapping: design/lib/eh2_lib.sv::eh2_btb_tag_hash.
// Function: compute EH2 BTB tag hash for tag compare.
unsigned
eh2BTB::tagHash(Addr instPC) const
{
    // RTL: eh2_btb_tag_hash
    // hash = { pc[ADDR_HI+3B:ADDR_HI+2B+1] ^
    //          pc[ADDR_HI+2B:ADDR_HI+B+1] ^
    //          pc[ADDR_HI+B:ADDR_HI+1] }  (B = BTB_BTAG_SIZE)
    const unsigned t1 = extractBits(instPC, tag_xor1_hi, tag_xor1_lo);
    const unsigned t2 = extractBits(instPC, tag_xor2_hi, tag_xor2_lo);
    const unsigned t3 = extractBits(instPC, tag_xor3_hi, tag_xor3_lo);
    return (t1 ^ t2 ^ t3) & tagMask;
}

// RTL mapping: BTB geometry in eh2_ifu_bp_ctl.sv/eh2_param.vh.
// Function: construct BTB table + hash constants + replacement state.
eh2BTB::eh2BTB(const eh2BTBParams &p)
    : BranchTargetBuffer(p),
      numSets(std::max(1u, p.BTB_SIZE / 4u)),
      numWaysPerBank(2),
      tagBits(p.BTB_BTAG_SIZE),
      tagMask((1u << p.BTB_BTAG_SIZE) - 1u),
      idx1_hi(9),
      idx1_lo(3),
      idx2_hi(16),
      idx2_lo(10),
      idx3_hi(23),
      idx3_lo(17),
      tag_xor1_hi(24),
      tag_xor1_lo(20),
      tag_xor2_hi(19),
      tag_xor2_lo(15),
      tag_xor3_hi(14),
      tag_xor3_lo(10),
      replacementWay(numSets, {0, 0})
{
    // RTL: BTB_ARRAY_DEPTH x 2 bank x 2 way in eh2_ifu_bp_ctl.sv
    DPRINTF(BTB,
            "EH2 BTB: %u sets x 2 banks x %u ways (RTL BTB_ARRAY_DEPTH x 2bank x 2way).\n",
            numSets, numWaysPerBank);

    if (p.BTB_SIZE % 4 != 0) {
        fatal("eh2BTB: BTB_SIZE (%u) must be a multiple of 4 (512 => 128 sets).",
              p.BTB_SIZE);
    }
    if (!isPowerOf2(numSets)) {
        fatal("eh2BTB: number of sets (%u) must be a power of 2.", numSets);
    }

    sets.resize(numSets);
}

// RTL mapping: clear BTB valid state (btb_valid / BTB flops reset behavior).
// Function: invalidate table and reset replacement state.
void
eh2BTB::memInvalidate()
{
    // RTL: equivalent to clearing btb_valid[] and BTB_FLOPS contents.
    for (auto &set : sets) {
        for (auto &bankWays : set.bankWays) {
            for (auto &way : bankWays) {
                way.valid = false;
                way.inst = nullptr;
                way.target.reset();
            }
        }
    }
    for (auto &repl : replacementWay) {
        repl[0] = 0;
        repl[1] = 0;
    }
    mpCollisionArbFavor = false;
    exu_mp_write_valid = false;
    exu_mp_write_both = false;
    exu_mp_write_tid = static_cast<ThreadID>(0);
    t0_error_lockout_index = 0;
    t1_error_lockout_index = 0;
}


// RTL mapping: way encode used by way_raw/ifu_bp_way_f2 decisions.
// Function: return hit way bit (0..1), or -1 when miss.
int
eh2BTB::findWayIndex(Addr instPC) const
{
    const unsigned si = addrHash(instPC);
    const unsigned bank = bankOfPc(instPC);
    const unsigned tg = tagHash(instPC);
    assert(si < numSets);
    for (unsigned wayBit = 0; wayBit < numWaysPerBank; wayBit++) {
        const auto &way = sets[si].bankWays[bank][wayBit];
        if (isDecTluConflictSlot(si, bank, wayBit))
            continue;
        if (way.valid && way.tag == tg)
            return static_cast<int>(wayBit);
    }
    return -1;
}

// RTL mapping: ifu_bp_way_f2 (from way_raw): hit-way else replacement-way.
// Function: expose predicted way bit for queried PC.
uint8_t
eh2BTB::getWayF2(ThreadID tid, Addr instPC) const
{
    (void)tid;
    // Hit chooses matched way bit; miss uses bank-local replacement state.
    const unsigned si = addrHash(instPC);
    assert(si < numSets);
    const unsigned bank = bankOfPc(instPC);
    const int hitWayBit = findWayIndex(instPC);
    if (hitWayBit >= 0) {
        return static_cast<uint8_t>(hitWayBit);
    }
    return replacementWay[si][bank];
}

// RTL mapping: BTB hit-valid check in predict path.
// Function: return true when PC has BTB hit.
bool
eh2BTB::valid(ThreadID tid, Addr instPC)
{
    (void)tid;
    return findWay(instPC) != nullptr;
}

// RTL mapping: BTB lookup stage that also updates replacement state on hit.
// Function: return target on hit and track misses on miss.
const PCStateBase *
eh2BTB::lookup(ThreadID tid, Addr instPC, BranchType type)
{
    (void)tid;
    stats.lookups[type]++;

    const unsigned si = addrHash(instPC);
    const unsigned bank = bankOfPc(instPC);
    const int hitWayBit = findWayIndex(instPC);
    BTBWay *hitWay = (hitWayBit >= 0) ?
        &sets[si].bankWays[bank][static_cast<unsigned>(hitWayBit)] : nullptr;
    if (hitWay) {
        touchReplacement(si, bank, static_cast<unsigned>(hitWayBit));
        return hitWay->target.get();
    }
    stats.misses[type]++;
    return nullptr;
}

// RTL mapping: fetch metadata attached to BTB hit entry.
// Function: return stored StaticInstPtr for hit entry.
const StaticInstPtr
eh2BTB::getInst(ThreadID tid, Addr instPC)
{
    (void)tid;
    BTBWay *hitWay = findWay(instPC);
    if (hitWay)
        return hitWay->inst;
    return nullptr;
}

// RTL mapping: BTB write/update path (exu_mp/dec_tlu driven in RTL).
// Function: insert/overwrite entry and update per-bank replacement state.
void
eh2BTB::update(ThreadID tid, Addr instPC, const PCStateBase &target,
               BranchType type, StaticInstPtr inst)
{
    (void)tid;
    const unsigned si = addrHash(instPC);
    const unsigned tg = tagHash(instPC);
    assert(si < numSets);

    stats.updates[type]++;

    const unsigned bank = bankOfPc(instPC);
    BTBWay *hit = findWay(instPC);
    BTBWay *slot = hit;
    unsigned fillWayBit = 0;
    if (!slot) {
        const unsigned victimWayBit = pickVictim(si, bank);
        slot = &sets[si].bankWays[bank][victimWayBit];
        fillWayBit = victimWayBit;
    } else {
        const int hitWay = findWayIndex(instPC);
        fillWayBit = static_cast<unsigned>(hitWay);
    }

    slot->valid = true;
    slot->tag = tg;
    set(slot->target, target);
    slot->inst = inst;
    // RTL: BTB_DWIDTH BOFF/PC4/CALL/RET come from execute (exu_mp_pkt)
    // when forming btb_wr_data[i] in eh2_ifu_bp_ctl.sv.
    slot->pc4 = true;
    slot->boffset = false;
    slot->call = false;
    slot->ret = false;

    touchReplacement(si, bank, fillWayBit);
}

// RTL mapping: bank selection equivalent for replacement bookkeeping.
// Function: map PC to bank {0,1}.
unsigned
eh2BTB::bankOfPc(Addr instPC) const
{
    return extractBits(instPC, 2, 2) & 0x1;
}

// Function: set opposite way as the next replacement candidate.
void
eh2BTB::touchReplacement(unsigned setIdx, unsigned bank, unsigned touchedWayBit)
{
    assert(setIdx < numSets);
    replacementWay[setIdx][bank & 0x1] =
        static_cast<uint8_t>((~touchedWayBit) & 0x1);
}

// Function: choose victim within bank (invalid-first else replacement bit).
unsigned
eh2BTB::pickVictim(unsigned setIdx, unsigned bank)
{
    assert(setIdx < numSets);
    if (!sets[setIdx].bankWays[bank][0].valid) {
        return 0;
    }
    if (!sets[setIdx].bankWays[bank][1].valid) {
        return 1;
    }
    return replacementWay[setIdx][bank];
}

// bool
// eh2BTB::eh2btbconflict(Addr instPC, bool dec_bank, Addr dec_mp_btb_addr)
// {
//     bool dec_tlu_error_wb = false;
//     const unsigned si = addrHash(instPC);
//     const unsigned bank = bankOfPc(instPC);

//     if(dec_mp_btb_addr == si && dec_tlu_error_wb)

//     assert(si < numSets);
//     for (auto &way : sets[si].bankWays[bank]) {
//         if (way.valid && way.tag == tg)
//             return &way;
//     }
//     return true
// }



bool
eh2BTB::isDecTluConflictSlot(unsigned setIdx, unsigned bank, unsigned wayBit) const
{
    const bool err_wb = dec_tlu_wb.dec_tlu_error_wb;
    if (!err_wb)
        return false;

    const bool br0_err = dec_tlu_wb.dec_tlu_br0_wb_pkt.br_error ||
                         dec_tlu_wb.dec_tlu_br0_wb_pkt.br_start_error;
    const Eh2BrTluPkt &err_pkt = br0_err ? dec_tlu_wb.dec_tlu_br0_wb_pkt
                                         : dec_tlu_wb.dec_tlu_br1_wb_pkt;
    const unsigned err_set = dec_tlu_wb.btb_error_addr_wb & (numSets - 1);
    const unsigned dec_tlu_bank = static_cast<unsigned>(err_pkt.bank) & 0x1;
    const unsigned dec_tlu_way = static_cast<unsigned>(err_pkt.way) & 0x1;

    return (err_set == setIdx) &&
           (dec_tlu_bank == (bank & 0x1)) &&
           (dec_tlu_way == (wayBit & 0x1));
}

// RTL mapping: BTB tag compare path over all candidate ways.
// Function: find mutable pointer to hit way.
eh2BTB::BTBWay *
eh2BTB::findWay(Addr instPC)
{
    // Single-PC lookup probes only the selected bank in 2bankx2way BTB.
    const unsigned si = addrHash(instPC);
    const unsigned bank = bankOfPc(instPC);
    const unsigned tg = tagHash(instPC);

    assert(si < numSets);
    
    for (unsigned wayBit = 0; wayBit < numWaysPerBank; ++wayBit) {
        auto &way = sets[si].bankWays[bank][wayBit];
        if (isDecTluConflictSlot(si, bank, wayBit)) {
            continue;
        }
        if (way.valid && way.tag == tg)
            return &way;
    }
    return nullptr;
}

// Fetch line: four 16-bit slots at instPC+0,+2,+4,+6.
// btb_sel_f2[3:0] is BHT predicted-taken bits (1 means predicted taken).
// Return the first slot where BHT predicts taken and BTB hits.
eh2BTB::BTBWay *
eh2BTB::getHitSlot(Addr instPC, unsigned btb_sel_f2)
{
    for (unsigned slot = 0; slot < 4; ++slot) {
        if (((btb_sel_f2 >> slot) & 0x1u) == 0) {
            continue;
        }
        BTBWay *hitWay = findWay(instPC + static_cast<Addr>(2 * slot));
        if (hitWay != nullptr) {
            return hitWay;
        }
    }
    return nullptr;
}


const PCStateBase *
eh2BTB::eh2btblookup(ThreadID tid, Addr instPC, BranchType type, unsigned btb_sel_f2)
{
    (void)tid;
    stats.lookups[type]++;

    BTBWay *hitWay = getHitSlot(instPC,btb_sel_f2);
    
    if (hitWay) {
        //touchReplacement(si, bank, static_cast<unsigned>(hitWayBit));
        return hitWay->target.get();
    }
    stats.misses[type]++;
    return nullptr;
}

bool
eh2BTB::get_mp_collision() const
{
    const bool err_wb = dec_tlu_wb.dec_tlu_error_wb;
    return exu_mp_btb.exu_mp_pkt[0].misp && !err_wb &&
           exu_mp_btb.exu_mp_pkt[1].misp && !err_wb &&
           (exu_mp_btb.exu_mp_index[0] == exu_mp_btb.exu_mp_index[1]) &&
           (exu_mp_btb.exu_mp_pkt[0].bank == exu_mp_btb.exu_mp_pkt[1].bank) &&
           (exu_mp_btb.exu_mp_pkt[0].way == exu_mp_btb.exu_mp_pkt[1].way);
}

// RTL: rvarbiter2 + RV_ARBITER2 (eh2_ifu_bp_ctl.sv mp_arbiter).
// ready[i] = exu_mp_valid[i] & mp_collision; exu_mp_valid ~ pkt.misp here.
ThreadID
eh2BTB::get_mp_collision_winner_tid(bool mp_collision)
{
    const bool rv0 = exu_mp_btb.exu_mp_pkt[0].misp && mp_collision;
    const bool rv1 = exu_mp_btb.exu_mp_pkt[1].misp && mp_collision;

    const bool rdy0 = !(rv0 || rv1);
    const bool rdy1 = rv0 ^ rv1;
    const bool rdy2 = rv0 && rv1;

    const bool favor = mpCollisionArbFavor;
    const bool favor_in = (rdy2 && !favor) || (rdy1 && rv0) || (rdy0 && favor);
    const bool tid_sel = (rdy2 && favor) || (rv1 && !rv0);

    if (mp_collision && rdy2)
        mpCollisionArbFavor = favor_in;

    return static_cast<ThreadID>(tid_sel ? 1 : 0);
}

bool
eh2BTB::update_btb_error_addr_wb()
{
    const bool br0_err = dec_tlu_wb.dec_tlu_br0_wb_pkt.br_error ||
                         dec_tlu_wb.dec_tlu_br0_wb_pkt.br_start_error;
    dec_tlu_wb.btb_error_addr_wb = br0_err ? dec_tlu_wb.dec_tlu_br0_index_wb
                                           : dec_tlu_wb.dec_tlu_br1_index_wb;
    return br0_err;
}

bool
eh2BTB::get_error_mp_collision()
{
    //update_btb_error_addr_wb();

    // eh2_ifu_bp_ctl.sv: EXU index vs registered lockout + dec_tlu_btb_write_kill.
    const unsigned mask = numSets - 1;
    const unsigned mp0 = exu_mp_btb.exu_mp_index[0] & mask;
    const unsigned mp1 = exu_mp_btb.exu_mp_index[1] & mask;
    const bool kill0 = dec_tlu_wb.dec_tlu_btb_write_kill[0];
    const bool kill1 = dec_tlu_wb.dec_tlu_btb_write_kill[1];
    return ((mp0 == (t1_error_lockout_index & mask)) && kill1) ||
           ((mp1 == (t0_error_lockout_index & mask)) && kill0);
}

void
eh2BTB::advance_dec_tlu_error_lockout()
{
    const bool br0_err = update_btb_error_addr_wb();

    if (!dec_tlu_wb.dec_tlu_error_wb)
        return;

    const bool dec_tlu_error_tid =
        br0_err ? dec_tlu_wb.dec_tlu_br0_wb_pkt.tid
                : dec_tlu_wb.dec_tlu_br1_wb_pkt.tid;
    const unsigned mask = numSets - 1;
    const unsigned addr = dec_tlu_wb.btb_error_addr_wb & mask;

    if (dec_tlu_error_tid)
        t1_error_lockout_index = addr;
    else
        t0_error_lockout_index = addr;
}

void
eh2BTB::btbWriteFromExuMp(size_t t, BranchType type, bool error_mp_collision)
{
    assert(t < exu_mp_btb.exu_mp_pkt.size());
    const exu_mp_pkt_struct &pkt = exu_mp_btb.exu_mp_pkt[t];

    const PCStateBase *ptarget = exu_mp_btb.exu_mp_target[t];
    if (!ptarget) {
        fatal("eh2BTB::btbWriteFromExuMp: exu_mp_btb.exu_mp_target[%zu] is null.", t);
    }

    const unsigned si = exu_mp_btb.exu_mp_index[t] & (numSets - 1);
    const unsigned bank = static_cast<unsigned>(pkt.bank) & 1u;
    const unsigned wayBit = static_cast<unsigned>(pkt.way) & 1u;
    const unsigned tg = exu_mp_btb.exu_mp_btag[t] & tagMask;
    assert(si < numSets);

    stats.updates[type]++;

    BTBWay *slot = &sets[si].bankWays[bank][wayBit];

    // RTL btb_wr_data valid bit: ~dec_tlu_error_wb & ~error_mp_collision
    slot->valid = !dec_tlu_wb.dec_tlu_error_wb && !error_mp_collision;
    slot->tag = tg;
    set(slot->target, *ptarget);
    slot->inst = exu_mp_btb.exu_mp_inst[t];
    slot->pc4 = pkt.pc4;
    slot->boffset = pkt.boffset;
    slot->call = pkt.pcall;
    slot->ret = pkt.pret;

    touchReplacement(si, bank, wayBit);
}


void
eh2BTB::btbWriteFromDecWb(BranchType type, bool error_mp_collision)
{
    const exu_mp_pkt_struct &pkt = exu_mp_btb.exu_mp_pkt[0];

    const PCStateBase *ptarget = exu_mp_btb.exu_mp_target[0];
    if (!ptarget) {
        fatal("eh2BTB::btbWriteFromExuMp: exu_mp_btb.exu_mp_target[0] is null.");
    }

    const unsigned si = dec_tlu_wb.btb_error_addr_wb;
    const unsigned bank = dec_tlu_wb.dec_tlu_br0_wb_pkt.br_error? dec_tlu_wb.dec_tlu_br0_wb_pkt.bank : dec_tlu_wb.dec_tlu_br1_wb_pkt.bank;
    const unsigned wayBit = dec_tlu_wb.dec_tlu_br0_wb_pkt.br_error? dec_tlu_wb.dec_tlu_br0_wb_pkt.way : dec_tlu_wb.dec_tlu_br1_wb_pkt.way;

    const unsigned tg = exu_mp_btb.exu_mp_btag[0] & tagMask;
    assert(si < numSets);

    stats.updates[type]++;

    BTBWay *slot = &sets[si].bankWays[bank][wayBit];

    // RTL btb_wr_data valid bit: ~dec_tlu_error_wb & ~error_mp_collision
    slot->valid = !dec_tlu_wb.dec_tlu_error_wb && !error_mp_collision;
    slot->tag = tg;
    set(slot->target, *ptarget);
    slot->inst = exu_mp_btb.exu_mp_inst[0];
    slot->pc4 = pkt.pc4;
    slot->boffset = pkt.boffset;
    slot->call = pkt.pcall;
    slot->ret = pkt.pret;

    touchReplacement(si, bank, wayBit);

}

void
eh2BTB::eh2btbupdate(Addr instPC, BranchType type)
{
    (void)instPC;

    exu_mp_write_both = false;

    // RTL exu_mp_valid_write gating: misp & ~dec_tlu_error_wb & ataken & ~pkt.valid
    // (plus collision / winner in the collision branch).
    auto mpWriteOk = [this](unsigned i) -> bool {
        const exu_mp_pkt_struct &p = exu_mp_btb.exu_mp_pkt[i];
        return p.misp && !dec_tlu_wb.dec_tlu_error_wb && p.ataken && !p.valid;
    };

    const bool mp_collision = get_mp_collision();
    ThreadID tid = static_cast<ThreadID>(0);

    if (mp_collision) {
        tid = get_mp_collision_winner_tid(mp_collision);
        exu_mp_write_valid = mpWriteOk(static_cast<unsigned>(tid));
    } else {
        const bool w0 = mpWriteOk(0);
        const bool w1 = mpWriteOk(1);
        if (w0 && !w1) {
            tid = static_cast<ThreadID>(0);
            exu_mp_write_valid = true;
        } else if (!w0 && w1) {
            tid = static_cast<ThreadID>(1);
            exu_mp_write_valid = true;
        } else if (w0 && w1) {
            tid = static_cast<ThreadID>(0);
            exu_mp_write_valid = true;
            exu_mp_write_both = true;
        } else {
            exu_mp_write_valid = false;
        }
    }

    exu_mp_write_tid = tid;
    const bool error_mp_collision = get_error_mp_collision();

    if (!exu_mp_write_valid && !dec_tlu_wb.dec_tlu_error_wb) {
        advance_dec_tlu_error_lockout();
        return;
    }

    if (dec_tlu_wb.dec_tlu_error_wb) {
        btbWriteFromDecWb(type, error_mp_collision);
        advance_dec_tlu_error_lockout();
        return;
    }

    if (exu_mp_write_both) {
        btbWriteFromExuMp(0, type, error_mp_collision);
        btbWriteFromExuMp(1, type, error_mp_collision);
    } else {
        btbWriteFromExuMp(static_cast<size_t>(tid), type, error_mp_collision);
    }

    advance_dec_tlu_error_lockout();
}



} // namespace branch_prediction
} // namespace gem5