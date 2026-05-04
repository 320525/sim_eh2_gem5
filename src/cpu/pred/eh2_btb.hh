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

/** @file
 * EH2 (VeeR-EH2) BTB model aligned to RTL:
 *   - eh2_ifu_bp_ctl.sv / eh2_ifu_btb_mem.sv: 2 bank x 2 way per set
 *   - eh2_lib.sv: eh2_btb_addr_hash, eh2_btb_tag_hash
 * Default geometry matches eh2_param.vh (BTB_ARRAY_DEPTH=128, BTB_SIZE=512,
 * BTB_BTAG_SIZE=5, BTB_TOFFSET_SIZE=12, non-SRAM, non-FOLD2 index hash).
 */

#ifndef __CPU_PRED_EH2_BTB_HH__
#define __CPU_PRED_EH2_BTB_HH__

#include <array>
#include <cstdint>
#include <vector>

#include "base/logging.hh"
#include "base/types.hh"
#include "cpu/pred/btb.hh"
#include "params/eh2BTB.hh"

namespace gem5
{

namespace branch_prediction
{

/** RTL: one element of \c exu_mp_pkt[i] — \c eh2_predict_pkt_t in
 *  design/include/eh2_def.sv. */
struct exu_mp_pkt_struct
{
    bool boffset = false;
    uint8_t hist = 0; /**< \c hist[1:0] */
    bool bank = false;
    bool way = false;
    bool ataken = false;
    bool valid = false;
    bool pc4 = false;
    bool misp = false;
    bool pcall = false;
    bool pret = false;
    bool pja = false;
};


struct exu_mp_btb_use
{
    std::array<exu_mp_pkt_struct, 2> exu_mp_pkt{};
    unsigned exu_mp_index[2]{};
    unsigned exu_mp_btag[2]{};
    /** Per-thread mispredict target PC for BTB write (non-owning; set before \c eh2btbupdate). */
    const PCStateBase *exu_mp_target[2]{nullptr, nullptr};
    /** Per-thread inst carried into BTB entry for that lane. */
    StaticInstPtr exu_mp_inst[2]{};
};
/** RTL: \c eh2_br_tlu_pkt_t in design/include/eh2_def.sv — DEC/TLU branch-predict
 *  commit update packet (\c dec_tlu_br0_wb_pkt / \c dec_tlu_br1_wb_pkt in
 *  eh2_ifu_bp_ctl.sv ~310–327). BTB/BHT index comes from parallel
 *  \c dec_tlu_br{0,1}_index_wb, not inside this struct. */
struct Eh2BrTluPkt
{
    bool valid = false;
    uint8_t hist = 0; /**< \c hist[1:0] */
    bool br_error = false;
    bool br_start_error = false;
    bool bank = false;
    bool way = false;
    bool middle = false;
    bool tid = false;
};


struct dec_tlu_wb_use
{
    Eh2BrTluPkt dec_tlu_br0_wb_pkt{};
    Eh2BrTluPkt dec_tlu_br1_wb_pkt{};
    /** RTL aggregate \c dec_tlu_error_wb (OR of br0/br1 \c br_error / \c br_start_error). */
    bool dec_tlu_error_wb = false;
    /** RTL \c dec_tlu_all_banks_error_wb — invalidate selected way in both banks. */
    bool dec_tlu_all_banks_error_wb = false;
    /** RTL \c dec_tlu_btb_write_kill[1:0] — per-thread kill for BTB write vs error lockout. */
    std::array<bool, 2> dec_tlu_btb_write_kill{};
    /** RTL \c dec_tlu_br{0,1}_index_wb — BTB set index (parallel packets, eh2_ifu_bp_ctl.sv). */
    unsigned dec_tlu_br0_index_wb = 0;
    unsigned dec_tlu_br1_index_wb = 0;
    unsigned btb_error_addr_wb = 0;
};



class eh2BTB : public BranchTargetBuffer
{
  public:
    /** RTL mapping:
     *  - eh2_ifu_bp_ctl.sv: BTB structures are 2-bank x 2-way per set.
     * Function:
     *  - Initialize EH2 BTB geometry/hash fields and local LRU state. */
    eh2BTB(const eh2BTBParams &params);

    /** RTL mapping:
     *  - Equivalent to clearing BTB valid contents (btb_valid / BTB flops).
     * Function:
     *  - Invalidate all entries and reset replacement state. */
    void memInvalidate() override;
    /** RTL mapping:
     *  - Tag-compare valid check in BTB read path.
     * Function:
     *  - Return whether instPC currently hits BTB. */
    bool valid(ThreadID tid, Addr instPC) override;
    /** RTL mapping:
     *  - BTB read/compare stage producing taken target candidate.
     * Function:
     *  - Lookup BTB target and update hit/miss stats. */
    const PCStateBase *lookup(ThreadID tid, Addr instPC,
                           BranchType type = BranchType::NoBranch) override;
    /** Fetch line: four 16-bit slots at instPC+0,+2,+4,+6. \p btb_sel_f2[3:0]
     *  is BHT predicted-taken (1 = predicted taken for that slot). Returns the
     *  first such slot where the BTB hits; otherwise records a miss and returns
     *  nullptr. */
    const PCStateBase *eh2btblookup(ThreadID tid, Addr instPC, BranchType type,
                                 unsigned btb_sel_f2);
    /** RTL mapping:
     *  - BTB write path: exu_mp/dec_tlu updates into BTB entry fields.
     * Function:
     *  - Insert/overwrite BTB entry and update replacement state. */
    void update(ThreadID tid, Addr instPC, const PCStateBase &target_pc,
                           BranchType type = BranchType::NoBranch,
                           StaticInstPtr inst = nullptr) override;
    /** EXU→BTB mispredict bundle (two threads). Fill before \c eh2btbupdate. */
    exu_mp_btb_use exu_mp_btb{};

    /** DEC/TLU write-back bundle; use \c dec_tlu_wb.dec_tlu_error_wb for EXU/BTB gating. */
    dec_tlu_wb_use dec_tlu_wb{};
    /** Last \c eh2btbupdate: \c exu_mp_valid_write qualified (see RTL gating). */
    bool exu_mp_write_valid = false;
    /** Thread selected for that write; meaningful only if \c exu_mp_write_valid. */
    ThreadID exu_mp_write_tid = static_cast<ThreadID>(0);
    /** When true, last \c eh2btbupdate wrote both thread 0 and 1 BTB slots. */
    bool exu_mp_write_both = false;
    /** BTB write using EXU-supplied slot for \p tid: \c exu_mp_index (set),
     *  \c exu_mp_pkt.bank / \c exu_mp_pkt.way, \c exu_mp_btag (tag), plus
     *  BTB_DWIDTH fields in \c exu_mp_pkt — no PC re-hash or victim search. */
    void eh2btbupdate(Addr instPC, BranchType type);
    /** RTL mapping:
     *  - Metadata carried with BTB entry in frontend.
     * Function:
     *  - Return stored instruction pointer for a BTB hit. */
    const StaticInstPtr getInst(ThreadID tid, Addr instPC) override;
    /** Predicted way bit aligned to RTL ifu_bp_way_f2 semantics for
     *  a single queried branch slot: hit uses matched way, miss uses LRU. */
    uint8_t getWayF2(ThreadID tid, Addr instPC) const;
    /** True when both threads target the same BTB slot (index/bank/way). */
    bool get_mp_collision() const;
    /** RTL \c error_mp_collision (eh2_ifu_bp_ctl.sv): EXU mispredict index hits other thread's
     *  error lockout while that thread's \c dec_tlu_btb_write_kill is set. */
    bool get_error_mp_collision();
    /** RTL \c errorindx flops: latch \c btb_error_addr_wb into per-thread lockout when
     *  \c dec_tlu_error_wb. Call once per cycle after BTB mispredict write uses
     *  \c get_error_mp_collision (same ordering as RTL posedge). */
    void advance_dec_tlu_error_lockout();
    /** RTL \c rvarbiter2 / \c RV_ARBITER2 on \c exu_mp_valid & mp_collision: returns
     *  winning \c ThreadID (0 or 1), updates internal \c favor when both ready. */
    ThreadID get_mp_collision_winner_tid(bool mp_collision);
  
  
    struct BTBWay
    {
        Addr tag = 0;
        std::unique_ptr<PCStateBase> target;
        StaticInstPtr inst = nullptr;
        bool valid = false;
        bool pc4 = false;
        bool boffset = false;
        bool call = false;
        bool ret = false;

        //used for dilivery way
        bool used_way;
        bool used_bank;
    };

    /** One set: LRU_SIZE index j with 4 flops (bank0/1 x way0/1).
     *  RTL: BTB_FLOPS in eh2_ifu_bp_ctl.sv (j loop). */
    struct BTBSet
    {
        std::array<std::array<BTBWay, 2>, 2> bankWays{};
    };


    unsigned addrHash(Addr instPC) const;

    unsigned tagHash(Addr instPC) const;

    unsigned extractBits(Addr w, unsigned hi, unsigned lo) const;


    BTBWay *findWay(Addr instPC);

    const BTBWay *findWayConst(Addr instPC) const;

    int findWayIndex(Addr instPC) const;

    unsigned bankOfPc(Addr instPC) const;

    unsigned pickVictim(unsigned setIdx, unsigned bank);

    void touchReplacement(unsigned setIdx, unsigned bank, unsigned touchedWayBit);

    bool isDecTluConflictSlot(unsigned setIdx, unsigned bank, unsigned wayBit) const;

    void btbWriteFromExuMp(size_t t, BranchType type, bool error_mp_collision);

    void btbWriteFromDecWb(BranchType type, bool error_mp_collision);

    /** RTL \c btb_error_addr_wb (eh2_ifu_bp_ctl.sv): mux br0/br1 index by err side.
     *  Writes \c dec_tlu_wb.btb_error_addr_wb; returns whether br0 carried the error
     *  (same condition as mux select, for \c dec_tlu_error_tid). */
    bool update_btb_error_addr_wb();

    /** Fetch-group 16b lanes: slot at instPC + 2*slot. \p mask[3:0] is the
     *  BHT predicted-taken vector (1 means predicted taken for that slot).
     *  Returns the first slot (lowest index) where BHT predicts taken and BTB
     *  also hits; returns nullptr when no such slot exists. */
    BTBWay *getHitSlot(Addr instPC, unsigned mask);

    std::vector<BTBSet> sets;
    unsigned numSets;
    unsigned numWaysPerBank;

    unsigned tagBits;
    unsigned tagMask;

    /** Index XOR fields (default EH2 from eh2_param.vh). */
    unsigned idx1_hi, idx1_lo;
    unsigned idx2_hi, idx2_lo;
    unsigned idx3_hi, idx3_lo;

    /** Tag XOR fields (default EH2, derived from BTB_ADDR_HI + BTB_BTAG). */
    unsigned tag_xor1_hi, tag_xor1_lo;
    unsigned tag_xor2_hi, tag_xor2_lo;
    unsigned tag_xor3_hi, tag_xor3_lo;

    /** Per-set per-bank replacement bit (0->way0 victim, 1->way1 victim). */
    std::vector<std::array<uint8_t, 2>> replacementWay;

    /** RTL \c mp_arbiter \c favor flop: default 0 (tid0 first on double-hit). */
    bool mpCollisionArbFavor = false;

    /** RTL \c t0_error_lockout_index / \c t1_error_lockout_index (errorindx in bp_ctl). */
    unsigned t0_error_lockout_index = 0;
    unsigned t1_error_lockout_index = 0;
};

} // namespace branch_prediction
} // namespace gem5

#endif // __CPU_PRED_EH2_BTB_HH__
