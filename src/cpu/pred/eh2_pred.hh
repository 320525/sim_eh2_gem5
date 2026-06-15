/*
 * Copyright (c) 2011-2012, 2014 ARM Limited
 * Copyright (c) 2010,2022-2023 The University of Edinburgh
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

#ifndef __CPU_PRED_EH2_PRED_HH__
#define __CPU_PRED_EH2_PRED_HH__
 
#include <array>
 
#include "base/types.hh"
#include "cpu/inst_seq.hh"
#include "cpu/pred/branch_type.hh"
#include "cpu/pred/eh2_bht.hh"
#include "cpu/pred/eh2_btb.hh"
#include "cpu/pred/eh2_ras.hh"
#include "cpu/static_inst.hh"
#include "params/BranchPredictor.hh"
#include "sim/sim_object.hh"
 
 namespace gem5
 {
 
 namespace branch_prediction
 {
 
class eh2_pred : public SimObject
 {
     typedef BranchPredictorParams Params;
 
  public:
     /**
      * @param params The params object, that has the size of the BP and BTB.
      */
    eh2_pred(const Params &p);
 
     /** Perform sanity checks after a drain. */
     void drainSanityCheck() const;
 
     bool predict(const StaticInstPtr &inst, const InstSeqNum &seqNum,
                  PCStateBase &pc, ThreadID tid);
 
     void update(const InstSeqNum &done_sn, ThreadID tid);
 
     void squash(const InstSeqNum &squashed_sn, ThreadID tid);
 
     void squash(const InstSeqNum &squashed_sn, const PCStateBase &corr_target,
                 bool actually_taken, ThreadID tid, bool from_commit=true);
 
    uint8_t btblookup(Addr fetch_addr);
    uint8_t predict(PCStateBase &pc, ThreadID tid, Addr fetch_addr,
                    bool ifc_fetch_req_f2);
    bool get_ifu_bp_kill_next_f2() const;
    uint8_t get_ifu_bp_way_f2(bool fetch_mp_collision_f2, bool exu_mp_way,
                              unsigned exu_mp_bank, Addr fetch_addr);
 
  protected:
    void lru_update(Addr addr);

    const unsigned numThreads;
    const unsigned instShiftAmt;

  public:
    /** The BTB. */
    eh2BTB *btb;
    /** The BHT. */
    eh2BHT *bht;
    /** The return address stack. */
    ReturnAddrStack *ras;
    std::array<eh2BTB::BTBWay *, 4> hitWays{};
     /** The global history register. */
    std::array<unsigned, 2> ghr{};
     /** lru */
    std::array<std::array<uint8_t, 128>, 2> lru_bank{};

     uint8_t force_taken = 0;
     uint8_t btb_sel_f2 = 0;

    uint8_t btb_hit_ways = 0;

    uint8_t bht_pred = 0;

    uint8_t bht_dir_f2 = 0;

    bool ifu_bp_kill_next_f2 = false;

    uint8_t tag_match_vway1_expanded_f2 = 0;
     //input        disable all branch prediction
    bool dec_tlu_bpred_disable = false;

    
    //input         fetch valid f1/f2
    bool ifc_fetch_req_f1 = false; 
    bool ifc_fetch_req_f2 = false;

    //output        ifu_bp_way_f2
    std::array<bool, 4> fetch_br_ret{0};
    std::array<bool, 4> fetch_br_pc4{0};
    std::array<bool, 4> fetch_br_way{0};
    std::array<bool, 4> fetch_br_taken{0};
    std::array<bool, 4> fetch_br_end{0};
    std::array<uint8_t, 4> fetch_br_counter{0};
 };
 
 } // namespace branch_prediction
 } // namespace gem5
 
#endif // __CPU_PRED_EH2_PRED_HH__
 