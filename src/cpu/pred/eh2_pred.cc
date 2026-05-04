/*
 * Copyright (c) 2011-2012, 2014 ARM Limited
 * Copyright (c) 2010,2022-2023 The University of Edinburgh
 * Copyright (c) 2012 Mark D. Hill and David A. Wood
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

#include "cpu/pred/eh2_pred.hh"

 #include <algorithm>
#include <cassert>
 
 #include "arch/generic/pcstate.hh"
 #include "base/trace.hh"
 #include "debug/Branch.hh"
 
 namespace gem5
 {
 
 namespace branch_prediction
 {
 
eh2_pred::eh2_pred(const Params &params)
     : SimObject(params),
       numThreads(params.numThreads),
       instShiftAmt(params.instShiftAmt),              //保存的指令右移位数
       btb(params.btb),
      bht(params.bht),
      ras(params.ras)
 {
 }
 
 
 void
eh2_pred::drainSanityCheck() const
 {
 }
 
 
 bool
eh2_pred::predict(const StaticInstPtr &inst, const InstSeqNum &seqNum,
                    PCStateBase &pc, ThreadID tid)
 {
    return false;
 }
 
 
 void
eh2_pred::update(const InstSeqNum &done_sn, ThreadID tid)
 {
 }

 void
eh2_pred::squash(const InstSeqNum &squashed_sn, ThreadID tid)
 {
 }

 void
eh2_pred::squash(const InstSeqNum &squashed_sn,
                   const PCStateBase &corr_target,
                   bool actually_taken, ThreadID tid, bool from_commit)
 {
 }
 

 uint8_t
eh2_pred::btblookup(Addr fetch_addr)
 {
    uint8_t hit_ways = 0;
   force_taken = 0;
    tag_match_vway1_expanded_f2 = 0;
    ifu_bp_ret_f2 = 0;
    ifu_bp_pc4_f2 = 0;
    
    for(int i = 0; i < 4; i++) {
    hitWays[i] = btb->findWay(fetch_addr + 2*i);
    if(hitWays[i] != nullptr) {
        hit_ways |= (1 << i);
        
        //if call or return, set force_taken
        if(hitWays[i]->call || hitWays[i]->ret)
        {
            force_taken |= (1 << i);
        }

        //used for update tag_match_vway1_expanded_f2
        if(hitWays[i]->used_way)
        {
            tag_match_vway1_expanded_f2 |= (1 << i);
        }

        //used for updating output ifu_bp_ret_f2
        if(hitWays[i]->ret && !hitWays[i]->call)
        {
            ifu_bp_ret_f2 |= (1 << i);
        }

        //used for updating output ifu_bp_pc4_f2
        if(hitWays[i]->pc4)
        {
            ifu_bp_pc4_f2 |= (1 << i);
        }
    }
    }
    return hit_ways & 0xF;

 }
 

 
 uint8_t
eh2_pred::predict(PCStateBase &pc, ThreadID tid, Addr fetch_addr, bool ifc_fetch_req_f2)
//  eh2_pred::predict(const StaticInstPtr &inst, const InstSeqNum &seqNum,
//                     PCStateBase &pc, ThreadID tid, Addr fetch_addr, ifc_fetch_req_f2)
{
    //write btb 4 hit ways to eh2_pred and generate 4bits hit vector
    btb_hit_ways = btblookup(fetch_addr);

    //lookup eh2 bht entry
    assert(bht != nullptr);
    bht_pred = bht->eh2bhtlookup(fetch_addr, ghr[tid]);

    bht_dir_f2 = (btb_hit_ways & (bht_pred | force_taken)) & 0xF;
    
    //get first taken
    btb_sel_f2 = 4;
    for (int i = 0; i <= 3; i++) {
        if (bht_dir_f2 & (1u << i)) {
            btb_sel_f2 = static_cast<uint8_t>(i);
            break;
        }
    }

    
    //kill next fetch signal
    ifu_bp_kill_next_f2 = (bht_dir_f2!=0) && ifc_fetch_req_f2 && dec_tlu_bpred_disable;
    // if(ifu_bp_kill_next_f2 == 0)
    // {
    //     return 0; 
    // }
    
    //instruction not taken branch found
    if(btb_sel_f2 == 4)
    {
        return 4;
    }
    
    //update read btb lru
    if(ifc_fetch_req_f2)
    {
        lru_update(fetch_addr); 
    }

    //first taken is return, pop return address from ras
    //需要更改如果ras为空时的逻辑
    if (hitWays[btb_sel_f2]->ret && ras) {
        const PCStateBase *rasTarget = ras->pop(tid);
        if (rasTarget != nullptr) {
            set(pc, *rasTarget);
            return btb_sel_f2;
        }
    }

    //set target of the first taken
    set(pc, *hitWays[btb_sel_f2]->target);

    //rtl mapping:btb_sel_f2 points to the first taken branch instruction
    return btb_sel_f2;
}

//note: use after predict function
bool
eh2_pred::get_ifu_bp_kill_next_f2() const
{
    return ifu_bp_kill_next_f2;
}


void
eh2_pred::lru_update(Addr addr)
 {
    unsigned btb_addr = btb->addrHash(addr);
    unsigned btb_bank = btb->bankOfPc(addr);
    lru_bank[btb_bank][btb_addr] = (lru_bank[btb_bank][btb_addr]+1) & 0x1;
 }


uint8_t
eh2_pred::get_ifu_bp_way_f2(bool fetch_mp_collision_f2, bool exu_mp_way, unsigned exu_mp_bank, Addr fetch_addr)
 {
    uint8_t btb_vlru_rd_f2 = 0;
    uint8_t btb_vlru_rd_f2_array[4] = {0};
    unsigned btb_bank_slot[4] = {0};

    for(int i = 0; i < 4; i++)
    {
        //look up bank of the slot
        btb_bank_slot[i] = btb->bankOfPc(fetch_addr + 2*i) & 0x1;
        
        //look up lru of the slot and store in array
        btb_vlru_rd_f2_array[i] = lru_bank[btb_bank_slot[i]][btb->addrHash(fetch_addr + 2*i)] & 0x1;
    }

    if(fetch_mp_collision_f2)
    {
        for(int i = 0; i < 4; i++)
        {
            if(btb_bank_slot[i] == exu_mp_bank)
            {
                btb_vlru_rd_f2_array[i] = exu_mp_way;
            }

        }
    }


    btb_vlru_rd_f2 = (btb_vlru_rd_f2_array[0] << 0) | (btb_vlru_rd_f2_array[1] << 1) | (btb_vlru_rd_f2_array[2] << 2) | (btb_vlru_rd_f2_array[3] << 3);
    
    ifu_bp_way_f2 = (tag_match_vway1_expanded_f2 | ((~btb_hit_ways) & btb_vlru_rd_f2)) & 0xF;
    return ifu_bp_way_f2;
 }

 } // namespace branch_prediction
 } // namespace gem5
 