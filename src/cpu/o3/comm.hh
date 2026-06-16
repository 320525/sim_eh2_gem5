/*
 * Copyright (c) 2011, 2016-2017 ARM Limited
 * Copyright (c) 2013 Advanced Micro Devices, Inc.
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
 * Copyright (c) 2004-2006 The Regents of The University of Michigan
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

#ifndef __CPU_O3_COMM_HH__
#define __CPU_O3_COMM_HH__

#include <array>
#include <vector>

#include "arch/generic/pcstate.hh"
#include "base/types.hh"
#include "cpu/inst_seq.hh"
#include "cpu/o3/dyn_inst_ptr.hh"
#include "cpu/o3/limits.hh"
#include "mem/packet.hh"
#include "sim/faults.hh"

namespace gem5
{

namespace o3
{

/** Struct that defines the information passed from F1 fetch to F2 fetch. */
struct FetchF1F2Struct
{
    struct Entry
    {
        bool valid = false;
        ThreadID tid = InvalidThreadID;
        Addr startPC = 0;
        Addr fetchAddr = 0;
        Addr nextFetchAddr = 0;
        Addr predictedTarget = 0;
        Addr blockAddr = 0;
        Addr missReplayAddr = 0;
        bool fetchReq = false;
        bool fetchReqRaw = false;
        bool fetchReqWon = false;
        bool fetchReqF2 = false;
        bool ready = false;
        bool fromBtb = false;
        bool fromReplay = false;
        bool uncacheable = false;
        bool iccmAccess = false;
        bool regionFault = false;
        bool lineWrap = false;
        Tick tick = 0;
        RequestPtr memReq = nullptr;

        uint8_t data[8];
        uint8_t fetch_data_valid_slots;

        //br package
        std::array<bool, 4> fetch_br_ret{};
        std::array<bool, 4> fetch_br_pc4{};
        std::array<bool, 4> fetch_br_way{};
        std::array<bool, 4> fetch_br_end{};
        std::array<bool, 4> fetch_br_taken{};
        std::array<uint8_t, 4> fetch_br_counter{};
    };

    int size = 0;
    Entry entries[2];
    ThreadID tid_won;
};



/** Struct that defines the information passed from F3 align to decode. */
struct AlignF3Struct
{
    /** Same contract as \ref FetchStruct: DynInstPtr batch to Decode. */
    int size = 0;

    Fault fetchFault;
    InstSeqNum fetchFaultSN;
    bool clearFetchFault;

    //br package
    struct Entry{
        //br package
            bool inst0_br_start_error = false;
            bool inst0_br_error = false;
            bool inst1_br_start_error = false;
            bool inst1_br_error = false;

        //inst
            bool inst0_valid = false;
            bool inst1_valid = false;
            bool inst0_2B = false;
            bool inst1_2B = false;
            Addr inst0_addr = 0;
            Addr inst1_addr = 0;
            DynInstPtr insts[2];

            uint32_t inst0 = 0;
            uint32_t inst1 = 0;
    };
    Entry entries[2];
    

};

struct F3ToF1F2Struct
{
    bool fb_consume1[2]{};
    bool fb_consume2[2]{};
};

/** Struct that defines the information passed from fetch to decode. */
struct FetchStruct
{
    int size;

    DynInstPtr insts[MaxWidth];
    Fault fetchFault;
    InstSeqNum fetchFaultSN;
    bool clearFetchFault;
};

/** Struct that defines the information passed from decode to rename. */
struct DecodeStruct
{
    int size;

    DynInstPtr insts[MaxWidth];
};

/** Struct that defines the information passed from rename to IEW. */
struct RenameStruct
{
    int size;

    DynInstPtr insts[MaxWidth];
};

/** Struct that defines the information passed from IEW to commit. */
struct IEWStruct
{
    int size;

    DynInstPtr insts[MaxWidth];
    DynInstPtr mispredictInst[MaxThreads];
    Addr mispredPC[MaxThreads];
    InstSeqNum squashedSeqNum[MaxThreads];
    std::unique_ptr<PCStateBase> pc[MaxThreads];

    bool squash[MaxThreads];
    bool branchMispredict[MaxThreads];
    bool branchTaken[MaxThreads];
    bool includeSquashInst[MaxThreads];
};


//eh2 custom
struct ExuStruct
{
    bool exu_flush_final[2];
    bool dec_tlu_flush_err_wb[2]; 
    bool dec_tlu_flush_noredir_wb[2]; 
    bool dec_tlu_flush_lower_wb[2]; 
    bool dec_tlu_flush_mp_wb[2];
    bool dec_tlu_fence_i_wb[2]; 
    bool dec_tlu_flush_leak_one_wb[2]; 
    bool dec_tlu_force_halt[2]; 

    std::unique_ptr<PCStateBase> exu_flush_path_final[2];
    std::unique_ptr<PCStateBase> exu_flush_path_final_lastcycle[2];
    std::unique_ptr<PCStateBase> dec_tlu_flush_path_wb_lastcycle[2];
};



struct IssueStruct
{
    int size;

    DynInstPtr insts[MaxWidth];
};

/** Struct that defines all backwards communication. */
struct TimeStruct
{
    struct DecodeComm
    {
        std::unique_ptr<PCStateBase> nextPC;
        DynInstPtr mispredictInst;
        DynInstPtr squashInst;
        InstSeqNum doneSeqNum;
        Addr mispredPC;
        uint64_t branchAddr;
        unsigned branchCount;
        bool squash;
        bool predIncorrect;
        bool branchMispredict;
        bool branchTaken;
    };

    DecodeComm decodeInfo[MaxThreads];

    struct RenameComm {};

    RenameComm renameInfo[MaxThreads];

    struct IewComm
    {
        // Also eventually include skid buffer space.
        unsigned freeIQEntries;
        unsigned freeLQEntries;
        unsigned freeSQEntries;
        unsigned dispatchedToLQ;
        unsigned dispatchedToSQ;
        unsigned iqCount;
        unsigned ldstqCount;
        unsigned dispatched;
        bool usedIQ;
        bool usedLSQ;
    };

    IewComm iewInfo[MaxThreads];

    struct CommitComm
    {
        /////////////////////////////////////////////////////////////////////
        // This code has been re-structured for better packing of variables
        // instead of by stage which is the more logical way to arrange the
        // data.
        // F = Fetch
        // D = Decode
        // I = IEW
        // R = Rename
        // As such each member is annotated with who consumes it
        // e.g. bool variable name // *F,R for Fetch and Rename
        /////////////////////////////////////////////////////////////////////

        /// The pc of the next instruction to execute. This is the next
        /// instruction for a branch mispredict, but the same instruction for
        /// order violation and the like
        std::unique_ptr<PCStateBase> pc; // *F

        /// Provide fetch the instruction that mispredicted, if this
        /// pointer is not-null a misprediction occured
        DynInstPtr mispredictInst;  // *F

        /// Instruction that caused the a non-mispredict squash
        DynInstPtr squashInst; // *F

        /// Hack for now to send back a strictly ordered access to the
        /// IEW stage.
        DynInstPtr strictlyOrderedLoad; // *I

        /// Communication specifically to the IQ to tell the IQ that it can
        /// schedule a non-speculative instruction.
        InstSeqNum nonSpecSeqNum; // *I

        /// Represents the instruction that has either been retired or
        /// squashed.  Similar to having a single bus that broadcasts the
        /// retired or squashed sequence number.
        InstSeqNum doneSeqNum; // *F, I

        /// Tell Rename how many free entries it has in the ROB
        unsigned freeROBEntries; // *R

        bool squash; // *F, D, R, I
        bool robSquashing; // *F, D, R, I

        /// Rename should re-read number of free rob entries
        bool usedROB; // *R

        /// Notify Rename that the ROB is empty
        bool emptyROB; // *R

        /// Was the branch taken or not
        bool branchTaken; // *F
        /// If an interrupt is pending and fetch should stall
        bool interruptPending; // *F
        /// If the interrupt ended up being cleared before being handled
        bool clearInterrupt; // *F

        /// Hack for now to send back an strictly ordered access to
        /// the IEW stage.
        bool strictlyOrdered; // *I

    };

    CommitComm commitInfo[MaxThreads];

    bool decodeBlock[MaxThreads];
    bool decodeUnblock[MaxThreads];
    bool renameBlock[MaxThreads];
    bool renameUnblock[MaxThreads];
    bool iewBlock[MaxThreads];
    bool iewUnblock[MaxThreads];
};

} // namespace o3
} // namespace gem5

#endif //__CPU_O3_COMM_HH__
