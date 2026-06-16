/*
 * Copyright (c) 2012 ARM Limited
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

#ifndef __CPU_O3_F4_DECODE_HH__
#define __CPU_O3_F4_DECODE_HH__

#include <array>
#include <list>
#include <string>

#include "base/statistics.hh"
#include "cpu/o3/comm.hh"
#include "cpu/o3/dyn_inst_ptr.hh"
#include "cpu/o3/limits.hh"
#include "cpu/timebuf.hh"

namespace gem5
 {
 
 struct BaseO3CPUParams;
 
 namespace o3
 {
 
 class CPU;
 
 /**
  * Decode class handles both single threaded and SMT
  * decode. Its width is specified by the parameters; each cycles it
  * tries to decode that many instructions. Because instructions are
  * actually decoded when the StaticInst is created, this stage does
  * not do much other than check any PC-relative branches.
  */
class F4Decode
 {
   public:
     /** Overall decode stage status. Used to determine if the CPU can
      * deschedule itself due to a lack of activity.
      */
     enum DecodeStatus
     {
         Active,
         Inactive
     };
 
     /** Individual thread status. */
     enum ThreadStatus
     {
         Running,
         Idle,
         StartSquash,
         Squashing,
         Blocked,
         Unblocking
     };
 
   private:
     /** Decode status. */
     DecodeStatus _status;
 
     /** Per-thread status. */
     ThreadStatus decodeStatus[MaxThreads];
 
   public:
     /** Decode constructor. */
    F4Decode(CPU *_cpu, const BaseO3CPUParams &params);
 
     void startupStage();
 
     /** Clear all thread-specific states */
     void clearStates(ThreadID tid);
 
     void resetStage();
 
     /** Returns the name of decode. */
     std::string name() const;
 
     /** Sets the main backwards communication time buffer pointer. */
     void setTimeBuffer(TimeBuffer<TimeStruct> *tb_ptr);
 
     /** Sets pointer to time buffer used to communicate to the next stage. */
     void setDecodeQueue(TimeBuffer<DecodeStruct> *dq_ptr);
    void setToDecodeQueue(TimeBuffer<DecodeStruct> *queue_ptr);
 
     /** Sets pointer to time buffer coming from fetch. */
     void setFetchQueue(TimeBuffer<AlignF3Struct> *fq_ptr);
    void setF3AlignQueue(TimeBuffer<AlignF3Struct> *queue_ptr);
 
     /** Sets pointer to list of active threads. */
     void setActiveThreads(std::list<ThreadID> *at_ptr);
 
     /** Ticks decode. */
     void tick();
    void update_ib();
 
   private:
     // Interfaces to objects outside of decode.
     /** CPU interface. */
     CPU *cpu;
 
     /** Time buffer interface. */
     TimeBuffer<TimeStruct> *timeBuffer;
 
     /** Wire to get rename's output from backwards time buffer. */
     TimeBuffer<TimeStruct>::wire fromRename;
 
     /** Wire to get iew's information from backwards time buffer. */
     TimeBuffer<TimeStruct>::wire fromIEW;
 
     /** Wire to get commit's information from backwards time buffer. */
     TimeBuffer<TimeStruct>::wire fromCommit;
 
     /** Wire to write information heading to previous stages. */
     // Might not be the best name as not only fetch will read it.
     TimeBuffer<TimeStruct>::wire toFetch;
 
     /** Decode instruction queue. */
     TimeBuffer<DecodeStruct> *decodeQueue;
 
     /** Wire used to write any information heading to rename. */
     TimeBuffer<DecodeStruct>::wire toRename;
 
     /** Fetch instruction queue interface. */
     TimeBuffer<AlignF3Struct> *alignQueue;
 
     /** Wire to get fetch's output from fetch queue. */
     TimeBuffer<AlignF3Struct>::wire fromAlign;
 
     /** Variable that tracks if decode has written to the time buffer this
      * cycle. Used to tell CPU if there is activity this cycle.
      */
     bool wroteToTimeBuffer;
 
     /** Source of possible stalls. */
     struct Stalls
     {
         bool rename;
     };
 
     /** Tracks which stages are telling decode to stall. */
     Stalls stalls[MaxThreads];
 
     /** Rename to decode delay. */
     Cycles renameToDecodeDelay;
 
     /** IEW to decode delay. */
     Cycles iewToDecodeDelay;
 
     /** Commit to decode delay. */
     Cycles commitToDecodeDelay;
 
     /** Fetch to decode delay. */
     Cycles fetchToDecodeDelay;
 
     /** The width of decode, in instructions. */
     unsigned decodeWidth;
 
     /** number of Active Threads*/
     ThreadID numThreads;
 
     /** List of active thread ids */
     std::list<ThreadID> *activeThreads;
 
     /** Maximum size of the skid buffer. */
     unsigned skidBufferMax;
 
     /** SeqNum of Squashing Branch Delay Instruction (used for MIPS)*/
     Addr bdelayDoneSeqNum[MaxThreads];
 
     /** Instruction used for squashing branch (used for MIPS)*/
     DynInstPtr squashInst[MaxThreads];
 
     /** Tells when their is a pending delay slot inst. to send
      *  to rename. If there is, then wait squash after the next
      *  instruction (used for MIPS).
      */
     bool squashAfterDelaySlot[MaxThreads];
 
     struct DecodeStats : public statistics::Group
     {
         DecodeStats(CPU *cpu);
 
         /** Stat for total number of idle cycles. */
         statistics::Scalar idleCycles;
         /** Stat for total number of blocked cycles. */
         statistics::Scalar blockedCycles;
         /** Stat for total number of normal running cycles. */
         statistics::Scalar runCycles;
         /** Stat for total number of unblocking cycles. */
         statistics::Scalar unblockCycles;
         /** Stat for total number of squashing cycles. */
         statistics::Scalar squashCycles;
         /** Stat for number of times a branch is resolved at decode. */
         statistics::Scalar branchResolved;
         /** Stat for number of times a branch mispredict is detected. */
         statistics::Scalar branchMispred;
         /** Stat for number of times decode detected a non-control instruction
          * incorrectly predicted as a branch.
          */
         statistics::Scalar controlMispred;
         /** Stat for total number of decoded instructions. */
         statistics::Scalar decodedInsts;
         /** Stat for total number of squashed instructions. */
         statistics::Scalar squashedInsts;
     } stats;
 
 
  //////////////eh2 instruction buffer//////////////
     struct instruction_buffer
     {
         //br package
             bool inst0_br_start_error = false;
             bool inst0_br_error = false;
             bool inst1_br_start_error = false;
             bool inst1_br_error = false;
 
         //inst
             bool inst0_2B = false;
             bool inst1_2B = false;
             Addr inst0_addr = 0;
             Addr inst1_addr = 0;
 
         DynInstPtr insts;
 
         //ib signals
         bool valid;
    };
     std::array<instruction_buffer, 2> ib0{};
     std::array<instruction_buffer, 2> ib1{};
     std::array<instruction_buffer, 2> ib2{};
     std::array<instruction_buffer, 2> ib3{};
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 };
 
 } // namespace o3
 } // namespace gem5
 
#endif // __CPU_O3_F4_DECODE_HH__
 