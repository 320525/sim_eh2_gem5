/*
 * Copyright (c) 2012, 2014 ARM Limited
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

#include "cpu/o3/f4_decode.hh"

#include "arch/generic/pcstate.hh"
#include "base/trace.hh"
#include "cpu/inst_seq.hh"
#include "cpu/o3/dyn_inst.hh"
#include "cpu/o3/limits.hh"
#include "debug/Activity.hh"
#include "debug/Decode.hh"
#include "debug/O3PipeView.hh"
#include "params/BaseO3CPU.hh"
#include "sim/full_system.hh"

// clang complains about std::set being overloaded with Packet::set if
// we open up the entire namespace std
 using std::list;
 
 namespace gem5
 {
 
 namespace o3
 {
 
F4Decode::F4Decode(CPU *_cpu, const BaseO3CPUParams &params)
     : cpu(_cpu),
       renameToDecodeDelay(params.renameToDecodeDelay),
       iewToDecodeDelay(params.iewToDecodeDelay),
       commitToDecodeDelay(params.commitToDecodeDelay),
       fetchToDecodeDelay(params.fetchToDecodeDelay),
       decodeWidth(params.decodeWidth),
       numThreads(params.numThreads),
       stats(_cpu)
 {
     if (decodeWidth > MaxWidth)
         fatal("decodeWidth (%d) is larger than compiled limit (%d),\n"
              "\tincrease MaxWidth in src/cpu/o3/limits.hh\n",
              decodeWidth, static_cast<int>(MaxWidth));
 
     // @todo: Make into a parameter
     skidBufferMax = (fetchToDecodeDelay + 1) *  params.decodeWidth;
     for (int tid = 0; tid < MaxThreads; tid++) {
         stalls[tid] = {false};
         decodeStatus[tid] = Idle;
         bdelayDoneSeqNum[tid] = 0;
         squashInst[tid] = nullptr;
         squashAfterDelaySlot[tid] = 0;
     }
 }
 
 void
F4Decode::startupStage()
 {
     resetStage();
 }
 
 void
F4Decode::clearStates(ThreadID tid)
 {
     decodeStatus[tid] = Idle;
     stalls[tid].rename = false;
 }
 
 void
F4Decode::resetStage()
 {
     _status = Inactive;
 
     // Setup status, make sure stall signals are clear.
     for (ThreadID tid = 0; tid < numThreads; ++tid) {
         decodeStatus[tid] = Idle;
 
         stalls[tid].rename = false;
     }
 }
 
 std::string
F4Decode::name() const
 {
    return cpu->name() + ".f4_decode";
 }
 
F4Decode::DecodeStats::DecodeStats(CPU *cpu)
    : statistics::Group(cpu, "f4_decode"),
       ADD_STAT(idleCycles, statistics::units::Cycle::get(),
                "Number of cycles decode is idle"),
       ADD_STAT(blockedCycles, statistics::units::Cycle::get(),
                "Number of cycles decode is blocked"),
       ADD_STAT(runCycles, statistics::units::Cycle::get(),
                "Number of cycles decode is running"),
       ADD_STAT(unblockCycles, statistics::units::Cycle::get(),
                "Number of cycles decode is unblocking"),
       ADD_STAT(squashCycles, statistics::units::Cycle::get(),
                "Number of cycles decode is squashing"),
       ADD_STAT(branchResolved, statistics::units::Count::get(),
                "Number of times decode resolved a branch"),
       ADD_STAT(branchMispred, statistics::units::Count::get(),
                "Number of times decode detected a branch misprediction"),
       ADD_STAT(controlMispred, statistics::units::Count::get(),
                "Number of times decode detected an instruction incorrectly "
                "predicted as a control"),
       ADD_STAT(decodedInsts, statistics::units::Count::get(),
                "Number of instructions handled by decode"),
       ADD_STAT(squashedInsts, statistics::units::Count::get(),
                "Number of squashed instructions handled by decode")
 {
     idleCycles.prereq(idleCycles);
     blockedCycles.prereq(blockedCycles);
     runCycles.prereq(runCycles);
     unblockCycles.prereq(unblockCycles);
     squashCycles.prereq(squashCycles);
     branchResolved.prereq(branchResolved);
     branchMispred.prereq(branchMispred);
     controlMispred.prereq(controlMispred);
     decodedInsts.prereq(decodedInsts);
     squashedInsts.prereq(squashedInsts);
 }
 
 void
F4Decode::setTimeBuffer(TimeBuffer<TimeStruct> *tb_ptr)
 {
     timeBuffer = tb_ptr;
 
     // Setup wire to write information back to fetch.
     toFetch = timeBuffer->getWire(0);
 
     // Create wires to get information from proper places in time buffer.
     fromRename = timeBuffer->getWire(-renameToDecodeDelay);
     fromIEW = timeBuffer->getWire(-iewToDecodeDelay);
     fromCommit = timeBuffer->getWire(-commitToDecodeDelay);
 }
 


void
F4Decode::setF3AlignQueue(TimeBuffer<AlignF3Struct> *queue_ptr)
 {
    alignQueue = queue_ptr;
 
    // Setup wire to read the aligned instruction bundle from F3.
    fromAlign = alignQueue->getWire(-1);
}

 
 void
F4Decode::setActiveThreads(std::list<ThreadID> *at_ptr)
 {
     activeThreads = at_ptr;
 }
 
 
 
 
 void
F4Decode::tick()
 {
     wroteToTimeBuffer = false;
 
 
 }
 
 void
F4Decode::update_ib()
 {
 
 }
 
 } // namespace o3
 } // namespace gem5
 