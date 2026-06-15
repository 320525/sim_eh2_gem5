/*
 * Copyright (c) 2011, 2014 ARM Limited
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

#ifndef __CPU_PRED_2BIT_LOCAL_PRED_HH__
#define __CPU_PRED_2BIT_LOCAL_PRED_HH__

#include <array>
#include <vector>

#include "base/sat_counter.hh"
#include "base/types.hh"
#include "params/eh2BHT.hh"
#include "sim/sim_object.hh"

namespace gem5
{

namespace branch_prediction
{

/**
 * Implements a local predictor that uses the PC to index into a table of
 * counters.  Note that any time a pointer to the bp_history is given, it
 * should be NULL using this predictor because it does not have any branch
 * predictor state that needs to be recorded or updated; the update can be
 * determined solely by the branch being taken or not taken.
 */
class eh2BHT : public SimObject
{
  public:
    /**
     * Default branch predictor constructor.
     */
    eh2BHT(const eh2BHTParams &params);

    unsigned eh2bhtlookup(Addr fetch_addr, unsigned ghr,
                          std::array<uint8_t, 4> &count,
                          std::array<bool, 4> &taken);
    void eh2bhtupdate(Addr fetch_addr, unsigned ghr, uint8_t data,
                      bool write_en);

  private:
    /**
     *  Returns the taken/not taken prediction given the value of the
     *  counter.
     *  @param count The value of the counter.
     *  @return The prediction based on the counter value.
     */
    inline bool getPrediction(uint8_t &count);

    bool eh2bhtlookup_slot(Addr fetch_addr, unsigned ghr, uint8_t &count);
    unsigned extractBits(Addr w, unsigned hi, unsigned lo) const;
    unsigned addrHashbtb(Addr instPC) const;
    unsigned addrHash(Addr instPC, unsigned ghr) const;

    /** Size of the local predictor. */
    const unsigned localPredictorSize;

    /** Number of bits of the local predictor's counters. */
    const unsigned localCtrBits;

    /** Number of sets. */
    const unsigned localPredictorSets;

    /** Array of counters that make up the local predictor. */
    std::vector<SatCounter8> localCtrs;                       //SatCounter8是一个8位的饱和计数器类型

    std::vector<uint8_t> eh2localCtrs; 
    /** Mask to get index bits. */
    const unsigned indexMask;
    const unsigned numSets;
    const unsigned idx1_hi;
    const unsigned idx1_lo;
    const unsigned idx2_hi;
    const unsigned idx2_lo;
    const unsigned idx3_hi;
    const unsigned idx3_lo;
};

} // namespace branch_prediction
} // namespace gem5

#endif // __CPU_PRED_2BIT_LOCAL_PRED_HH__
