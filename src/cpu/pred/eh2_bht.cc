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

#include "cpu/pred/eh2_bht.hh"

#include <cassert>

#include "base/intmath.hh"
#include "base/logging.hh"
#include "base/trace.hh"
#include "debug/Fetch.hh"

namespace gem5
{

namespace branch_prediction
{

eh2BHT::eh2BHT(const eh2BHTParams &params)
    : SimObject(params),
      localPredictorSize(4*128),
      localCtrBits(2),
      localPredictorSets(localPredictorSize / localCtrBits),
      localCtrs(localPredictorSize, SatCounter8(localCtrBits)),
      indexMask(localPredictorSize - 1),
      numSets(128),
      idx1_hi(9),
      idx1_lo(3),
      idx2_hi(16),
      idx2_lo(10),
      idx3_hi(23),
      idx3_lo(17),
      eh2localCtrs(4*128,0)
{
    if (!isPowerOf2(localPredictorSize)) {
        fatal("Invalid local predictor size!\n");
    }

    if (!isPowerOf2(localPredictorSets)) {
        fatal("Invalid number of local predictor sets! Check localCtrBits.\n");
    }

    DPRINTF(Fetch, "index mask: %#x\n", indexMask);

    DPRINTF(Fetch, "local predictor size: %i\n",
            localPredictorSize);

    DPRINTF(Fetch, "local counter bits: %i\n", localCtrBits);
}




//eh2 custom code
inline
bool
eh2BHT::getPrediction(uint8_t &count)
{
    // Get the MSB of the count
    return (count >> 1) & 0x1;           
}



//eh2 custom code
// RTL mapping: Verilog bit slicing helper used across hash equations.
// Function: extract inclusive [hi:lo] bits from an address.
unsigned
eh2BHT::extractBits(Addr w, unsigned hi, unsigned lo) const
{
    assert(hi >= lo && hi < 32);
    const unsigned n = hi - lo + 1;
    const uint32_t mask = n >= 32 ? ~0u : ((1u << n) - 1u);
    return static_cast<unsigned>((w >> lo) & mask);
}

// RTL mapping: design/lib/eh2_lib.sv::eh2_btb_addr_hash.
// Function: compute EH2 BTB set index hash.
unsigned
eh2BHT::addrHashbtb(Addr instPC) const
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

unsigned
eh2BHT::addrHash(Addr instPC, unsigned ghr) const
{
    // RTL: assign hash[pt.BHT_ADDR_HI:pt.BHT_ADDR_LO] =
    //   { hashin[pt.BHT_GHR_SIZE+2:5]^ghr[pt.BHT_GHR_SIZE-1:2], ghr[1:0]};
    constexpr unsigned bhtGhrSize = 7;
    constexpr unsigned upperWidth = bhtGhrSize - 2;
    constexpr unsigned upperMask = (1u << upperWidth) - 1u;
    constexpr unsigned hashMask = (1u << bhtGhrSize) - 1u;

    const unsigned hashin = addrHashbtb(instPC);
    const unsigned upper = ((hashin >> 5) & upperMask) ^
                           ((ghr >> 2) & upperMask);
    const unsigned lower = ghr & 0x3u;

    return ((upper << 2) | lower) & hashMask;
}


bool
eh2BHT::eh2bhtlookup_slot(Addr fetch_addr, unsigned ghr)
{
    bool taken;
  
    unsigned local_predictor_idx = addrHash(fetch_addr, ghr);


    uint8_t counter_val = eh2localCtrs[local_predictor_idx];              


    taken = getPrediction(counter_val);                               

    return taken;
}

unsigned
eh2BHT::eh2bhtlookup(Addr fetch_addr, unsigned ghr)
{
    bool taken_slot0 = eh2bhtlookup_slot(fetch_addr, ghr);
    bool taken_slot1 = eh2bhtlookup_slot(fetch_addr+2, ghr);
    bool taken_slot2 = eh2bhtlookup_slot(fetch_addr+4, ghr);
    bool taken_slot3 = eh2bhtlookup_slot(fetch_addr+6, ghr);

    // Return packed 4-bit prediction in slot order: {slot0, slot1, slot2, slot3}.
    return (static_cast<unsigned>(taken_slot3) << 3) |
           (static_cast<unsigned>(taken_slot2) << 2) |
           (static_cast<unsigned>(taken_slot1) << 1) |
           static_cast<unsigned>(taken_slot0);
}

void
eh2BHT::eh2bhtupdate(Addr fetch_addr, unsigned ghr, uint8_t data, bool write_en)
{
    unsigned local_predictor_idx = addrHash(fetch_addr, ghr);
    if (write_en) {
        eh2localCtrs[local_predictor_idx] = data;
    }
}


} // namespace branch_prediction
} // namespace gem5
