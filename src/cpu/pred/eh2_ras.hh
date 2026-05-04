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

#ifndef __CPU_PRED_EH2_RAS_HH__
#define __CPU_PRED_EH2_RAS_HH__

#include <memory>
#include <string>
#include <vector>

#include "arch/generic/pcstate.hh"
#include "base/statistics.hh"
#include "base/types.hh"
#include "params/ReturnAddrStack.hh"
#include "sim/sim_object.hh"

namespace gem5
{

namespace branch_prediction
{

class ReturnAddrStack : public SimObject
{
  public:
    class AddrStack
    {
      public:
        explicit AddrStack(ReturnAddrStack &_parent) : parent(_parent) {}

        void init(unsigned numEntries);
        void reset();
        const PCStateBase *top();
        void push(const PCStateBase &return_addr);
        void pop();
        std::string toString(int n);

        inline void incrTos()
        {
            if (++tos == numEntries) {
                tos = 0;
            }
        }

        inline void decrTos()
        {
            tos = (tos == 0 ? numEntries - 1 : tos - 1);
        }

        std::vector<std::unique_ptr<PCStateBase>> addrStack;
        unsigned numEntries = 0;
        unsigned usedEntries = 0;
        unsigned tos = 0;

      protected:
        ReturnAddrStack &parent;
    };

    using Params = ReturnAddrStackParams;

    explicit ReturnAddrStack(const Params &p);

    void reset();
    void push(ThreadID tid, const PCStateBase &pc);
    const PCStateBase *pop(ThreadID tid);

  private:
    std::vector<AddrStack> addrStacks;
    unsigned numEntries;
    unsigned numThreads;

    struct ReturnAddrStackStats : public statistics::Group
    {
        explicit ReturnAddrStackStats(statistics::Group *parent);
        statistics::Scalar pushes;
        statistics::Scalar pops;
        statistics::Scalar squashes;
        statistics::Scalar used;
        statistics::Scalar correct;
        statistics::Scalar incorrect;
    } stats;
};

} // namespace branch_prediction
} // namespace gem5

#endif // __CPU_PRED_EH2_RAS_HH__
 