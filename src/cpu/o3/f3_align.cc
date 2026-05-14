#include "f3_align.hh"

#include <cstring>

#include "base/logging.hh"
#include "cpu/o3/cpu.hh"
#include "cpu/o3/dyn_inst.hh"
#include "debug/Fetch.hh"
#include "debug/O3CPU.hh"
#include "debug/O3PipeView.hh"
#include "params/BaseO3CPU.hh"
#include "sim/core.hh"

namespace gem5
{

namespace o3
{

F3Align::F3Align(CPU *_cpu, const BaseO3CPUParams &params)
    : cpu(_cpu),
    //   timeBuffer(nullptr),
    //   f1f2ToF3Buffer(nullptr),
    //   fetchQueueBuffer(nullptr),
    //   activeThreads(nullptr),
      _status(Inactive),
      fetchWidth(params.fetchWidth),
      decodeWidth(params.decodeWidth),
      fetchBufferSize(params.fetchBufferSize),
      numThreads(params.numThreads)
{
    for (ThreadID tid = 0; tid < MaxThreads; ++tid) {
        decoder[tid] = nullptr;
    }
    for (ThreadID tid = 0; tid < numThreads; ++tid) {
        decoder[tid] = params.decoder[tid];
    }
    resetStage();
    for (ThreadID tid = 0; tid < numThreads; ++tid) {
        decoder[tid] = params.decoder[tid];
    }
}

std::string
F3Align::name() const
{
    return cpu->name() + ".f3_align";
}

void
F3Align::setTimeBuffer(TimeBuffer<TimeStruct> *time_buffer)
{
    timeBuffer = time_buffer;
    fromDecode = timeBuffer->getWire(-1);
    fromRename = timeBuffer->getWire(-1);
    fromIEW = timeBuffer->getWire(-1);
    fromCommit = timeBuffer->getWire(-1);
}

void
F3Align::setActiveThreads(std::list<ThreadID> *at_ptr)
{
    activeThreads = at_ptr;
}

void
F3Align::setF1F2Buffer(TimeBuffer<FetchF1F2Struct> *queue_ptr)
{
    f1f2ToF3Buffer = queue_ptr;
    fromF1F2 = f1f2ToF3Buffer->getWire(-1);
}

void
F3Align::setFetchQueue(TimeBuffer<AlignF3Struct> *queue_ptr)
{
    fetchQueueBuffer = queue_ptr;
    toDecode = fetchQueueBuffer->getWire(0);
}

void
F3Align::setToF1F2FetchQueue(TimeBuffer<F3ToF1F2Struct> *queue_ptr)
{
    toF1F2FetchBuffer = queue_ptr;
    toF1F2Fetch = toF1F2FetchBuffer->getWire(0);
}

void
F3Align::startupStage()
{
    resetStage();
    _status = Active;
}

void
F3Align::resetStage()
{
    _status = Inactive;
    for (ThreadID tid = 0; tid < MaxThreads; ++tid) {
        fetchStatus[tid] = Idle;
        pendingLines[tid].clear();
    }
    for (ThreadID tid = 0; tid < numThreads; ++tid) {
        fetchbuffer0[tid] = FetchDataBlock{};
        fetchbuffer1[tid] = FetchDataBlock{};
        fetchbuffer2[tid] = FetchDataBlock{};
        fetchbuffer3[tid] = FetchDataBlock{};
    }
}

void
F3Align::clearStates(ThreadID tid)
{
    fetchStatus[tid] = Running;
    pendingLines[tid].clear();
    fetchbuffer0[tid] = FetchDataBlock{};
    fetchbuffer1[tid] = FetchDataBlock{};
    fetchbuffer2[tid] = FetchDataBlock{};
    fetchbuffer3[tid] = FetchDataBlock{};
}

void
F3Align::tick()
{
    toDecode->size = 0;
    for (int i = 0; i < MaxWidth; ++i) {
        toDecode->insts[i] = nullptr;
    }
    for (ThreadID tid = 0; tid < numThreads; ++tid) {
        toF1F2Fetch->fb_consume1[tid] = false;
        toF1F2Fetch->fb_consume2[tid] = false;
    }

    ThreadID tid_won = fromF1F2->tid_won;
    // if (fromF1F2->size == 0 || tid_won >= numThreads ||
    //     !fromF1F2->entries[tid_won].valid) {
    //     updatedff();
    //     return;
    // }

    //update_fetchbuffer(tid_won);
    
    instruction_block instr_temp{};
    get_instr(tid_won, &instr_temp);

    predecodeAndMaterializeAlignedInsts(tid_won, instr_temp);
    
    update_fetchbuffer(tid_won);
    
    updatedff();
}


void
F3Align::fetchbufferslots_shift(FetchDataBlock* fetchbuffer)
{
    FetchDataBlock fetchbuffer_temp{};
    uint8_t shift_slots = fetchbuffer->fb_valid_slots_num - fetchbuffer->fb_valid_slots_num_nextcycle;
    if(fetchbuffer->fb_valid_slots_num_nextcycle > 0)
    {
        for(int i = 0; i < 4 - shift_slots; i++)
        {
            fetchbuffer_temp.fetchbufferdata[i] = fetchbuffer->fetchbufferdata[i + shift_slots];
            fetchbuffer_temp.fetchbuffervalid[i] = fetchbuffer->fetchbuffervalid[i + shift_slots];
        }
    }
    fetchbuffer_temp.fb_valid_slots_num = fetchbuffer->fb_valid_slots_num_nextcycle;
    fetchbuffer_temp.fetch_firstslot_pcaddr =fetchbuffer->fetch_firstslot_pcaddr + shift_slots * 2;
    *fetchbuffer = fetchbuffer_temp;
}

void
F3Align::fetchbuffer_shift(uint8_t shift, ThreadID tid)
{
    if(shift == 0)
    {
        return;
    }
    
    if(shift == 1)
    {
        fetchbuffer0[tid] = fetchbuffer1[tid];
        fetchbuffer1[tid] = fetchbuffer2[tid];
        fetchbuffer2[tid] = fetchbuffer3[tid];
        fetchbuffer3[tid] = FetchDataBlock{};
    }

    if(shift == 2)
    {
        fetchbuffer0[tid] = fetchbuffer2[tid];
        fetchbuffer1[tid] = fetchbuffer3[tid];
        fetchbuffer2[tid] = FetchDataBlock{};
        fetchbuffer3[tid] = FetchDataBlock{};
    }
}

void 
F3Align::update_fetchbuffer(ThreadID tid)
{
    auto packFetchData = [&](int low) -> uint16_t {
        return (static_cast<uint16_t>(fromF1F2->entries[tid].data[low + 1]) << 8) |
                static_cast<uint16_t>(fromF1F2->entries[tid].data[low]);
    };

    //temporary fetchbuffer use for substitute fetchbuffer
    FetchDataBlock fetchbuffer_temp{};
    fetchbuffer_temp.fb_valid_slots_num = fromF1F2->entries[tid].fetch_data_valid_slots;
    fetchbuffer_temp.fetch_firstslot_pcaddr = fromF1F2->entries[tid].fetchAddr;
    for(int i = 0; i < fromF1F2->entries[tid].fetch_data_valid_slots; i++)
    {
        fetchbuffer_temp.fetchbufferdata[i] = packFetchData(2 * i);
        fetchbuffer_temp.fetchbuffervalid[i] = true;
    }

    //according f0buffer and f1buffer to used slots to update themselves initially
    fetchbufferslots_shift(&fetchbuffer0[tid]);
    fetchbufferslots_shift(&fetchbuffer1[tid]);

    uint8_t shift = fetchbuffer0[tid].fb_valid_slots_num == 0 && fetchbuffer1[tid].fb_valid_slots_num == 0 ? 2 : fetchbuffer0[tid].fb_valid_slots_num == 0 ? 1 : 0;
    fetchbuffer_shift(shift, tid);
    
    //update fetchbuffer3
    // if(fetchtof3[tid] && fromF1F2->entries[tid].fetch_data_valid_slots > 0)
    // {
    //     fetchbuffer3[tid] = fetchbuffer_temp;
    // }
    // if(fetchtof2[tid] && fromF1F2->entries[tid].fetch_data_valid_slots > 0)
    // {
    //     fetchbuffer2[tid] = fetchbuffer_temp;
    // }
    // if(fetchtof1[tid] && fromF1F2->entries[tid].fetch_data_valid_slots > 0)
    // {
    //     fetchbuffer1[tid] = fetchbuffer_temp;
    // }
    // if(fetchtof0[tid] && fromF1F2->entries[tid].fetch_data_valid_slots > 0)
    // {
    //     fetchbuffer0[tid] = fetchbuffer_temp;
    // }

    if (fromF1F2->size == 0||
        !fromF1F2->entries[tid].valid) {
        return;
    }
    //after update fetchbuffer valid
    if(fetchbuffer0[tid].fb_valid_slots_num == 0 && fromF1F2->entries[tid].fetch_data_valid_slots > 0)
    {
        fetchbuffer0[tid] = fetchbuffer_temp;
        return;
    }
    if(fetchbuffer1[tid].fb_valid_slots_num == 0 && fromF1F2->entries[tid].fetch_data_valid_slots > 0)
    {
        fetchbuffer1[tid] = fetchbuffer_temp;
        return;
    }
    if(fetchbuffer2[tid].fb_valid_slots_num == 0 && fromF1F2->entries[tid].fetch_data_valid_slots > 0)
    {
        fetchbuffer2[tid] = fetchbuffer_temp;
        return;
    }
    if(fetchbuffer3[tid].fb_valid_slots_num == 0 && fromF1F2->entries[tid].fetch_data_valid_slots > 0)
    {
        fetchbuffer3[tid] = fetchbuffer_temp;
        return;
    }
}

void 
F3Align::get_fetchdata_align(ThreadID tid, FetchDataAlginBlock* fetchdata_align)
{
    int idx = 0;
    //fetchdata_align->fb0_used_slots = 0;
    //fetchdata_align->fb1_used_slots = 0;

    for(int i = 0; i < 4; ++i)
    {
        fetchdata_align->fetchbuffervalid[i] = false;
        fetchdata_align->data2B[i] = false;
    }
    
    fetchdata_align->fetch_algin_buffer_start_addr = fetchbuffer0[tid].fetch_firstslot_pcaddr;
    for(int i = 0; i < 4 && idx < 4; ++i)
    {
        if(fetchbuffer0[tid].fetchbuffervalid[i])
        {
            fetchdata_align->fetchbufferdata[i] = fetchbuffer0[tid].fetchbufferdata[i]; 
            fetchdata_align->fetchbuffervalid[i] = true; 
            fetchdata_align->data2B[i] = (fetchbuffer0[tid].fetchbufferdata[i] & 0x3) != 0x3;
            fetchdata_align->data_fb[i] = 0;
            fetchdata_align->data_fbslot[i] = i;
            //fetchdata_align->fb0_used_slots++;
            idx++;
        }
        else
        {
            break;
        }
    }
    for (int i = 0; i < 4 && idx < 4; ++i) 
    {
        if (fetchbuffer1[tid].fetchbuffervalid[i]) 
        {
            fetchdata_align->fetchbufferdata[idx] = fetchbuffer1[tid].fetchbufferdata[i];
            fetchdata_align->fetchbuffervalid[idx] = true;
            fetchdata_align->data2B[idx] = (fetchbuffer1[tid].fetchbufferdata[i] & 0x3) != 0x3;
            fetchdata_align->data_fb[idx] = 1;
            fetchdata_align->data_fbslot[idx] = i;
            //fetchdata_align->fb1_used_slots++;
            idx++;
        } 
        else 
        {
            break;
        }
    }

}

//update fetchbuffer0 and fetchbuffer1 valid slots number nextcycle
//update fb_consume1 and fb_consume2
void 
F3Align::update_f0f1_valid_slots(bool inst0_2B, bool inst1_2B, bool inst0_valid, bool inst1_valid, ThreadID tid)
{
    uint8_t used_slots = 0;
    if (inst0_valid) {
        used_slots += inst0_2B ? 1 : 2;
    }
    if (inst1_valid) {
        used_slots += inst1_2B ? 1 : 2;
    }

    //uint8_t use_all_alginslots_index = used_slots > 0 ? (used_slots - 1) : 0;
    
    if (used_slots == 0) {
        return;
    }

    if(used_slots >= fetchbuffer0[tid].fb_valid_slots_num)
    {
        fetchbuffer1[tid].fb_valid_slots_num_nextcycle = fetchbuffer1[tid].fb_valid_slots_num - (used_slots - fetchbuffer0[tid].fb_valid_slots_num);
        fetchbuffer0[tid].fb_valid_slots_num_nextcycle = 0;
        if(fetchbuffer1[tid].fb_valid_slots_num_nextcycle == 0)
        {   
            toF1F2Fetch->fb_consume1[tid] = true;
            toF1F2Fetch->fb_consume2[tid] = true;
        }
        else
        {
            toF1F2Fetch->fb_consume1[tid] = true;
            toF1F2Fetch->fb_consume2[tid] = false;
        }
        return;
    }
    else
    {
        fetchbuffer1[tid].fb_valid_slots_num_nextcycle = fetchbuffer1[tid].fb_valid_slots_num;
        fetchbuffer0[tid].fb_valid_slots_num_nextcycle = fetchbuffer0[tid].fb_valid_slots_num - used_slots;
        toF1F2Fetch->fb_consume1[tid] = false;
        toF1F2Fetch->fb_consume2[tid] = false;
        return;
    }
}

void 
F3Align::get_fetchbuffer_update(ThreadID tid)
{
    //hold[tid] = fetchbuffer0[tid].fb_valid_slots_num_nextcycle > 0;
    f1tof0[tid] = fetchbuffer0[tid].fb_valid_slots_num_nextcycle == 0 && fetchbuffer1[tid].fb_valid_slots_num_nextcycle > 0;
    f2tof0[tid] = fetchbuffer0[tid].fb_valid_slots_num_nextcycle == 0 && fetchbuffer1[tid].fb_valid_slots_num_nextcycle == 0 && fetchbuffer2[tid].fb_valid_slots_num_nextcycle > 0;

    f2tof1[tid] = f1tof0[tid] && fetchbuffer2[tid].fb_valid_slots_num_nextcycle > 0;
    f3tof1[tid] = f2tof0[tid] && fetchbuffer3[tid].fb_valid_slots_num_nextcycle > 0;

    f3tof2[tid] = f2tof1[tid] && fetchbuffer3[tid].fb_valid_slots_num_nextcycle > 0;

    fetchtof0[tid] = fetchbuffer0[tid].fb_valid_slots_num_nextcycle == 0 && 
                     fetchbuffer1[tid].fb_valid_slots_num_nextcycle == 0 && 
                     fetchbuffer2[tid].fb_valid_slots_num_nextcycle == 0 && 
                     fetchbuffer3[tid].fb_valid_slots_num_nextcycle == 0;

    fetchtof1[tid] = (f1tof0[tid] && ~f2tof1[tid] && ~f3tof1[tid]) || 
                     (f2tof0[tid] && ~f3tof1[tid]) || 
                     (fetchbuffer0[tid].fb_valid_slots_num_nextcycle > 0 && fetchbuffer1[tid].fb_valid_slots_num_nextcycle == 0);

    fetchtof2[tid] = f3tof1[tid] || 
                    (f2tof1[tid] && ~f3tof2[tid]) || 
                    (fetchbuffer0[tid].fb_valid_slots_num_nextcycle > 0 && fetchbuffer1[tid].fb_valid_slots_num_nextcycle > 0 && fetchbuffer2[tid].fb_valid_slots_num_nextcycle == 0);

    fetchtof3[tid] = f3tof2[tid] || 
                     (fetchbuffer0[tid].fb_valid_slots_num_nextcycle > 0 && fetchbuffer1[tid].fb_valid_slots_num_nextcycle > 0 && fetchbuffer2[tid].fb_valid_slots_num_nextcycle > 0 && fetchbuffer3[tid].fb_valid_slots_num_nextcycle == 0);
}

void 
F3Align::get_instr(ThreadID tid, instruction_block* Instr)
{
    FetchDataAlginBlock fetchdata_align{};
    get_fetchdata_align(tid, &fetchdata_align);

    // Instr->inst0_2B = fetchdata_align->data2B[0];
    // Instr->inst0_valid = ((Instr->inst0_2B) && fetchdata_align->fetchbuffervalid[0]) || ((!Instr->inst0_2B) && fetchdata_align->fetchbuffervalid[1]);

    // Instr->inst1_2B = Instr->inst0_2B ? fetchdata_align->data2B[1] : fetchdata_align->data2B[2];
    // Instr->inst1_valid = Instr->inst0_2B ? (Instr->inst1_2B && fetchdata_align->fetchbuffervalid[1]) || (!Instr->inst1_2B && fetchdata_align->fetchbuffervalid[2]) 
    //                                      : (Instr->inst1_2B && fetchdata_align->fetchbuffervalid[2] || (!Instr->inst1_2B && fetchdata_align->fetchbuffervalid[3]));

    // Instr->inst0 = Instr->inst0_2B ?  static_cast<uint32_t>(fetchdata_align->fetchbufferdata[0]) 
    //                                   : (static_cast<uint32_t>(fetchdata_align->fetchbufferdata[1]) << 16) | static_cast<uint32_t>(fetchdata_align->fetchbufferdata[0]);
    
    // Instr->inst1 = Instr->inst0_2B ? (Instr->inst1_2B? static_cast<uint32_t>(fetchdata_align->fetchbufferdata[1]) 
    //                                                     : (static_cast<uint32_t>(fetchdata_align->fetchbufferdata[2]) << 16) | static_cast<uint32_t>(fetchdata_align->fetchbufferdata[1]))
    //                                   :(Instr->inst1_2B? static_cast<uint32_t>(fetchdata_align->fetchbufferdata[2]) 
    //                                                     : (static_cast<uint32_t>(fetchdata_align->fetchbufferdata[3]) << 16) | static_cast<uint32_t>(fetchdata_align->fetchbufferdata[2]));

    auto packInst = [&](int low, bool is2B) -> uint32_t {
        if (is2B) return static_cast<uint32_t>(fetchdata_align.fetchbufferdata[low]);
        return (static_cast<uint32_t>(fetchdata_align.fetchbufferdata[low + 1]) << 16) |
                static_cast<uint32_t>(fetchdata_align.fetchbufferdata[low]);
    };
    
    const int inst0_idx = 0;
    const bool inst0_2B = fetchdata_align.data2B[inst0_idx];
    const int inst1_idx = inst0_2B ? 1 : 2;
    const bool inst1_2B = fetchdata_align.data2B[inst1_idx];
    
    Instr->inst0_2B = inst0_2B;
    Instr->inst1_2B = inst1_2B;
    
    bool ibuffer_room1_more = 1;
    bool ibuffer_room2_more = 0;

    Instr->inst0_valid = (fetchdata_align.fetchbuffervalid[inst0_idx] &&
                         (inst0_2B || fetchdata_align.fetchbuffervalid[inst0_idx + 1])) && ibuffer_room1_more;
    Instr->inst1_valid = (fetchdata_align.fetchbuffervalid[inst1_idx] &&
                         (inst1_2B || fetchdata_align.fetchbuffervalid[inst1_idx + 1])) && ibuffer_room2_more;

    Instr->inst0_addr = fetchdata_align.fetch_algin_buffer_start_addr;
    Instr->inst1_addr = Instr->inst0_addr + (Instr->inst0_2B ? 2 : 4);
    
    Instr->inst0 = packInst(inst0_idx, inst0_2B);
    Instr->inst1 = packInst(inst1_idx, inst1_2B);

    update_f0f1_valid_slots(Instr->inst0_2B, Instr->inst1_2B, Instr->inst0_valid, Instr->inst1_valid, tid);
    get_fetchbuffer_update(tid);
}


void
F3Align::updatedff()
{
    for(int i = 0; i < 2; i++)
    {
        f1tof0_lastcycle[i] = f1tof0[i];
        f2tof0_lastcycle[i] = f2tof0[i];
        f2tof1_lastcycle[i] = f2tof1[i];
        f3tof1_lastcycle[i] = f3tof1[i];
        f3tof2_lastcycle[i] = f3tof2[i];
        fetchtof0_lastcycle[i] = fetchtof0[i];
        fetchtof1_lastcycle[i] = fetchtof1[i];
        fetchtof2_lastcycle[i] = fetchtof2[i];
        fetchtof3_lastcycle[i] = fetchtof3[i];
    }
}



//gem5-compatible
StaticInstPtr
F3Align::decodeAlignedMachInst(ThreadID tid, uint32_t mach_inst,
        unsigned inst_bytes, PCStateBase &this_pc)
{
    InstDecoder *dec = decoder[tid];
    if (!dec) {
        return nullptr;
    }

    std::memcpy(dec->moreBytesPtr(), &mach_inst, inst_bytes);
    dec->moreBytes(this_pc, this_pc.instAddr());

    if (!dec->instReady()) {
        return nullptr;
    }

    return dec->decode(this_pc);
}

DynInstPtr
F3Align::buildDynInst(ThreadID tid, StaticInstPtr static_inst,
        StaticInstPtr cur_macroop, const PCStateBase &this_pc,
        const PCStateBase &next_pc, bool trace)
{
    InstSeqNum seq = cpu->getAndIncrementInstSeq();

    DynInst::Arrays arrays;
    arrays.numSrcs = static_inst->numSrcRegs();
    arrays.numDests = static_inst->numDestRegs();

    DynInstPtr instruction = new (arrays) DynInst(
            arrays, static_inst, cur_macroop, this_pc, next_pc, seq, cpu);
    instruction->setTid(tid);
    instruction->setThreadState(cpu->thread[tid]);

    DPRINTF(Fetch, "[tid:%i] F3 align: PC %s [sn:%lli].\n",
            tid, this_pc, seq);

#if TRACING_ON
    if (trace) {
        instruction->traceData =
            cpu->getTracer()->getInstRecord(curTick(), cpu->tcBase(tid),
                    static_inst, this_pc, cur_macroop);
    }
#else
    instruction->traceData = nullptr;
#endif

#if TRACING_ON
    if (debug::O3PipeView) {
        instruction->fetchTick = curTick();
    }
#endif

    instruction->setInstListIt(cpu->addInst(instruction));

    return instruction;
}

void
F3Align::predecodeAndMaterializeAlignedInsts(ThreadID tid,
        const instruction_block &ib)
{
    if (!decoder[tid]) {
        return;
    }

    const auto &fetch_entry = fromF1F2->entries[tid];
    if (!fetch_entry.valid) {
        return;
    }

    /** PC for decode/labels: F1/F2 owns the fetch PC. F3 only copies the
     *  bundle PC into local PCState objects used to construct DynInsts. */
    std::unique_ptr<PCStateBase> cur_pc(cpu->pcState(tid).clone());
    cur_pc->set(fetch_entry.fetchAddr);

    auto emit_one = [&](uint32_t word, bool is_2B, bool valid) {
        if (!valid) {
            return;
        }
        if (toDecode->size >= static_cast<int>(fetchWidth) ||
            toDecode->size >= MaxWidth) {
            return;
        }

        const unsigned nbytes = is_2B ? 2U : 4U;
        if (is_2B) {
            word &= 0xffff;
        }

        std::unique_ptr<PCStateBase> decode_pc(cur_pc->clone());
        StaticInstPtr static_inst =
                decodeAlignedMachInst(tid, word, nbytes, *decode_pc);
        if (!static_inst) {
            warn("F3Align: decode failed at PC %#x\n", cur_pc->instAddr());
            return;
        }

        if (static_inst->isMacroop()) {
            warn("F3Align: macro-op at PC %#x not expanded in align\n",
                    cur_pc->instAddr());
            return;
        }

        std::unique_ptr<PCStateBase> next_pc(decode_pc->clone());
        static_inst->advancePC(*next_pc);

        DynInstPtr dyn = buildDynInst(tid, static_inst, nullptr,
                *decode_pc, *next_pc, true);

        toDecode->insts[toDecode->size++] = dyn;

        DPRINTF(Fetch, "[tid:%i] F3 align -> decode slot %i sn:%lli\n",
                tid, toDecode->size - 1, dyn->seqNum);

        set(*cur_pc, *next_pc);
    };

    emit_one(static_cast<uint32_t>(ib.inst0), ib.inst0_2B, ib.inst0_valid);
    emit_one(static_cast<uint32_t>(ib.inst1), ib.inst1_2B, ib.inst1_valid);
}

} // namespace o3
} // namespace gem5
