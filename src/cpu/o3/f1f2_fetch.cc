#include "f1f2_fetch.hh"

#include <algorithm>

#include "base/logging.hh"
#include "cpu/o3/cpu.hh"
#include "mem/packet.hh"
#include "mem/port.hh"
#include "mem/request.hh"
#include "params/BaseO3CPU.hh"
#include "sim/core.hh"
#include "sim/system.hh"

namespace gem5
{

namespace o3
{

F1F2Fetch::IcachePort::IcachePort(F1F2Fetch *_fetch, CPU *_cpu)
    : RequestPort(_cpu->name() + ".icache_port"),
      fetch(_fetch)
{}

bool
F1F2Fetch::IcachePort::recvTimingResp(PacketPtr pkt)
{
    panic("F1F2Fetch functional fetch does not expect timing responses");
    return true;
}

void
F1F2Fetch::IcachePort::recvReqRetry()
{
    panic("F1F2Fetch functional fetch does not expect timing retries");
}

F1F2Fetch::F1F2Fetch(CPU *_cpu, const BaseO3CPUParams &params)
    : cpu(_cpu),
      icachePort(this, _cpu),
    //   timeBuffer(nullptr),
    //   F1F2ToAlignF3Buffer(nullptr),
    //   AlignF3ToF1F2Buffer(nullptr),
    //   activeThreads(nullptr),
      //branchPred(params.branchPred),
      eh2pred(params.branchPred),
      _status(Inactive),
      decodeToFetchDelay(params.decodeToFetchDelay),
      renameToFetchDelay(params.renameToFetchDelay),
      exuToFetchDelay(params.iewToFetchDelay),
      commitToFetchDelay(params.commitToFetchDelay),
      fetchWidth(params.fetchWidth),
      decodeWidth(params.decodeWidth),
      fetchBufferSize(params.fetchBufferSize),
      fetchBufferMask(params.fetchBufferSize - 1),
      numThreads(params.numThreads),
      selectedThread(InvalidThreadID),
      wroteToTimeBuffer(false)
{
    resetStage();
}

std::string
F1F2Fetch::name() const
{
    return cpu->name() + ".f1f2_fetch";
}

void
F1F2Fetch::setTimeBuffer(TimeBuffer<TimeStruct> *time_buffer)
{
    timeBuffer = time_buffer;
    fromDecode = timeBuffer->getWire(-decodeToFetchDelay);
    fromRename = timeBuffer->getWire(-renameToFetchDelay);
    fromExu = timeBuffer->getWire(-exuToFetchDelay);
    fromCommit = timeBuffer->getWire(-commitToFetchDelay);
}

void
F1F2Fetch::setActiveThreads(std::list<ThreadID> *at_ptr)
{
    activeThreads = at_ptr;
}

void
F1F2Fetch::setFetchQueue(TimeBuffer<FetchF1F2Struct> *queue_ptr)
{
    F1F2ToAlignF3Buffer = queue_ptr;
    toAlignF3 = F1F2ToAlignF3Buffer->getWire(0);
}

void
F1F2Fetch::setFromF3Align(TimeBuffer<F3ToF1F2Struct> *queue_ptr)
{
    AlignF3ToF1F2Buffer = queue_ptr;
    fromAlignF3 = AlignF3ToF1F2Buffer->getWire(-1);
}

void
F1F2Fetch::setFromExuOneCycle(TimeBuffer<ExuStruct> *queue_ptr)
{
    ExuToF1F2BufferOneCycle = queue_ptr;
    fromExuOneCycle = ExuToF1F2BufferOneCycle->getWire(-1);
}

void
F1F2Fetch::startupStage()
{
    resetStage();
    for (ThreadID tid = 0; tid < 2; ++tid) {
        const PCStateBase &start_pc = cpu->pcState(tid);
        set(fetch_addr_bf_lastcycle[tid], start_pc);
        set(fetch_addr_f1_lastcycle[tid], start_pc);
        set(fetch_addr_bf[tid], start_pc);
        set(fetch_addr_f1[tid], start_pc);
        set(fetch_addr_f2[tid], start_pc);
        set(miss_addr_pc[tid], start_pc);
        set(addr_f1_pc_temp[tid], start_pc);
    }
    _status = Active;
}

void
F1F2Fetch::resetStage()
{
    _status = Inactive;
    selectedThread = InvalidThreadID;
    wroteToTimeBuffer = false;

    for (ThreadID tid = 0; tid < 2; ++tid) {
        fetchStatus[tid] = Idle;
        fetchAddrBF[tid] = 0;
        fetchAddrF1[tid] = 0;
        fetchAddrF2[tid] = 0;
        missReplayAddr[tid] = 0;
        predictedTarget[tid] = 0;
        nextFetchAddr[tid] = 0;
        fetchReqBF[tid] = false;
        fetchReqF1[tid] = false;
        fetchReqF1Raw[tid] = false;
        fetchReqF1Won[tid] = false;
        fetchReqF2[tid] = false;
        fetchReady[tid] = false;
        fetchUncacheable[tid] = false;
        iccmAccess[tid] = false;
        regionFault[tid] = false;
        lineWrap[tid] = false;
        fetch_req_f1_lastcycle[tid] = false;
        fetch_req_f2_lastcycle[tid] = false;
        fb_consume1_lastcycle[tid] = false;
        fb_consume2_lastcycle[tid] = false;
        my_bp_kill_next_f2_lastcycle[tid] = false;
        flush_fb_lastcycle[tid] = false;
        current_fb_count[tid] = 0;
        next_fb_count[tid] = 0;
        fetchStateLastCycle[tid] = FetchState::Idle;
        fetchState[tid] = FetchState::Idle;
        fetchStateNext[tid] = FetchState::Idle;
    }

    //flush 修改fetch状态 暂时先直接fetch
    for (ThreadID tid = 0; tid < numThreads; ++tid) {
        fetchStatus[tid] = Running;
        fetchState[tid] = FetchState::Fetch;
        fetchStateNext[tid] = FetchState::Fetch;
    }
}

void
F1F2Fetch::clearStates(ThreadID tid)
{
    fetchStatus[tid] = Running;
    fetchAddrBF[tid] = 0;
    fetchAddrF1[tid] = 0;
    fetchAddrF2[tid] = 0;
    missReplayAddr[tid] = 0;
    predictedTarget[tid] = 0;
    nextFetchAddr[tid] = 0;
    fetchReqBF[tid] = false;
    fetchReqF1[tid] = false;
    fetchReqF1Raw[tid] = false;
    fetchReqF1Won[tid] = false;
    fetchReqF2[tid] = false;
    fetchReady[tid] = false;
    fetchUncacheable[tid] = false;
    iccmAccess[tid] = false;
    regionFault[tid] = false;
    lineWrap[tid] = false;
    fetch_req_f1_lastcycle[tid] = false;
    fetch_req_f2_lastcycle[tid] = false;
    fb_consume1_lastcycle[tid] = false;
    fb_consume2_lastcycle[tid] = false;
    my_bp_kill_next_f2_lastcycle[tid] = false;
    flush_fb_lastcycle[tid] = false;
    current_fb_count[tid] = 0;
    next_fb_count[tid] = 0;
    fetchStateLastCycle[tid] = FetchState::Idle;
    fetchState[tid] = FetchState::Fetch;
    fetchStateNext[tid] = FetchState::Fetch;

    const PCStateBase &start_pc = cpu->pcState(tid);
    set(fetch_addr_bf_lastcycle[tid], start_pc);
    set(fetch_addr_f1_lastcycle[tid], start_pc);
    set(fetch_addr_bf[tid], start_pc);
    set(fetch_addr_f1[tid], start_pc);
    set(fetch_addr_f2[tid], start_pc);
    set(miss_addr_pc[tid], start_pc);
    set(addr_f1_pc_temp[tid], start_pc);
}



void
F1F2Fetch::tick()
{
    fetch_req_f1[0] = get_fetch_req_f1(0);
    fetch_req_f1[1] = get_fetch_req_f1(1);
    fetch_req_f2[0] = get_fetch_req_f2(fetch_req_f1_lastcycle[0], 0);
    fetch_req_f2[1] = get_fetch_req_f2(fetch_req_f1_lastcycle[1], 1);
    
    //tid_won = tidarbiter(fetch_req_f1[0], fetch_req_f1[1]);   
    tid_won = 0;    
    
    // if (F1F2ToAlignF3Buffer) {
    //     for (int t = 0; t < 2; ++t) {
    //         F1F2ToAlignF3Buffer->entries[t].valid = false;
    //     }
    // }

    updatefetchstates();
    updateFbFullPoint();

    std::unique_ptr<PCStateBase> btb_pc;
    set(btb_pc, *fetch_addr_f1_lastcycle[tid_won]);
    uint8_t eh2_pred_sel_first = eh2pred -> predict(*btb_pc, tid_won, fetch_addr_f1_lastcycle[tid_won]->instAddr(), fetch_req_f2[tid_won]);
    bool target_taken = eh2_pred_sel_first != 4;
    
    for(int i = 0; i < 2; i++)
    {
        bool flush_fb = fromExuOneCycle -> exu_flush_final[i];
        bool my_bp_kill_next_f2 = eh2pred -> get_ifu_bp_kill_next_f2() && fetch_req_f1[i];
        if(flush_fb || my_bp_kill_next_f2 || (fetch_req_f2[i] && i != tid_won))
        {
            update_miss_addr(i, *miss_addr_pc[i]);
        }
    }

    for(int i = 0; i < 2; i++)
    {
        set_addr_f1(i, *fetch_addr_f1[i]);
    }

    for(int i = 0; i < 2; i++)
    {
        sel_addr_bf(i, *fetch_addr_bf[i], *btb_pc, *miss_addr_pc[i]);
    }

    if(fetch_req_f1[tid_won])
    {
        set(fetch_addr_f2[tid_won], *fetch_addr_f1[tid_won]);
    }
    
    toAlignF3->size = 0;
    for (ThreadID tid = 0; tid < numThreads; ++tid) {
        toAlignF3->entries[tid].valid = false;
    }

    if(fetch_req_f1_lastcycle[tid_won])
    {
        auto &ent = toAlignF3->entries[tid_won];
        fetchEightBytesFunctional(tid_won, *fetch_addr_f1_lastcycle[tid_won],
                ent.data);

        const Addr line_pc = fetch_addr_f1_lastcycle[tid_won]->instAddr();
        ent.valid = true;
        ent.tid = tid_won;
        ent.startPC = line_pc;
        ent.fetchAddr = line_pc;
        ent.nextFetchAddr = line_pc + 8;
        ent.blockAddr = fetchBufferAlignPC(line_pc);
        ent.tick = curTick();

        ent.fetch_br_ret = eh2pred -> fetch_br_ret;
        ent.fetch_br_pc4 = eh2pred -> fetch_br_pc4;
        ent.fetch_br_way = eh2pred -> fetch_br_way;
        ent.fetch_br_end = eh2pred -> fetch_br_end;
        ent.fetch_br_taken = eh2pred -> fetch_br_taken;
        ent.fetch_br_counter = eh2pred -> fetch_br_counter;

        toAlignF3->tid_won = tid_won;
        toAlignF3->size = 1;

        if(target_taken)
        {
            ent.fetch_data_valid_slots = eh2_pred_sel_first + 1;
        }
        else
        {
            ent.fetch_data_valid_slots = 4;
        }
    }
    //update deff at last of this tick
    updatedff();
}

//pc is pointer
void
F1F2Fetch::sel_addr_bf(ThreadID tid, PCStateBase &pc, PCStateBase &btb_pc, PCStateBase &miss_addr_pc)
{
    //assert(tid < numThreads);

    // bool miss_sel_flush = flush_fb_lastcycle[tid] && (tid != tid_won_lastcycle || fetchStateLastCycle[tid] == FetchState::Idle);
    // bool sel_last_addr = !(fetch_req_f1_lastcycle[tid] && (tid_won_lastcycle == tid)) && fetch_req_f2_lastcycle[tid] && !my_bp_kill_next_f2_lastcycle[tid];
    // bool sel_miss_addr = !my_bp_kill_next_f2_lastcycle[tid] && !(fetch_req_f1_lastcycle[tid] && (tid_won_lastcycle == tid)) && !fetch_req_f2_lastcycle[tid];
    // bool sel_btb_addr = my_bp_kill_next_f2_lastcycle[tid];
    // bool sel_next_addr = fetch_req_f1_lastcycle[tid] && (tid_won_lastcycle == tid);
    
    bool my_bp_kill_next_f2 = eh2pred -> get_ifu_bp_kill_next_f2() && fetch_req_f1[tid];
    bool flush_fb = fromExuOneCycle -> exu_flush_final[tid];
    
    bool miss_sel_flush = flush_fb && (tid != tid_won || fetchState[tid] == FetchState::Idle);
    bool sel_last_addr = !(fetch_req_f1[tid] && (tid_won == tid)) && fetch_req_f2[tid] && !my_bp_kill_next_f2;
    bool sel_miss_addr = !my_bp_kill_next_f2 && !(fetch_req_f1[tid] && (tid_won == tid)) && !fetch_req_f2[tid];
    bool sel_btb_addr = my_bp_kill_next_f2;
    bool sel_next_addr = fetch_req_f1[tid] && (tid_won == tid);

    const PCStateBase &base_pc = *fetch_addr_f1[tid];

    if(miss_sel_flush)
    {
        const auto &flush_pc = fromExuOneCycle->exu_flush_path_final[tid];

        set(pc, *flush_pc);
        return;
    }

    if(sel_last_addr)
    {
        set(pc, base_pc);
        return;
    }

    if(sel_miss_addr)
    {
        set(pc, miss_addr_pc);
        return;
    }
    
    if(sel_btb_addr)
    {
        set(pc, btb_pc);
        return;
    }

    if(sel_next_addr)
    {
        Addr next_addr = base_pc.instAddr() + 8;
        set(pc, base_pc);
        pc.set(next_addr);
        return;
    }
}


void
F1F2Fetch::set_addr_f1(ThreadID tid, PCStateBase &addr_f1_pc)
{
    if(fromExuOneCycle -> exu_flush_final[tid])
    {
        set(addr_f1_pc, *fromExuOneCycle->exu_flush_path_final[tid]);
    }
    else
    {
        if(fetchState[tid] == FetchState::Fetch)
        {
            set(addr_f1_pc_temp[tid], *fetch_addr_bf_lastcycle[tid]);
        }
        set(addr_f1_pc, *addr_f1_pc_temp[tid]);
    }
}


void
F1F2Fetch::update_miss_addr(ThreadID tid, PCStateBase &miss_addr_pc)
{
    bool miss_sel_flush = flush_fb_lastcycle[tid] && (tid != tid_won_lastcycle || fetchStateLastCycle[tid] == FetchState::Idle);
    bool miss_sel_f2 = 0;
    bool miss_sel_f1 = !flush_fb_lastcycle[tid] && !my_bp_kill_next_f2_lastcycle[tid] && fetch_req_f2_lastcycle[tid] && !(fetch_req_f1_lastcycle[tid] && (tid_won_lastcycle == tid));
    bool miss_sel_bf = !miss_sel_f1 && !miss_sel_flush;

    if(miss_sel_flush)
    {
        const auto &flush_pc = fromExuOneCycle->exu_flush_path_final_lastcycle[tid];
        set(miss_addr_pc, *flush_pc);
        return;
    }

    if(miss_sel_f1)
    {
        set(miss_addr_pc, *fetch_addr_f1_lastcycle[tid]);
        return;
    }

    if(miss_sel_bf)
    {
        set(miss_addr_pc, *fetch_addr_bf_lastcycle[tid]);
        return;
    }
    
}

void
F1F2Fetch::updatedff()
{
        
    for(int i = 0; i < 2; i++)
    {
        fetch_req_f1_lastcycle[i] = fetch_req_f1[i];
        fetch_req_f2_lastcycle[i] = fetch_req_f2[i];
        fb_consume1_lastcycle[i] = fromAlignF3 -> fb_consume1[i];
        fb_consume2_lastcycle[i] = fromAlignF3 -> fb_consume2[i];
        flush_fb_lastcycle[i] = fromExuOneCycle -> exu_flush_final[i];
        my_bp_kill_next_f2_lastcycle[i] = eh2pred -> get_ifu_bp_kill_next_f2() && fetch_req_f1_lastcycle[i];
        
        set(fetch_addr_bf_lastcycle[i], *fetch_addr_bf[i]);
        set(fetch_addr_f1_lastcycle[i], *fetch_addr_f1[i]);
    }

    tid_won_lastcycle = tid_won;
}

bool
F1F2Fetch::get_fetch_req_f1_raw(ThreadID tid)
{
    return isThreadActive(tid) && fetchState[tid] == FetchState::Fetch;
}

//return fetch_req_f1  indicating whether the fetch request is valid(or whether the fetch request can be send)
bool
F1F2Fetch::get_fetch_req_f1(ThreadID tid)
{
    bool fb_full_f1 = 0;
    bool flush_fb = 0;
    bool flush_noredir = 0;
    bool my_bp_kill_next_f2 = 0;
    bool fetch_req_f1_raw = 0;

    fb_full_f1 = getFbFullState(tid);
    flush_fb = fromExuOneCycle -> exu_flush_final[tid];
    flush_noredir = fromExuOneCycle -> dec_tlu_flush_noredir_wb[tid];
    my_bp_kill_next_f2 = eh2pred -> get_ifu_bp_kill_next_f2() && fetch_req_f1_lastcycle[tid];
    fetch_req_f1_raw = get_fetch_req_f1_raw(tid);

    return fetch_req_f1_raw && !my_bp_kill_next_f2 && !flush_noredir && !(fb_full_f1 && !flush_fb);
}

//return fetch_req_f2  indicating fetch_req_f1 won the arbitration and the fetch request is valid
bool
F1F2Fetch::get_fetch_req_f2(bool fetch_req_f1_prev, ThreadID tid)
{
    bool flush_fb = fromExuOneCycle -> exu_flush_final[tid];
    return isThreadActive(tid) && fetch_req_f1_prev && !flush_fb;
}

bool
F1F2Fetch::isThreadActive(ThreadID tid) const
{
    return activeThreads &&
        std::find(activeThreads->begin(), activeThreads->end(), tid) !=
            activeThreads->end();
}


//update fetch state machine
//update at the start of this tick
void
F1F2Fetch::updatefetchstates()
{
    bool goto_idle[2] = {};
    bool leave_idle[2] = {};

    for(int i = 0; i < 2; i++)
    {
        //dff
        fetchStateLastCycle[i] = fetchState[i];
        fetchState[i] = fetchStateNext[i];
        
        //next state
        //goto idle
        goto_idle[i] = fromExuOneCycle -> exu_flush_final[i] && fromExuOneCycle -> dec_tlu_flush_noredir_wb[i];
        if(goto_idle[i])
        {
            fetchStateNext[i] = FetchState::Idle;
        }

        //goto fetch
        leave_idle[i] = fromExuOneCycle -> exu_flush_final[i] && !fromExuOneCycle -> dec_tlu_flush_noredir_wb[i];
        if(fetchState[i] == FetchState::Idle && leave_idle[i])
        {
            fetchStateNext[i] = FetchState::Fetch;
        }

    }

}

//update fetch buffer pointer state
void
F1F2Fetch::updateFbFullPoint()
{
    for(int i = 0; i < 2; i++)
    {
    if (flush_fb_lastcycle[i]) {
        current_fb_count[i] = (fetch_req_f1_lastcycle[i] && tid_won_lastcycle == i) ? 1 : 0;
    } else {
        int consumed = 0;
        if (fromAlignF3 -> fb_consume2[i]) {
            consumed = 2;
        } else if (fromAlignF3 -> fb_consume1[i]) {
            consumed = 1;
        }
        int pushed = (fetch_req_f1_lastcycle[i] && tid_won_lastcycle == i) ? 1 : 0;
        current_fb_count[i] = current_fb_count[i] - consumed + pushed;
    }

        if (current_fb_count[i] < 0) current_fb_count[i] = 0;
        if (current_fb_count[i] > 4) current_fb_count[i] = 4;
    }
}

//return fb_full_f1
bool 
F1F2Fetch::getFbFullState(ThreadID tid)
{
    return current_fb_count[tid] == 4;
}


ThreadID
F1F2Fetch::tidarbiter(bool tid0, bool tid1)
{
    ThreadID temp_tid = 0;
    if(tid0 && tid1)
    {
        collision_tid = (collision_tid + 1) & 0x1;
        return collision_tid;
    }
    
    if(tid0 && !tid1)
    {
        temp_tid = 0;
    }
    
    if(!tid0 && tid1)
    {
        temp_tid = 1;
    }
    return temp_tid;

}



//gem5 fetch data interface
void
F1F2Fetch::fetchEightBytesFunctional(ThreadID tid, const PCStateBase &pc,
                                     uint8_t out[8])
{
    assert(!cpu->switchedOut());
    assert(tid < numThreads);

    const Addr fetch_addr = pc.instAddr();

    if (!cpu->system->isMemAddr(fetch_addr)) {
        panic("fetchEightBytesFunctional: addr %#llx outside physical memory",
              fetch_addr);
    }


    RequestPtr req = std::make_shared<Request>(
        fetch_addr, 8,
        Request::INST_FETCH | Request::PHYSICAL,
        cpu->instRequestorId());
    req->setContext(cpu->thread[tid]->contextId());
    req->setPC(fetch_addr);
    req->taskId(cpu->taskId());

    PacketPtr pkt = new Packet(req, MemCmd::ReadReq);
    pkt->dataStatic(out);

    if (!icachePort.isConnected()) {
        panic("fetchEightBytesFunctional: instruction port is not connected");
    }
    icachePort.sendFunctional(pkt);

    delete pkt;
}


} // namespace o3
} // namespace gem5
