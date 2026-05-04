/*
 * Minimal F1 fetch stage skeleton for gem5-style EH2 IFU.
 */

#ifndef __CPU_O3_F1_FETCH_HH__
#define __CPU_O3_F1_FETCH_HH__

#include <array>
#include <list>
#include <memory>
#include <string>

#include "arch/generic/pcstate.hh"
#include "cpu/o3/comm.hh"
#include "cpu/o3/limits.hh"
#include "cpu/pred/eh2_pred.hh"
#include "cpu/timebuf.hh"
#include "mem/packet.hh"
#include "mem/port.hh"
#include "sim/core.hh"

namespace gem5
{

struct BaseO3CPUParams;

namespace o3
{

class CPU;

class F1F2Fetch
{
  public:
    class IcachePort : public RequestPort
    {
      private:
        F1F2Fetch *fetch;

      public:
        IcachePort(F1F2Fetch *_fetch, CPU *_cpu);

      protected:
        bool recvTimingResp(PacketPtr pkt) override;
        void recvReqRetry() override;
    };

    enum FetchStatus
    {
        Active,
        Inactive
    };

    enum ThreadStatus
    {
        Running,
        Idle,
        Blocked,
        Squashing
    };

    F1F2Fetch(CPU *_cpu, const BaseO3CPUParams &params);

    std::string name() const;

    RequestPort &getInstPort() { return icachePort; }

    void setTimeBuffer(TimeBuffer<TimeStruct> *time_buffer);
    void setActiveThreads(std::list<ThreadID> *at_ptr);
    void setFetchQueue(TimeBuffer<FetchF1F2Struct> *queue_ptr);
    void setFromF3Align(TimeBuffer<F3ToF1F2Struct> *queue_ptr);
    void setFromExuOneCycle(TimeBuffer<ExuStruct> *queue_ptr);

    void startupStage();
    void resetStage();
    void clearStates(ThreadID tid);
    void tick();

    Addr fetchBufferAlignPC(Addr addr) const
    {
        return addr & ~fetchBufferMask;
    }

    void evaluateRedirects(ThreadID tid);
    void selectFetchThread();
    void selectFetchAddress(ThreadID tid);
    void computeNextSequentialAddress(ThreadID tid);
    void applyBranchPredictionRedirection(ThreadID tid);
    void applyFlushRedirection(ThreadID tid);
    void applyMissReplayRedirection(ThreadID tid);
    void qualifyFetchRequest(ThreadID tid);
    void arbitrateFetchRequest(ThreadID tid);
    void updateFetchRequestPipeline(ThreadID tid);
    void updateFetchBufferCredits(ThreadID tid);
    void detectLineWrap(ThreadID tid);
    void evaluateRegionAttributes(ThreadID tid);
    void evaluateFetchReady(ThreadID tid);
    void writeF1ToF2(ThreadID tid);
    void updateStageStatus();

    //eh2 custom
    void updatef0prefetch();
    void updatestates();
    void seladdrf1(ThreadID tid, PCStateBase &pc);
    void sel_addr_bf(ThreadID tid, PCStateBase &pc, PCStateBase &btb_pc,
            PCStateBase &miss_addr_pc);
    void set_addr_f1(ThreadID tid, PCStateBase &addr_f1_pc);
    void update_miss_addr(ThreadID tid, PCStateBase &miss_addr_pc);
    void updatedff();
    bool get_fetch_req_f1_raw(ThreadID tid);
    bool get_fetch_req_f1(ThreadID tid);
    bool get_fetch_req_f2(bool fetch_req_f1_prev, ThreadID tid);
    void updatefetchstates();
    void updateFbFullPoint();
    bool getFbFullState(ThreadID tid);
    ThreadID tidarbiter(bool tid0, bool tid1);
    bool isThreadActive(ThreadID tid) const;

    /** Functionally read 8 bytes at \p pc.instAddr() through the CPU I-port.
     *  Uses a physical Request (INST_FETCH|PHYSICAL): no MMU walk; \p pc's
     *  instruction address must match the physical byte address of the code
     *  (identity map / bare-metal is typical). Writes bytes into \p out for
     *  InstDecoder::moreBytes etc. */
    void fetchEightBytesFunctional(ThreadID tid, const PCStateBase &pc,
                                   uint8_t out[8]);

  private:
    CPU *cpu;
    IcachePort icachePort;

    TimeBuffer<TimeStruct> *timeBuffer;
    TimeBuffer<TimeStruct>::wire fromDecode;
    TimeBuffer<TimeStruct>::wire fromRename;
    TimeBuffer<TimeStruct>::wire fromExu;
    TimeBuffer<TimeStruct>::wire fromCommit;

    TimeBuffer<FetchF1F2Struct> *F1F2ToAlignF3Buffer;
    TimeBuffer<FetchF1F2Struct>::wire toAlignF3;

    TimeBuffer<F3ToF1F2Struct> *AlignF3ToF1F2Buffer;
    TimeBuffer<F3ToF1F2Struct>::wire fromAlignF3;

    TimeBuffer<ExuStruct> *ExuToF1F2BufferOneCycle;
    TimeBuffer<ExuStruct>::wire fromExuOneCycle;
    
    std::unique_ptr<PCStateBase> pc[2];

    std::list<ThreadID> *activeThreads;
    branch_prediction::eh2_pred *branchPred;

    FetchStatus _status;
    std::array<ThreadStatus, MaxThreads> fetchStatus;

    Cycles decodeToFetchDelay;
    Cycles renameToFetchDelay;
    Cycles exuToFetchDelay;
    Cycles commitToFetchDelay;

    unsigned fetchWidth;
    unsigned decodeWidth;
    unsigned fetchBufferSize;
    Addr fetchBufferMask;
    ThreadID numThreads;

    ThreadID selectedThread;
    bool wroteToTimeBuffer;

    std::array<Addr, MaxThreads> fetchAddrBF;
    std::array<Addr, MaxThreads> fetchAddrF1;
    std::array<Addr, MaxThreads> fetchAddrF2;
    std::array<Addr, MaxThreads> missReplayAddr;
    std::array<Addr, MaxThreads> predictedTarget;
    std::array<Addr, MaxThreads> nextFetchAddr;

    std::array<bool, MaxThreads> fetchReqBF;
    std::array<bool, MaxThreads> fetchReqF1;
    std::array<bool, MaxThreads> fetchReqF1Raw;
    std::array<bool, MaxThreads> fetchReqF1Won;
    std::array<bool, MaxThreads> fetchReqF2;
    std::array<bool, MaxThreads> fetchReady;
    std::array<bool, MaxThreads> fetchUncacheable;
    std::array<bool, MaxThreads> iccmAccess;
    std::array<bool, MaxThreads> regionFault;
    std::array<bool, MaxThreads> lineWrap;




    //eh2 custom
    ThreadID tid;
    ThreadID collision_tid = 1;
    ThreadID ifc_select_tid_f1;

    bool fetch_req_f2[2];
    bool fetch_req_f1[2];
    
    //use as dff
    bool fetch_req_f1_lastcycle[2];
    bool fetch_req_f2_lastcycle[2];
    bool fb_consume1_lastcycle[2];
    bool fb_consume2_lastcycle[2];

    bool my_bp_kill_next_f2_lastcycle[2];
    bool flush_fb_lastcycle[2];

    std::unique_ptr<PCStateBase> fetch_addr_f1_lastcycle[2];
    std::unique_ptr<PCStateBase> fetch_addr_bf_lastcycle[2];
    std::unique_ptr<PCStateBase> fetch_addr_bf[2];
    std::unique_ptr<PCStateBase> fetch_addr_f1[2];
    std::unique_ptr<PCStateBase> fetch_addr_f2[2];
    std::unique_ptr<PCStateBase> miss_addr_pc[2];
    std::unique_ptr<PCStateBase> addr_f1_pc_temp[2];

    //std::unique_ptr<PCStateBase> pc[2];

    //fetchStateLastCycle[2];
    
    ThreadID tid_won_lastcycle;

    //tid won arbitration
    ThreadID tid_won;

    //fb count
    int current_fb_count[2] = {0, 0};
    int next_fb_count[2] = {0, 0};


    branch_prediction::eh2_pred *eh2pred;

    enum class FetchState : uint8_t {
      Idle  = 0b00,  
      Fetch = 0b01,  
      Stall = 0b10,  
      WFM   = 0b11   
  };
  FetchState fetchStateLastCycle[2] = {FetchState::Idle, FetchState::Idle};
  FetchState fetchState[2] = {FetchState::Idle, FetchState::Idle};
  FetchState fetchStateNext[2] = {FetchState::Idle, FetchState::Idle};
};

} // namespace o3
} // namespace gem5

#endif // __CPU_O3_F1_FETCH_HH__
