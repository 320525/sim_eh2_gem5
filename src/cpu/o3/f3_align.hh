/*
 * Minimal F3 align stage skeleton for gem5-style EH2 IFU.
 */

#ifndef __CPU_O3_F3_ALIGN_HH__
#define __CPU_O3_F3_ALIGN_HH__

#include <array>
#include <cstdint>
#include <deque>
#include <list>
#include <string>

#include "arch/generic/decoder.hh"
#include "arch/generic/pcstate.hh"
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

class F3Align
{
  public:
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

    F3Align(CPU *_cpu, const BaseO3CPUParams &params);

    std::string name() const;

    void setTimeBuffer(TimeBuffer<TimeStruct> *time_buffer);
    void setActiveThreads(std::list<ThreadID> *at_ptr);
    /** F1/F2 align bundle (raw bytes + valid slots) from \ref FetchF1F2Struct. */
    void setF1F2Buffer(TimeBuffer<FetchF1F2Struct> *queue_ptr);
    void setFetchQueue(TimeBuffer<AlignF3Struct> *queue_ptr);
    void setToF1F2FetchQueue(TimeBuffer<F3ToF1F2Struct> *queue_ptr);

    void startupStage();
    void resetStage();
    void clearStates(ThreadID tid);
    void tick();
    InstDecoder *getDecoderPtr(ThreadID tid) { return decoder[tid]; }

  private:
    void updatedff();
    CPU *cpu;

    TimeBuffer<TimeStruct> *timeBuffer;
    TimeBuffer<TimeStruct>::wire fromDecode;
    TimeBuffer<TimeStruct>::wire fromRename;
    TimeBuffer<TimeStruct>::wire fromIEW;
    TimeBuffer<TimeStruct>::wire fromCommit;

    TimeBuffer<FetchF1F2Struct> *f1f2ToF3Buffer;
    TimeBuffer<FetchF1F2Struct>::wire fromF1F2;

    TimeBuffer<AlignF3Struct> *fetchQueueBuffer;
    TimeBuffer<AlignF3Struct>::wire toDecode;

    TimeBuffer<F3ToF1F2Struct> *toF1F2FetchBuffer;
    TimeBuffer<F3ToF1F2Struct>::wire toF1F2Fetch;

    std::list<ThreadID> *activeThreads;

    FetchStatus _status;
    std::array<ThreadStatus, MaxThreads> fetchStatus;

    unsigned fetchWidth;
    unsigned decodeWidth;
    unsigned fetchBufferSize;
    ThreadID numThreads;

    std::array<InstDecoder *, MaxThreads> decoder;
    std::array<std::deque<FetchF1F2Struct::Entry>, MaxThreads> pendingLines;

    struct FetchDataBlock
    {
        std::array<uint16_t, 4> fetchbufferdata{};
        std::array<bool, 4> fetchbuffervalid{};
        uint8_t fb_valid_slots_num;
        uint8_t fb_valid_slots_num_nextcycle;
    };

    struct FetchDataAlginBlock
    {
        std::array<uint16_t, 4> fetchbufferdata{};
        std::array<bool, 4> fetchbuffervalid{};
        std::array<bool, 4> data2B{};
        std::array<bool, 4> data_fb{};
        std::array<uint8_t, 4> data_fbslot{};
    };

    struct instruction_block
    {
        int inst0;
        int inst1;
        bool inst0_valid;
        bool inst1_valid;
        bool inst0_2B;
        bool inst1_2B;
    };

    std::array<FetchDataBlock, 2> fetchbuffer0{};
    std::array<FetchDataBlock, 2> fetchbuffer1{};
    std::array<FetchDataBlock, 2> fetchbuffer2{};
    std::array<FetchDataBlock, 2> fetchbuffer3{};

    void fetchbufferslots_shift(FetchDataBlock *fetchbuffer);
    void fetchbuffer_shift(uint8_t shift, ThreadID tid);
    void update_fetchbuffer(ThreadID tid);
    void get_fetchdata_align(ThreadID tid, FetchDataAlginBlock *fetchdata_align);
    void update_f0f1_valid_slots(bool inst0_2B, bool inst1_2B, bool inst0_valid,
            bool inst1_valid, ThreadID tid);
    void get_fetchbuffer_update(ThreadID tid);
    void get_instr(ThreadID tid, instruction_block *Instr);

    std::array<bool, 2> hold{};

    DynInstPtr buildDynInst(ThreadID tid, StaticInstPtr static_inst,
            StaticInstPtr cur_macroop, const PCStateBase &this_pc,
            const PCStateBase &next_pc, bool trace);
    StaticInstPtr decodeAlignedMachInst(ThreadID tid, uint32_t mach_inst,
            unsigned inst_bytes, PCStateBase &this_pc);

    void predecodeAndMaterializeAlignedInsts(ThreadID tid,
            const instruction_block &ib);

    // use as dff
    std::array<bool, 2> f1tof0{};
    std::array<bool, 2> f2tof0{};
    std::array<bool, 2> f2tof1{};
    std::array<bool, 2> f3tof1{};
    std::array<bool, 2> f3tof2{};
    std::array<bool, 2> fetchtof0{};
    std::array<bool, 2> fetchtof1{};
    std::array<bool, 2> fetchtof2{};
    std::array<bool, 2> fetchtof3{};

    std::array<bool, 2> f1tof0_lastcycle{};
    std::array<bool, 2> f2tof0_lastcycle{};
    std::array<bool, 2> f2tof1_lastcycle{};
    std::array<bool, 2> f3tof1_lastcycle{};
    std::array<bool, 2> f3tof2_lastcycle{};
    std::array<bool, 2> fetchtof0_lastcycle{};
    std::array<bool, 2> fetchtof1_lastcycle{};
    std::array<bool, 2> fetchtof2_lastcycle{};
    std::array<bool, 2> fetchtof3_lastcycle{};
};

} // namespace o3
} // namespace gem5

#endif // __CPU_O3_F3_ALIGN_HH__
