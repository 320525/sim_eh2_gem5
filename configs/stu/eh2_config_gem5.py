import argparse

import m5
from m5.objects import *


def build_system(binary, cpu_clock, mem_size):
    system = System()

    system.clk_domain = SrcClockDomain()
    system.clk_domain.clock = cpu_clock
    system.clk_domain.voltage_domain = VoltageDomain()

    system.mem_mode = "timing"
    system.mem_ranges = [AddrRange(mem_size)]

    system.cpu = RiscvO3CPU(
        numThreads=2,
        fetchWidth=2,
        decodeWidth=2,
        renameWidth=2,
        dispatchWidth=2,
        issueWidth=2,
        wbWidth=2,
        commitWidth=2,
        smtNumFetchingThreads=1,
        smtFetchPolicy="RoundRobin",
        smtIQPolicy="Partitioned",
        smtROBPolicy="Partitioned",
        smtLSQPolicy="Partitioned",
        smtCommitPolicy="RoundRobin",
    )

    system.membus = SystemXBar()

    # F1/F2 uses the CPU I-port for functional instruction reads.
    system.cpu.icache_port = system.membus.cpu_side_ports
    system.cpu.dcache_port = system.membus.cpu_side_ports

    system.cpu.createInterruptController()
    system.system_port = system.membus.cpu_side_ports

    system.mem_ctrl = MemCtrl()
    system.mem_ctrl.dram = DDR3_1600_8x8()
    system.mem_ctrl.dram.range = system.mem_ranges[0]
    system.mem_ctrl.port = system.membus.mem_side_ports

    system.workload = SEWorkload.init_compatible(binary)

    process = Process()
    process.cmd = [binary]

    # Only one workload is supplied, so O3 activates tid0 while still building
    # the CPU with two hardware thread contexts.
    system.cpu.workload = [process]
    system.cpu.createThreads()

    return system


def main():
    parser = argparse.ArgumentParser(
        description="Run a dual-issue RISC-V O3 CPU with two hardware threads "
        "configured and only tid0 active."
    )
    parser.add_argument(
        "--binary",
        default="tests/test-progs/hello/bin/riscv/linux/hello",
        help="RISC-V SE binary to execute on tid0.",
    )
    parser.add_argument("--cpu-clock", default="1GHz")
    parser.add_argument("--mem-size", default="512MB")
    args = parser.parse_args()

    system = build_system(args.binary, args.cpu_clock, args.mem_size)
    root = Root(full_system=False, system=system)

    m5.instantiate()

    print("Beginning dual-issue SE simulation with numThreads=2, active tid0")
    exit_event = m5.simulate()

    print(
        "Exiting @ tick {} because {}".format(
            m5.curTick(), exit_event.getCause()
        )
    )


if __name__ == "__m5_main__":
    main()
