import m5
from m5.objects import *
from m5.objects import Cache

class L1Cache(Cache):
    assoc = 2
    tag_latency = 2
    data_latency = 2
    response_latency = 2
    mshrs = 4
    tgts_per_mshr = 20

    def __init__(self, options=None):
        super().__init__()
        pass

    def connectCPU(self, cpu):
    # need to define this in a base class!
        raise NotImplementedError

    def connectBus(self, bus):
        self.mem_side = bus.cpu_side_ports

class L1ICache(L1Cache):
    size = '16kB'

    def __init__(self, opts=None):
        super().__init__(opts)
        if not opts or not opts.l1i_size:
            return
        self.size = opts.l1i_size

    def connectCPU(self, cpu):
        """Connect this cache's port to a CPU icache port"""
        self.cpu_side = cpu.icache_port

class L1DCache(L1Cache):
    size = '64kB'

    def __init__(self, opts=None):
        super().__init__(opts)
        if not opts or not opts.l1d_size:
            return
        self.size = opts.l1d_size

    def connectCPU(self, cpu):
        """Connect this cache's port to a CPU dcache port"""
        self.cpu_side = cpu.dcache_port

class L2Cache(Cache):
    size = '256kB'
    assoc = 8
    tag_latency = 20
    data_latency = 20
    response_latency = 20
    mshrs = 20
    tgts_per_mshr = 12

    def __init__(self, opts=None):
        super().__init__()
        if not opts or not opts.l2_size:
            return
        self.size = opts.l2_size

    def connectCPUSideBus(self, bus):
        self.cpu_side = bus.mem_side_ports

    def connectMemSideBus(self, bus):
        self.mem_side = bus.cpu_side_ports


#create system object
system = System()

#create clock
system.clk_domain = SrcClockDomain()
system.clk_domain.clock = '1GHz'

#create voltage domain
system.clk_domain.voltage_domain = VoltageDomain()

#
system.mem_mode = 'timing'
system.mem_ranges = [AddrRange('512MB')]

#create cpu
system.cpu = RiscvO3CPU()

#create cache
system.cpu.icache = L1ICache()
system.cpu.dcache = L1DCache()

system.cpu.icache.connectCPU(system.cpu)
system.cpu.dcache.connectCPU(system.cpu)

system.l2bus = L2XBar()

system.cpu.icache.connectBus(system.l2bus)
system.cpu.dcache.connectBus(system.l2bus)

system.l2cache = L2Cache()
system.l2cache.connectCPUSideBus(system.l2bus)

#create bus
system.membus = SystemXBar()
system.l2cache.connectMemSideBus(system.membus)
#connect cache port to cpu


system.cpu.createInterruptController()
system.system_port = system.membus.cpu_side_ports


#create memory contraller
system.mem_ctrl = MemCtrl()
system.mem_ctrl.dram = DDR3_1600_8x8()
system.mem_ctrl.dram.range = system.mem_ranges[0]       #system.mem_ranges = [
                                                        #                       AddrRange('512MB'),                     # [0]: 0x0000_0000 -> 0x1FFF_FFFF
                                                        #                       AddrRange('1GB', start='0x80000000')    # [1]: 0x8000_0000 -> 0xBFFF_FFFF
                                                        #                     ]
system.mem_ctrl.port = system.membus.mem_side_ports


#set binary file  (c compile)
binary = 'tests/test-progs/hello/bin/riscv/linux/hello'


system.workload = SEWorkload.init_compatible(binary)


process = Process()
process.cmd = [binary]
system.cpu.workload = process
system.cpu.createThreads()

root = Root(full_system = False, system = system)
m5.instantiate()

print("Beginning simulation!")
exit_event = m5.simulate()

print('Exiting @ tick {} because {}'
      .format(m5.curTick(), exit_event.getCause()))