// See LICENSE.SiFive for license details.

package freechips.rocketchip.subsystem

import chisel3._

import freechips.rocketchip.config.Field
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.tilelink._
import freechips.rocketchip.amba.axi4._
import freechips.rocketchip.util._
import os.perms
import os.read

/** Specifies the size and width of external memory ports */
case class MasterPortParams(
  base: BigInt,
  size: BigInt,
  beatBytes: Int,
  idBits: Int,
  maxXferBytes: Int = 256,
  executable: Boolean = true,
  interleave: Boolean = false, // interleave transactions across memory channels, currently we set default to false
  remoteSize: BigInt = 0, // remote memory size. The remaining parts are local memory
  nRemoteChannels: Int = 0
)

/** Specifies the width of external slave ports */
case class SlavePortParams(beatBytes: Int, idBits: Int, sourceBits: Int)
// if incohBase is set, creates an incoherent alias for the region that hangs off the sbus
case class MemoryPortParams(master: MasterPortParams, nMemoryChannels: Int, incohBase: Option[BigInt] = None)

case object ExtMem extends Field[Option[MemoryPortParams]](None)
case object ExtBus extends Field[Option[MasterPortParams]](None)
case object ExtIn extends Field[Option[SlavePortParams]](None)

///// The following traits add ports to the sytem, in some cases converting to different interconnect standards

/** Adds a port to the system intended to master an AXI4 DRAM controller. */
trait CanHaveMasterAXI4MemPort { this: BaseSubsystem =>
  private val memPortParamsOpt = p(ExtMem)
  private val portName = "axi4"
  private val device = new MemoryDevice
  private val idBits = memPortParamsOpt.map(_.master.idBits).getOrElse(1)

  val memAXI4Node = AXI4SlaveNode(memPortParamsOpt.map({ case MemoryPortParams(memPortParams, nMemoryChannels, _) =>
    Seq.tabulate(nMemoryChannels) { channel =>
      val base = AddressSet.misaligned(memPortParams.base, memPortParams.size) // all ext. memory size, eg. 32GB
      var filter = Seq(AddressSet(channel * mbus.blockBytes, ~((nMemoryChannels-1) * mbus.blockBytes)))
      val nLocalChannels = nMemoryChannels - memPortParams.nRemoteChannels
      assert(nLocalChannels > 0, "Number of local channels must be greater than 0")
      if (!memPortParams.interleave) {
        if(channel < nLocalChannels){
          val localMemSize = memPortParams.size - memPortParams.remoteSize
          assert(localMemSize >= 0, "Local memory size must be greater than or equal to 0")
          val perChannelSize = localMemSize / nLocalChannels
          assert(localMemSize % nLocalChannels == 0, "Local memory size must be divisible by number of local channels")
          filter = AddressSet.misaligned(memPortParams.base + channel * perChannelSize, perChannelSize)
        }
        else{
          val perChannelSize = memPortParams.remoteSize / memPortParams.nRemoteChannels
          assert(memPortParams.remoteSize % memPortParams.nRemoteChannels == 0, "Remote memory size must be divisible by number of remote channels")
          val localMemSize = memPortParams.size - memPortParams.remoteSize
          filter = AddressSet.misaligned(memPortParams.base + localMemSize + (channel - nLocalChannels) * perChannelSize, perChannelSize)
        }
      }
      AXI4SlavePortParameters(
        slaves = Seq(AXI4SlaveParameters(
          // address       = base.flatMap(_.intersect(filter)),
          address = base.flatMap(x => filter.flatMap(y => x.intersect(y))),
          resources     = device.reg,
          regionType    = RegionType.UNCACHED, // cacheable
          executable    = true,
          supportsWrite = TransferSizes(1, mbus.blockBytes),
          supportsRead  = TransferSizes(1, mbus.blockBytes),
          interleavedId = Some(0))), // slave does not interleave read responses
        beatBytes = memPortParams.beatBytes)
    }
  }).toList.flatten)

  for (i <- 0 until memAXI4Node.portParams.size) {
    val mem_bypass_xbar = mbus { TLXbar() }

    // Create an incoherent alias for the AXI4 memory
    memPortParamsOpt.foreach(memPortParams => {
      memPortParams.incohBase.foreach(incohBase => {
        val cohRegion = AddressSet(0, incohBase-1)
        val incohRegion = AddressSet(incohBase, incohBase-1)
        val replicator = sbus {
          val replicator = LazyModule(new RegionReplicator(ReplicatedRegion(cohRegion, cohRegion.widen(incohBase))))
          val prefixSource = BundleBridgeSource[UInt](() => UInt(1.W))
          replicator.prefix := prefixSource
          // prefix is unused for TL uncached, so this is ok
          InModuleBody { prefixSource.bundle := 0.U(1.W) }
          replicator
        }
        sbus.coupleTo(s"memory_controller_bypass_port_named_$portName") {
          (mbus.crossIn(mem_bypass_xbar)(ValName("bus_xing"))(p(SbusToMbusXTypeKey))
            := TLWidthWidget(sbus.beatBytes)
            := replicator.node
            := TLFilter(TLFilter.mSubtract(cohRegion))
            := TLFilter(TLFilter.mResourceRemover)
            := _
          )
        }
      })
    })

    mbus.coupleTo(s"memory_controller_port_named_$portName") {
      (memAXI4Node
        := AXI4UserYanker()
        := AXI4IdIndexer(idBits)
        := TLToAXI4()
        := TLWidthWidget(mbus.beatBytes)
        := mem_bypass_xbar
        := _
      )
    }
  }

  val mem_axi4 = InModuleBody { memAXI4Node.makeIOs() }
}

/** Adds a AXI4 port to the system intended to master an MMIO device bus */
trait CanHaveMasterAXI4MMIOPort { this: BaseSubsystem =>
  private val mmioPortParamsOpt = p(ExtBus)
  private val portName = "mmio_port_axi4"
  private val device = new SimpleBus(portName.kebab, Nil)

  val mmioAXI4Node = AXI4SlaveNode(
    mmioPortParamsOpt.map(params =>
      AXI4SlavePortParameters(
        slaves = Seq(AXI4SlaveParameters(
          address       = AddressSet.misaligned(params.base, params.size),
          resources     = device.ranges,
          executable    = params.executable,
          supportsWrite = TransferSizes(1, params.maxXferBytes),
          supportsRead  = TransferSizes(1, params.maxXferBytes))),
        beatBytes = params.beatBytes)).toSeq)

  mmioPortParamsOpt.map { params =>
    sbus.coupleTo(s"port_named_$portName") {
      (mmioAXI4Node
        := AXI4Buffer()
        := AXI4UserYanker()
        := AXI4Deinterleaver(sbus.blockBytes)
        := AXI4IdIndexer(params.idBits)
        := TLToAXI4()
        := TLWidthWidget(sbus.beatBytes)
        := _)
    }
  }

  val mmio_axi4 = InModuleBody { mmioAXI4Node.makeIOs() }
}

/** Adds an AXI4 port to the system intended to be a slave on an MMIO device bus */
trait CanHaveSlaveAXI4Port { this: BaseSubsystem =>
  private val slavePortParamsOpt = p(ExtIn)
  private val portName = "slave_port_axi4"
  private val fifoBits = 1

  val l2FrontendAXI4Node = AXI4MasterNode(
    slavePortParamsOpt.map(params =>
      AXI4MasterPortParameters(
        masters = Seq(AXI4MasterParameters(
          name = portName.kebab,
          id   = IdRange(0, 1 << params.idBits))))).toSeq)

  slavePortParamsOpt.map { params =>
    fbus.coupleFrom(s"port_named_$portName") {
      ( _
        := TLBuffer(BufferParams.default)
        := TLFIFOFixer(TLFIFOFixer.all)
        := TLWidthWidget(params.beatBytes)
        := AXI4ToTL()
        := AXI4UserYanker(Some(1 << (params.sourceBits - fifoBits - 1)))
        := AXI4Fragmenter()
        := AXI4IdIndexer(fifoBits)
        := l2FrontendAXI4Node )
    }
  }

  val l2_frontend_bus_axi4 = InModuleBody { l2FrontendAXI4Node.makeIOs() }
}

/** Adds a TileLink port to the system intended to master an MMIO device bus */
trait CanHaveMasterTLMMIOPort { this: BaseSubsystem =>
  private val mmioPortParamsOpt = p(ExtBus)
  private val portName = "mmio_port_tl"
  private val device = new SimpleBus(portName.kebab, Nil)

  val mmioTLNode = TLManagerNode(
    mmioPortParamsOpt.map(params =>
      TLSlavePortParameters.v1(
        managers = Seq(TLSlaveParameters.v1(
          address            = AddressSet.misaligned(params.base, params.size),
          resources          = device.ranges,
          executable         = params.executable,
          supportsGet        = TransferSizes(1, sbus.blockBytes),
          supportsPutFull    = TransferSizes(1, sbus.blockBytes),
          supportsPutPartial = TransferSizes(1, sbus.blockBytes))),
        beatBytes = params.beatBytes)).toSeq)

  mmioPortParamsOpt.map { params =>
    sbus.coupleTo(s"port_named_$portName") {
      (mmioTLNode
        := TLBuffer()
        := TLSourceShrinker(1 << params.idBits)
        := TLWidthWidget(sbus.beatBytes)
        := _ )
    }
  }

  val mmio_tl = InModuleBody {
    mmioTLNode.out.foreach { case (_, edge) => println(edge.prettySourceMapping(s"TL MMIO Port")) }
    mmioTLNode.makeIOs()
  }
}

/** Adds an TL port to the system intended to be a slave on an MMIO device bus.
  * NOTE: this port is NOT allowed to issue Acquires.
  */
trait CanHaveSlaveTLPort { this: BaseSubsystem =>
  private val slavePortParamsOpt = p(ExtIn)
  private val portName = "slave_port_tl"

  val l2FrontendTLNode = TLClientNode(
    slavePortParamsOpt.map(params =>
      TLMasterPortParameters.v1(
        clients = Seq(TLMasterParameters.v1(
          name     = portName.kebab,
          sourceId = IdRange(0, 1 << params.idBits))))).toSeq)

  slavePortParamsOpt.map { params =>
    sbus.coupleFrom(s"port_named_$portName") {
      ( _
        := TLSourceShrinker(1 << params.sourceBits)
        := TLWidthWidget(params.beatBytes)
        := l2FrontendTLNode )
    }
  }

  val l2_frontend_bus_tl = InModuleBody { l2FrontendTLNode.makeIOs() }
}
