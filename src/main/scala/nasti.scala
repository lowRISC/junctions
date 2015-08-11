/// See LICENSE for license details.

package junctions
import Chisel._
import scala.math.max
import scala.collection.mutable.ArraySeq
import scala.collection.mutable.HashMap

case object NASTIDataBits extends Field[Int]
case object NASTIAddrBits extends Field[Int]
case object NASTIIdBits extends Field[Int]

trait NASTIParameters extends UsesParameters {
  val nastiXDataBits = params(NASTIDataBits)
  val nastiWStrobeBits = nastiXDataBits / 8
  val nastiXAddrBits = params(NASTIAddrBits)
  val nastiWIdBits = params(NASTIIdBits)
  val nastiRIdBits = params(NASTIIdBits)
  val nastiXIdBits = max(nastiWIdBits, nastiRIdBits)
  val nastiXUserBits = 1
  val nastiAWUserBits = nastiXUserBits
  val nastiWUserBits = nastiXUserBits
  val nastiBUserBits = nastiXUserBits
  val nastiARUserBits = nastiXUserBits
  val nastiRUserBits = nastiXUserBits
  val nastiXLenBits = 8
  val nastiXSizeBits = 3
  val nastiXBurstBits = 2
  val nastiXCacheBits = 4
  val nastiXProtBits = 3
  val nastiXQosBits = 4
  val nastiXRegionBits = 4
  val nastiXRespBits = 2

  def bytesToXSize(bytes: UInt) = MuxLookup(bytes, UInt("b111"), Array(
    UInt(1) -> UInt(0),
    UInt(2) -> UInt(1),
    UInt(4) -> UInt(2),
    UInt(8) -> UInt(3),
    UInt(16) -> UInt(4),
    UInt(32) -> UInt(5),
    UInt(64) -> UInt(6),
    UInt(128) -> UInt(7)))
}

abstract class NASTIBundle extends Bundle with NASTIParameters
abstract class NASTIModule extends Module with NASTIParameters

trait NASTIChannel extends NASTIBundle
trait NASTIMasterToSlaveChannel extends NASTIChannel
trait NASTISlaveToMasterChannel extends NASTIChannel

class NASTIMasterIO extends Bundle {
  val aw = Decoupled(new NASTIWriteAddressChannel)
  val w  = Decoupled(new NASTIWriteDataChannel)
  val b  = Decoupled(new NASTIWriteResponseChannel).flip
  val ar = Decoupled(new NASTIReadAddressChannel)
  val r  = Decoupled(new NASTIReadDataChannel).flip
}

class NASTISlaveIO extends NASTIMasterIO { flip() }

trait HasNASTIMetadata extends NASTIBundle {
  val addr   = UInt(width = nastiXAddrBits)
  val len    = UInt(width = nastiXLenBits)
  val size   = UInt(width = nastiXSizeBits)
  val burst  = UInt(width = nastiXBurstBits)
  val lock   = Bool()
  val cache  = UInt(width = nastiXCacheBits)
  val prot   = UInt(width = nastiXProtBits)
  val qos    = UInt(width = nastiXQosBits)
  val region = UInt(width = nastiXRegionBits)
}

trait HasNASTIData extends NASTIBundle {
  val data = UInt(width = nastiXDataBits)
  val last = Bool()
}

class NASTIAddressChannel extends NASTIMasterToSlaveChannel with HasNASTIMetadata

class NASTIResponseChannel extends NASTISlaveToMasterChannel {
  val resp = UInt(width = nastiXRespBits)
}

class NASTIWriteAddressChannel extends NASTIAddressChannel {
  val id   = UInt(width = nastiWIdBits)
  val user = UInt(width = nastiAWUserBits)
}

class NASTIWriteDataChannel extends NASTIMasterToSlaveChannel with HasNASTIData {
  val strb = UInt(width = nastiWStrobeBits)
  val user = UInt(width = nastiWUserBits)
}

class NASTIWriteResponseChannel extends NASTIResponseChannel {
  val id   = UInt(width = nastiWIdBits)
  val user = UInt(width = nastiBUserBits)
}

class NASTIReadAddressChannel extends NASTIAddressChannel {
  val id   = UInt(width = nastiRIdBits)
  val user = UInt(width = nastiARUserBits)
}

class NASTIReadDataChannel extends NASTIResponseChannel with HasNASTIData {
  val id   = UInt(width = nastiRIdBits)
  val user = UInt(width = nastiRUserBits)
}

class MemIONASTISlaveIOConverter(cacheBlockOffsetBits: Int) extends MIFModule with NASTIParameters {
  val io = new Bundle {
    val nasti = new NASTISlaveIO
    val mem = new MemIO
  }

  require(mifDataBits == nastiXDataBits, "Data sizes between LLC and MC don't agree")
  val (mif_cnt_out, mif_wrap_out) = Counter(io.mem.resp.fire(), mifDataBeats)

  // according to the spec, we can't send b until the last transfer on w
  val b_ok = Reg(init = Bool(true))
  when (io.nasti.aw.fire()) { b_ok := Bool(false) }
  when (io.nasti.w.fire() && io.nasti.w.bits.last) { b_ok := Bool(true) }

  val id_q = Module(new Queue(UInt(width = nastiWIdBits), 2))
  id_q.io.enq.valid := io.nasti.aw.valid
  id_q.io.enq.bits := io.nasti.aw.bits.id
  id_q.io.deq.ready := io.nasti.b.ready && b_ok

  io.mem.req_cmd.bits.addr := Mux(io.nasti.aw.valid, io.nasti.aw.bits.addr, io.nasti.ar.bits.addr) >>
                                UInt(cacheBlockOffsetBits)
  io.mem.req_cmd.bits.tag := Mux(io.nasti.aw.valid, io.nasti.aw.bits.id, io.nasti.ar.bits.id)
  io.mem.req_cmd.bits.rw := io.nasti.aw.valid
  io.mem.req_cmd.valid := (io.nasti.aw.valid && id_q.io.enq.ready) || io.nasti.ar.valid
  io.nasti.ar.ready := io.mem.req_cmd.ready && !io.nasti.aw.valid
  io.nasti.aw.ready := io.mem.req_cmd.ready && id_q.io.enq.ready

  io.nasti.b.valid := id_q.io.deq.valid && b_ok
  io.nasti.b.bits.id := id_q.io.deq.bits
  io.nasti.b.bits.resp := UInt(0)

  io.nasti.w.ready := io.mem.req_data.ready
  io.mem.req_data.valid := io.nasti.w.valid
  io.mem.req_data.bits.data := io.nasti.w.bits.data
  assert(!io.nasti.w.valid || io.nasti.w.bits.strb.andR, "MemIO must write full cache line")

  io.nasti.r.valid := io.mem.resp.valid
  io.nasti.r.bits.data := io.mem.resp.bits.data
  io.nasti.r.bits.last := mif_wrap_out
  io.nasti.r.bits.id := io.mem.resp.bits.tag
  io.nasti.r.bits.resp := UInt(0)
  io.mem.resp.ready := io.nasti.r.ready
}

class NASTIArbiter(val arbN: Int) extends NASTIModule {
  val io = new Bundle {
    val master = Vec.fill(arbN) { new NASTISlaveIO }
    val slave = new NASTIMasterIO
  }

  if (arbN > 1) {
    val arbIdBits = log2Up(arbN)

    val ar_arb = Module(new RRArbiter(new NASTIReadAddressChannel, arbN))
    val aw_arb = Module(new RRArbiter(new NASTIWriteAddressChannel, arbN))

    val slave_r_arb_id = io.slave.r.bits.id(arbIdBits - 1, 0)
    val slave_b_arb_id = io.slave.b.bits.id(arbIdBits - 1, 0)

    val w_chosen = Reg(UInt(width = arbIdBits))
    val w_done = Reg(init = Bool(true))

    when (aw_arb.io.out.fire()) {
      w_chosen := aw_arb.io.chosen
      w_done := Bool(false)
    }

    when (io.slave.w.fire() && io.slave.w.bits.last) {
      w_done := Bool(true)
    }

    for (i <- 0 until arbN) {
      val m_ar = io.master(i).ar
      val m_aw = io.master(i).aw
      val m_r = io.master(i).r
      val m_b = io.master(i).b
      val a_ar = ar_arb.io.in(i)
      val a_aw = aw_arb.io.in(i)
      val m_w = io.master(i).w

      a_ar <> m_ar
      a_ar.bits.id := Cat(m_ar.bits.id, UInt(i, arbIdBits))

      a_aw <> m_aw
      a_aw.bits.id := Cat(m_aw.bits.id, UInt(i, arbIdBits))

      m_r.valid := io.slave.r.valid && slave_r_arb_id === UInt(i)
      m_r.bits := io.slave.r.bits
      m_r.bits.id := io.slave.r.bits.id >> UInt(arbIdBits)

      m_b.valid := io.slave.b.valid && slave_b_arb_id === UInt(i)
      m_b.bits := io.slave.b.bits
      m_b.bits.id := io.slave.b.bits.id >> UInt(arbIdBits)

      m_w.ready := io.slave.w.ready && w_chosen === UInt(i) && !w_done
    }

    io.slave.r.ready := io.master(slave_r_arb_id).r.ready
    io.slave.b.ready := io.master(slave_b_arb_id).b.ready

    io.slave.w.bits := io.master(w_chosen).w.bits
    io.slave.w.valid := io.master(w_chosen).w.valid && !w_done

    io.slave.ar <> ar_arb.io.out
    io.slave.aw <> aw_arb.io.out
    aw_arb.io.out.ready := io.slave.aw.ready && w_done

  } else { io.slave <> io.master.head }
}

// TODO: More efficient implementation a/la Chisel Stdlib
class NASTIReadDataArbiter(arbN: Int) extends NASTIModule {
  val io = new Bundle {
    val in = Vec.fill(arbN) { Decoupled(new NASTIReadDataChannel) }.flip
    val out = Decoupled(new NASTIReadDataChannel)
  }

  val counter = Counter(arbN)
  val choice = counter.value
  val locked = Reg(init = Bool(false))

  for (i <- 0 until arbN) {
    io.in(i).ready := io.out.ready && choice === UInt(i)
  }

  io.out.valid := io.in(choice).valid
  io.out.bits := io.in(choice).bits

  when (io.out.fire()) {
    when (io.out.bits.last) {
      locked := Bool(false)
    } .otherwise {
      locked := Bool(true)
    }
  } .elsewhen (!locked) {
    counter.inc()
  }
}

class NASTIRouter(addrmap: Seq[(Int, Int)]) extends NASTIModule {
  val nSlaves = addrmap.size

  val io = new Bundle {
    val master = new NASTISlaveIO
    val slave = Vec.fill(nSlaves) { new NASTIMasterIO }
  }

  var ar_ready = Bool(false)
  var aw_ready = Bool(false)
  var w_ready = Bool(false)

  addrmap.zip(io.slave).zipWithIndex.foreach { case (((base, size), s), i) =>
    val offset = log2Floor(size)
    val regionId = base >> offset

    require(isPow2(size))
    require(base % size == 0)

    val ar_reg_addr = io.master.ar.bits.addr(nastiXAddrBits - 1, offset)
    val ar_match = ar_reg_addr === UInt(regionId)

    s.ar.valid := io.master.ar.valid && ar_match
    s.ar.bits := io.master.ar.bits
    ar_ready = ar_ready || (s.ar.ready && ar_match)

    val aw_reg_addr = io.master.aw.bits.addr(nastiXAddrBits - 1, offset)
    val aw_match = aw_reg_addr === UInt(regionId)

    s.aw.valid := io.master.aw.valid && aw_match
    s.aw.bits := io.master.aw.bits
    aw_ready = aw_ready || (s.aw.ready && aw_match)

    val chosen = Reg(init = Bool(false))
    when (s.aw.fire()) { chosen := Bool(true) }
    when (s.w.fire() && s.w.bits.last) { chosen := Bool(false) }

    s.w.valid := io.master.w.valid && chosen
    s.w.bits := io.master.w.bits
    w_ready = w_ready || (s.w.ready && chosen)
  }

  io.master.ar.ready := ar_ready
  io.master.aw.ready := aw_ready
  io.master.w.ready := w_ready

  val b_arb = Module(new RRArbiter(new NASTIWriteResponseChannel, nSlaves))
  val r_arb = Module(new NASTIReadDataArbiter(nSlaves))

  b_arb.io.in <> io.slave.map(s => s.b)
  r_arb.io.in <> io.slave.map(s => s.r)

  io.master.b <> b_arb.io.out
  io.master.r <> r_arb.io.out
}

class NASTICrossbar(nMasters: Int, nSlaves: Int, addrmap: Seq[(Int, Int)])
    extends NASTIModule {
  val io = new Bundle {
    val masters = Vec.fill(nMasters) { new NASTISlaveIO }
    val slaves = Vec.fill(nSlaves) { new NASTIMasterIO }
  }

  val routers = Vec.fill(nMasters) { Module(new NASTIRouter(addrmap)).io }
  val arbiters = Vec.fill(nSlaves) { Module(new NASTIArbiter(nMasters)).io }

  for (i <- 0 until nMasters) {
    routers(i).master <> io.masters(i)
  }

  for (i <- 0 until nSlaves) {
    arbiters(i).master <> Vec(routers.map(r => r.slave(i)))
    io.slaves(i) <> arbiters(i).slave
  }
}

case object NASTINMasters extends Field[Int]
case object NASTINSlaves extends Field[Int]

object AddrMapTypes {
  type AddrMapEntry = (String, Option[Int], MemRegion)
  type AddrMap = Seq[AddrMapEntry]
}
import AddrMapTypes._

abstract class MemRegion

case class MemSize(size: Int) extends MemRegion
case class MemSubmap(entries: AddrMap) extends MemRegion

object Submap {
  def apply(entries: AddrMapEntry*) =
    new MemSubmap(entries)
}

class PortMap(addrmap: AddrMap) {
  val mapping = new HashMap[String, Int]

  private def genPairs(addrmap: AddrMap): Seq[(String, Int)] = {
    var ind = 0
    var pairs = Seq[(String, Int)]()
    addrmap.foreach { case (name, _, region) =>
      region match {
        case MemSize(_) => {
          pairs = (name, ind) +: pairs
          ind += 1
        }
        case MemSubmap(submap) => {
          val subpairs = genPairs(submap).map {
            case (subname, subind) => (name + ":" + subname, ind + subind)
          }
          pairs = subpairs ++ pairs
          ind += subpairs.size
        }
      }
    }
    pairs
  }

  for ((name, ind) <- genPairs(addrmap)) { mapping(name) = ind }

  def apply(name: String): Int = mapping(name)
  def get(name: String): Option[Int] = mapping.get(name)
  def getOrElse(name: String, default: Int): Int = mapping.getOrElse(name, default)
}

case object NASTIAddrMap extends Field[AddrMap]

class NASTIInterconnectIO(val nMasters: Int, val nSlaves: Int) extends Bundle {
  /* This is a bit confusing. The interconnect is a slave to the masters and
   * a master to the slaves. Hence why the declarations seem to be backwards. */
  val masters = Vec.fill(nMasters) { new NASTISlaveIO }
  val slaves = Vec.fill(nSlaves) { new NASTIMasterIO }
  override def cloneType =
    new NASTIInterconnectIO(nMasters, nSlaves).asInstanceOf[this.type]
}

abstract class NASTIInterconnect extends NASTIModule {
  val nMasters: Int
  val nSlaves: Int

  lazy val io = new NASTIInterconnectIO(nMasters, nSlaves)
}

case object BuildNASTI extends Field[() => NASTIInterconnect]

class NASTIRecursiveInterconnect(
    val nMasters: Int, val nSlaves: Int,
    base: Int, addrmap: AddrMap) extends NASTIInterconnect {

  private def regionSize(region: MemRegion): Int = {
    val size = region match {
      case MemSize(size) => size
      case MemSubmap(submap) => submap.map {
        case (_, _, region) => regionSize(region)
      }.reduceLeft(_ + _)
    }
    if (isPow2(size)) size else (1 << log2Ceil(size))
  }

  private def mapCountSlaves(addrmap: AddrMap): Int = {
    addrmap.map {
      case (_, _, MemSize(_)) => 1
      case (_, _, MemSubmap(submap)) => mapCountSlaves(submap)
    }.reduceLeft(_ + _)
  }

  var lastEnd = base
  var slaveInd = 0
  val levelSize = addrmap.size

  val realAddrMap = new ArraySeq[(Int, Int)](addrmap.size)

  addrmap.zipWithIndex.foreach { case ((_, startOpt, region), i) =>
    val start = startOpt.getOrElse(lastEnd)
    val size = regionSize(region)
    realAddrMap(i) = (start, size)
    lastEnd = start + size
  }

  val flatSlaves = if (nMasters > 1) {
    val xbar = Module(new NASTICrossbar(nMasters, levelSize, realAddrMap))
    xbar.io.masters <> io.masters
    xbar.io.slaves
  } else {
    val router = Module(new NASTIRouter(realAddrMap))
    router.io.master <> io.masters.head
    router.io.slave
  }

  addrmap.zip(realAddrMap).zipWithIndex.foreach {
    case (((_, _, region), (start, size)), i) => {
      region match {
        case MemSize(_) =>
          io.slaves(slaveInd) <> flatSlaves(i)
          slaveInd += 1
        case MemSubmap(submap) =>
          val subSlaves = mapCountSlaves(submap)
          val ic = Module(new NASTIRecursiveInterconnect(
            1, subSlaves, start, submap))
          ic.io.masters.head <> flatSlaves(i)
          io.slaves.drop(slaveInd).take(subSlaves).zip(ic.io.slaves).foreach {
            case (s, m) => s <> m
          }
          slaveInd += subSlaves
      }
    }
  }
}

class NASTITopInterconnect extends NASTIInterconnect {
  val nMasters = params(NASTINMasters)
  val nSlaves = params(NASTINSlaves)

  val temp = Module(new NASTIRecursiveInterconnect(
    nMasters, nSlaves, 0, params(NASTIAddrMap)))

  temp.io.masters.zip(io.masters).foreach { case (t, i) =>
    t.ar <> i.ar
    t.aw <> i.aw
    // this queue is necessary to break up the aw - w dependence
    // introduced by the TileLink -> NASTI converter
    t.w <> Queue(i.w)
    i.b <> t.b
    i.r <> t.r
  }
  //temp.io.masters <> io.masters
  io.slaves <> temp.io.slaves
}
