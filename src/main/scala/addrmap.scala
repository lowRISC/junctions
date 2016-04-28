// See LICENSE for license details.

package junctions

import Chisel._
import cde.{Parameters, Field}
import scala.collection.mutable.HashMap

case object PAddrBits extends Field[Int]
case object VAddrBits extends Field[Int]
case object PgIdxBits extends Field[Int]
case object PgLevels extends Field[Int]
case object PgLevelBits extends Field[Int]
case object ASIdBits extends Field[Int]
case object PPNBits extends Field[Int]
case object VPNBits extends Field[Int]

case object GlobalAddrMap extends Field[AddrMap]
case object MMIOBase extends Field[BigInt]

trait HasAddrMapParameters {
  implicit val p: Parameters

  val paddrBits = p(PAddrBits)
  val vaddrBits = p(VAddrBits)
  val pgIdxBits = p(PgIdxBits)
  val ppnBits = p(PPNBits)
  val vpnBits = p(VPNBits)
  val pgLevels = p(PgLevels)
  val pgLevelBits = p(PgLevelBits)
  val asIdBits = p(ASIdBits)

  val addrMap = new AddrHashMap(p(GlobalAddrMap), p(MMIOBase))
}

abstract class MemRegion { def size: BigInt }

case class MemSize(size: BigInt, prot: Int) extends MemRegion
case class MemSubmap(size: BigInt, entries: AddrMap, sharePort: Boolean = false) extends MemRegion

object AddrMapConsts {
  val R = 0x1
  val W = 0x2
  val X = 0x4
  val RW = R | W
  val RX = R | X
  val RWX = R | W | X
}

class AddrMapProt extends Bundle {
  val x = Bool()
  val w = Bool()
  val r = Bool()
}

case class AddrMapEntry(name: String, start: Option[BigInt], region: MemRegion)

case class AddrHashMapEntry(port: Int, start: BigInt, size: BigInt, prot: Int)

class AddrMap(entries: Seq[AddrMapEntry]) extends scala.collection.IndexedSeq[AddrMapEntry] {
  
  def apply(index: Int): AddrMapEntry = entries(index)

  def length: Int = entries.size

  def countSlaves: Int = {
    this map { entry: AddrMapEntry => entry.region match {
      case MemSize(_, _) => 1
      case MemSubmap(_, submap, _) => submap.countSlaves
    }} reduceLeft(_ + _)
  }
}

object AddrMap {
  def apply(elems: AddrMapEntry*): AddrMap = new AddrMap(elems)
}

class AddrHashMap(addrmap: AddrMap, start: BigInt) {
  val entries = new HashMap[String, AddrHashMapEntry] // all entries in addrMap
  val ports = new HashMap[String, AddrHashMapEntry]   // all ported entries
  val ends = new HashMap[String, AddrHashMapEntry]    // all leaf ends

  private def genPairs(prefix: Option[String], am: AddrMap, start: BigInt, addPort: Boolean = true): Unit = {
    var base = start
    am.foreach { case AddrMapEntry(entryName, startOpt, region) =>
      val name = if(prefix.isEmpty) entryName else prefix.get + ":" + entryName
      region match {
        case MemSize(size, prot) => {
          if (!startOpt.isEmpty) base = startOpt.get
          val entry = AddrHashMapEntry(ports.size, base, size, prot)
          entries(name) = entry
          ends(name) = entry
          if(addPort) {
            ports(name) = entry
          }
          base += size
        }
        case MemSubmap(size, submap, sharePort) => {
          if (!startOpt.isEmpty) base = startOpt.get
          val entry = AddrHashMapEntry(ports.size, base, size, 0)
          entries(name) = entry
          if(addPort && sharePort) { // a shared port for the whole subtree
            ports(name) = entry
            genPairs(Some(name), submap, base, false)
          } else {
            genPairs(Some(name), submap, base, addPort)
          }
          base += size
        }
      }
    }
  }

  genPairs(None, addrmap, start)

  def nEntries: Int = ends.size
  def nPorts:Int = ports.size
  def apply(name: String): AddrHashMapEntry = entries(name)
  def get(name: String): Option[AddrHashMapEntry] = entries.get(name)

  def sortedEntries(): Seq[(String, BigInt, BigInt, Int)] = {
    val arr = new Array[(String, BigInt, BigInt, Int)](ports.size)
    ports.foreach { case (name, AddrHashMapEntry(port, base, size, prot)) =>
      arr(port) = (name, base, size, prot)
    }
    arr.toSeq
  }

  def isValid(addr: UInt): Bool = {
    addr < UInt(start) || ends.map {
      case (_, AddrHashMapEntry(_, base, size, _)) =>
        addr >= UInt(base) && addr < UInt(base + size)
    }.reduceLeft(_ || _)
  }

  def getProt(addr: UInt): AddrMapProt = {
    val protBits =
      Mux(addr < UInt(start),
        Bits(AddrMapConsts.RWX, 3),
        Mux1H(
        ends.map {
          case (_, AddrHashMapEntry(_, base, size, prot)) =>
            (addr >= UInt(base) && addr < UInt(base + size), Bits(prot, 3))
        }))
    new AddrMapProt().fromBits(protBits)
  }

}

/** Every elaborated entry ends up in this global arry so it can be printed
  * out later. */
object AllDeviceEntries {
  var entries = new HashMap[String, AddrHashMapEntry]

  def as_c_header(): String = {
    entries.map { case (name, AddrHashMapEntry(_, base, size, _)) => {
      val devName = name.replace(':','_')
      List(
        "#define DEV_MAP__" + devName + "__BASE 0x%x".format(base),
        "#define DEV_MAP__" + devName + "__MASK 0x%x".format(size-1)
      )
    }
    }.flatten.mkString("\n") + "\n"
  }
}
