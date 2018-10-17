// See LICENSE.SiFive for license details.

package freechips.rocketchip.util

import chisel3._
import chisel3.util._

/////////////////////////////// Abstract API for the Queue /////////////////////////

class LanePositionedDecoupledIO[T <: Data](private val gen: T, val lanes: Int) extends Bundle {
  val laneBits1 = log2Ceil(lanes+1) // [0, lanes]

  val ready = Input (UInt(laneBits1.W))
  val valid = Output(UInt(laneBits1.W))
  val bits  = Output(Vec(lanes, gen))
}

class LanePositionedQueueIO[T <: Data](private val gen: T, val lanes: Int) extends Bundle {
  val laneBitsU = log2Up(lanes)

  val enq = Flipped(new LanePositionedDecoupledIO(gen, lanes))
  val deq = new LanePositionedDecoupledIO(gen, lanes)
  val enq_0_lane = Output(UInt(laneBitsU.W))
  val deq_0_lane = Output(UInt(laneBitsU.W))
}

trait LanePositionedQueueModule[T <: Data] extends Module {
  val io: LanePositionedQueueIO[T]
}

trait LanePositionedQueue {
  def apply[T <: Data](gen: T, lanes: Int, rows: Int): LanePositionedQueueModule[T]
}

/////////////////////////////// Index math implementation //////////////////////////

// A shared base class that keeps track of the indexing and flow control
class LanePositionedQueueBase[T <: Data](val gen: T, val lanes: Int, val rows: Int) extends Module with LanePositionedQueueModule[T] {
  require (rows >= 1)
  require (lanes >= 1)

  val io = IO(new LanePositionedQueueIO(gen, lanes))

  val capacity  = rows * lanes
  val rowBits   = log2Ceil(rows)
  val laneBits  = log2Ceil(lanes)
  val laneBits1 = log2Ceil(lanes+1) // [0, lanes]

  def lane(add: UInt) = {
    if (lanes == 1) 0.U else {
      val out = RegInit(0.U(laneBits.W))
      if (isPow2(lanes)) {
        out := out + add
      } else {
        val z = out +& add
        val s = z - lanes.U
        out := Mux(s.asSInt >= 0.S, s, z)
      }
      out
    }
  }

  def row(inc: Bool) = {
    if (rows == 1) (0.U, 0.U) else {
      val out = RegInit(0.U(rowBits.W))
      val out1 = WireInit(out + 1.U)
      if (!isPow2(rows)) {
        when (out === (rows-1).U) { out1 := 0.U }
      }
      when (inc) { out := out1 }
      (out, out1)
    }
  }

  val enq_add  = Wire(UInt(laneBits1.W))
  val enq_wrap = Wire(Bool())
  val enq_lane = lane(enq_add)
  val (enq_row, enq_row1) = row(enq_wrap)

  val deq_add  = Wire(UInt(laneBits1.W))
  val deq_wrap = Wire(Bool())
  val deq_lane = lane(deq_add)
  val (deq_row, deq_row1) = row(deq_wrap)

  val capBits1 = log2Ceil(capacity+1)
  val delta = enq_add.zext() - deq_add.zext()
  val used = RegInit(       0.U(capBits1.W))
  val free = RegInit(capacity.U(capBits1.W))
  used := (used.asSInt + delta).asUInt
  free := (free.asSInt - delta).asUInt

  val enq_pos = enq_row * lanes.U + enq_lane
  val deq_pos = deq_row * lanes.U + deq_lane
  val diff_pos = enq_pos + Mux(enq_pos >= deq_pos, 0.U, capacity.U) - deq_pos
  assert (used === diff_pos || (diff_pos === 0.U && used === capacity.U))
  assert (used + free === capacity.U)

  io.enq_0_lane := enq_lane
  io.deq_0_lane := deq_lane

  val enq_all = free >= lanes.U
  val enq_low = free(laneBits1-1, 0)
  io.enq.ready := Mux(enq_all, lanes.U, enq_low)
  enq_add := Mux(enq_all || enq_low > io.enq.valid, io.enq.valid, enq_low)
  enq_wrap := enq_add +& enq_lane >= lanes.U

  val deq_all = used >= lanes.U
  val deq_low = used(laneBits1-1, 0)
  io.deq.valid := Mux(deq_all, lanes.U, deq_low)
  deq_add := Mux(deq_all || deq_low > io.deq.ready, io.deq.ready, deq_low)
  deq_wrap := deq_add +& deq_lane >= lanes.U

  val enq_vmask = UIntToOH1(io.enq.valid +& enq_lane, 2*lanes-1).pad(2*lanes)
  val enq_rmask = UIntToOH1(io.enq.ready +& enq_lane, 2*lanes-1).pad(2*lanes)
  val enq_lmask = UIntToOH1(                enq_lane,     lanes).pad(2*lanes)
  val enq_mask  = ((enq_vmask & enq_rmask) & ~enq_lmask)

  val deq_vmask = UIntToOH1(io.deq.valid +& deq_lane, 2*lanes-1).pad(2*lanes)
  val deq_rmask = UIntToOH1(io.deq.ready +& deq_lane, 2*lanes-1).pad(2*lanes)
  val deq_lmask = UIntToOH1(                deq_lane,     lanes).pad(2*lanes)
  val deq_mask  = ((deq_vmask & deq_rmask) & ~deq_lmask)
}

/////////////////////////////// Registered implementation //////////////////////////

class FloppedLanePositionedQueueModule[T <: Data](gen: T, lanes: Int, rows: Int)
    extends LanePositionedQueueBase(gen, lanes, rows) {

  require (rows % 2 == 0)
  val bank = Seq.fill(2) { Mem(rows/2, Vec(lanes, gen)) }

  val b0_out = bank(0).read(deq_row1 >> 1)
  val b1_out = bank(1).read(deq_row  >> 1)
  for (l <- 0 until lanes) {
    io.deq.bits(l) := Mux(deq_row(0) ^ deq_lmask(l), b1_out(l), b0_out(l))
  }

  val hi_mask = enq_mask(2*lanes-1, lanes)
  val lo_mask = enq_mask(lanes-1, 0)
  val b0_mask = Mux(enq_row(0), hi_mask, lo_mask).toBools
  val b1_mask = Mux(enq_row(0), lo_mask, hi_mask).toBools
  val b0_row  = enq_row1 >> 1
  val b1_row  = enq_row  >> 1

  bank(0).write(b0_row, io.enq.bits, b0_mask)
  bank(1).write(b1_row, io.enq.bits, b1_mask)
}

object FloppedLanePositionedQueue extends LanePositionedQueue {
  def apply[T <: Data](gen: T, lanes: Int, rows: Int) = new FloppedLanePositionedQueueModule(gen, lanes, rows)
}

/////////////////////////////// One port implementation ////////////////////////////

class OnePortLanePositionedQueueModule[T <: Data](ecc: Code)(gen: T, lanes: Int, rows: Int)
    extends LanePositionedQueueBase(gen, lanes, rows) {

  require (rows > 8 && rows % 4 == 0)
  // If rows <= 8, use FloppedLanePositionedQueue instead

  // Make the SRAM twice as wide, so that we can use 1RW port
  // Read accesses have priority, but we never do two back-back
  val ram = SyncReadMem(rows/2, Vec(2*lanes, gen))

  val enq_buffer = Reg(Vec(4, Vec(lanes, gen)))
  val deq_buffer = Reg(Vec(4, Vec(lanes, gen)))

  val gap = (enq_row >> 1).zext() - (deq_row >> 1).zext()
  val gap0 = gap === 0.S
  val gap1 = gap0 || gap === (1-rows/2).S || gap === 1.S
  val gap2 = gap1 || gap === (2-rows/2).S || gap === 2.S

  val deq_push = deq_wrap && deq_row(0)
  val enq_push = enq_wrap && enq_row(0)
  val ren = deq_push // !!! optimize
  val wen = RegInit(false.B)

  when (!ren)     { wen := false.B }
  when (enq_push) { wen := true.B } // !!! optimize

  val write_row = RegEnable(enq_row, enq_push)
  val ram_i = Mux(write_row(1),
    VecInit(enq_buffer(2) ++ enq_buffer(3)),
    VecInit(enq_buffer(0) ++ enq_buffer(1)))
  val deq_row_half = deq_row >> 1
  val read_row =
    Mux(!isPow2(rows).B && deq_row_half === (rows/2-1).U, 1.U,
    Mux(!isPow2(rows).B && deq_row_half === (rows/2-2).U, 0.U,
    deq_row_half + 2.U))
  val ram_o = ram.read(read_row, ren)
  when (wen && !ren) { ram.write(write_row >> 1, ram_i) }

  val deq_fill = RegNext(deq_push)
  for (l <- 0 until lanes) {
    when (deq_fill && deq_row(1)) {
      deq_buffer(0)(l) := Mux(gap2, enq_buffer(0)(l), ram_o(l))
      deq_buffer(1)(l) := Mux(gap2, enq_buffer(1)(l), ram_o(l+lanes))
    }
    when (deq_fill && !deq_row(1)) {
      deq_buffer(2)(l) := Mux(gap2, enq_buffer(2)(l), ram_o(l))
      deq_buffer(3)(l) := Mux(gap2, enq_buffer(3)(l), ram_o(l+lanes))
    }
    for (r <- 0 until 4) {
      val rn = (r+3) % 4
      when ((enq_mask(l)       && enq_row(1,0) === r.U) ||
            (enq_mask(l+lanes) && enq_row(1,0) === rn.U)) {
        enq_buffer(r)(l) := io.enq.bits(l)
      }
      val gap = if (r % 2 == 0) gap0 else gap1
      when ((gap1 && enq_mask(l)       && enq_row(1,0) === r.U) ||
            (gap  && enq_mask(l+lanes) && enq_row(1,0) === rn.U)) {
        deq_buffer(r)(l) := io.enq.bits(l)
      }
    }
  }

  val deq_buf0 = deq_buffer(deq_row(1,0))
  val deq_buf1 = VecInit.tabulate(4) { i => deq_buffer((i+1) % 4) } (deq_row(1,0))
  for (l <- 0 until lanes) {
    io.deq.bits(l) := Mux(deq_lmask(l), deq_buf1(l), deq_buf0(l))
  }
}

case class OnePortLanePositionedQueue(ecc: Code) extends LanePositionedQueue {
  def apply[T <: Data](gen: T, lanes: Int, rows: Int) = new OnePortLanePositionedQueueModule(ecc)(gen, lanes, rows)
}

/////////////////////////////// Black Box Unit Testing /////////////////////////////

import freechips.rocketchip.unittest._
import freechips.rocketchip.tilelink.LFSR64

class PositionedQueueTest(queueFactory: LanePositionedQueue, lanes: Int, rows: Int, cycles: Int, timeout: Int = 500000) extends UnitTest(timeout) {
  val ids = (cycles+1) * lanes
  val bits = log2Ceil(ids+1)

  val q = Module(queueFactory(UInt(bits.W), lanes, rows))

  val enq = RegInit(0.U(bits.W))
  val deq = RegInit(0.U(bits.W))
  val done = RegInit(false.B)

  q.io.enq.valid := (LFSR64() * (q.io.enq.ready +& 1.U)) >> 64
  q.io.deq.ready := (LFSR64() * (q.io.deq.valid +& 1.U)) >> 64

  enq := enq + q.io.enq.valid
  deq := deq + q.io.deq.ready

  when (enq >= (cycles*lanes).U) { done := true.B }
  io.finished := done

  q.io.enq.bits := VecInit.tabulate(lanes) { i =>
    val pos = Mux(i.U >= q.io.enq_0_lane, i.U, (i + lanes).U) - q.io.enq_0_lane
    enq + pos
  }

  q.io.deq.bits.zipWithIndex.foreach { case (d, i) =>
    val pos = Mux(i.U >= q.io.deq_0_lane, i.U, (i + lanes).U) - q.io.deq_0_lane
    assert (pos >= q.io.deq.valid || d === deq + pos)
  }
}
