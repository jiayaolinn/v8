// Copyright 2017 the V8 project authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef V8_WASM_BASELINE_S390_LIFTOFF_ASSEMBLER_S390_H_
#define V8_WASM_BASELINE_S390_LIFTOFF_ASSEMBLER_S390_H_

#include "src/wasm/baseline/liftoff-assembler.h"


namespace v8 {
namespace internal {
namespace wasm {

namespace liftoff {

//  half
//  slot        Frame
//  -----+--------------------+---------------------------
//  n+3  |   parameter n      |
//  ...  |       ...          |
//   4   |   parameter 1      | or parameter 2
//   3   |   parameter 0      | or parameter 1
//   2   |  (result address)  | or parameter 0
//  -----+--------------------+---------------------------
//   1   | return addr (lr)   |
//   0   | previous frame (fp)|
//  -----+--------------------+  <-- frame ptr (fp)
//  -1   | 0xa: WASM_COMPILED |
//  -2   |     instance       |
//  -----+--------------------+---------------------------
//  -3   |    slot 0 (high)   |   ^
//  -4   |    slot 0 (low)    |   |
//  -5   |    slot 1 (high)   | Frame slots
//  -6   |    slot 1 (low)    |   |
//       |                    |   v
//  -----+--------------------+  <-- stack ptr (sp)
//
constexpr int32_t kInstanceOffset = 2 * kSystemPointerSize;

inline MemOperand GetHalfStackSlot(int offset, RegPairHalf half) {
  int32_t half_offset =
      half == kLowWord ? 0 : LiftoffAssembler::kStackSlotSize / 2;
  return MemOperand(fp, -offset + half_offset);
}

inline MemOperand GetStackSlot(uint32_t offset) {
  return MemOperand(fp, -offset);
}

inline MemOperand GetInstanceOperand() {   
  return GetStackSlot(kInstanceOffset);
}

inline MemOperand GetMemOp(LiftoffAssembler* assm,
                           UseScratchRegisterScope* temps, Register addr,
                           Register offset, int32_t offset_imm) {
  DCHECK(is_uint32(offset_imm));
   if (offset != no_reg){
      assm->iihf(offset, Operand(0));
      if (offset_imm == 0){
        //assm->iihf(offset, Operand(0)); 
        return MemOperand(addr, offset);
      }
      Register tmp = temps->Acquire();
      assm->iihf(tmp,  Operand(0));
      assm->AddP(tmp, offset, Operand(offset_imm));
      return MemOperand(addr, tmp);
   }
   return MemOperand(addr, offset_imm);
}

inline Condition LiftoffCondToCond(Condition cond) {
  switch (cond) {
    case kSignedLessThan:
      return kUnsignedLessThan;
    case kSignedLessEqual:
      return kUnsignedLessEqual;
    case kSignedGreaterThan:
      return kUnsignedGreaterThan;
    case kSignedGreaterEqual:
      return kUnsignedGreaterEqual;
    case kEqual:
    case kUnequal:
    case kUnsignedLessThan:
    case kUnsignedLessEqual:
    case kUnsignedGreaterThan:
    case kUnsignedGreaterEqual:
      return cond;
    default:
      UNREACHABLE();
   }
  }
}  // namespace liftoff

int LiftoffAssembler::PrepareStackFrame() {
  int offset = pc_offset();
  lay(sp, MemOperand(sp, -0));
  return offset;
}

void LiftoffAssembler::PatchPrepareStackFrame(int offset, int frame_size) {
  const int kXRegSizeInBits = 64;
  const int kXRegSize = kXRegSizeInBits >> 3;
  static_assert(kStackSlotSize == kXRegSize,
                "kStackSlotSize must equal kXRegSize");

  // frame_size = RoundUp(frame_size, 16);
  // if (!(is_uint12(frame_size) ||
  //        (is_uint12(frame_size >> 12) && ((frame_size & 0xFFF) == 0)))) {
  //   // Round the stack to a page to try to fit a add/sub immediate.
  //   frame_size = RoundUp(frame_size, 0x1000);
  //   if (!((is_uint12(frame_size) ||
  //        (is_uint12(frame_size >> 12) && ((frame_size & 0xFFF) == 0))))) {
  //     // Stack greater than 4M! Because this is a quite improbable case, we
  //     // just fallback to TurboFan.
  //     bailout(kOtherReason, "Stack too big");
  //     return;
  //   }
  // }
  //#ifdef USE_SIMULATOR
  //  // When using the simulator, deal with Liftoff which allocates the stack
  //  // before checking it.
  //  // TODO(arm): Remove this when the stack check mechanism will be updated.
  //  printf("bailout\n");
  //  if (frame_size > KB / 2) {
  //    bailout(kOtherReason,
  //            "Stack limited to 512 bytes to avoid a bug in StackCheck");
  //    return;
  //  }
  //#endif
  constexpr int kAvailableSpace = 64;
  Assembler patching_assembler(
      AssemblerOptions{},
      ExternalAssemblerBuffer(buffer_start_ + offset, kAvailableSpace));
  patching_assembler.lay(sp, MemOperand(sp, -frame_size));
  DCHECK_EQ(6, patching_assembler.pc_offset());
}

void LiftoffAssembler::FinishCode() {}

void LiftoffAssembler::AbortCompilation() { AbortedCodeGeneration(); }

// static
constexpr int LiftoffAssembler::StaticStackFrameSize() {
  return liftoff::kInstanceOffset;
}

int LiftoffAssembler::SlotSizeForType(ValueType type) {
  switch (type) {
    case kWasmS128:
      return ValueTypes::ElementSizeInBytes(type);
    default:
      return kStackSlotSize;
  }
}

bool LiftoffAssembler::NeedsAlignment(ValueType type) {
  switch (type) {
    case kWasmS128:
      return true;
    default:
      // No alignment because all other types are kStackSlotSize.
      return false;
  }
}

void LiftoffAssembler::LoadConstant(LiftoffRegister reg, WasmValue value,
                                    RelocInfo::Mode rmode) {
  switch (value.type()) {
      case kWasmI32:  {
        TurboAssembler::mov(reg.gp(), Operand(value.to_i32(), rmode)); //}                    
        break;
      }
      case kWasmI64: {
        DCHECK(RelocInfo::IsNone(rmode));
        TurboAssembler::mov(reg.gp(), Operand(value.to_i64()));
        break;
      }
      case kWasmF32: {
        UseScratchRegisterScope temps(this);
        Register scratch = temps.Acquire();
        LoadFloat32Literal(reg.fp(), value.to_f32_boxed().get_scalar(), scratch);
        break;
      }
      case kWasmF64: {
        UseScratchRegisterScope temps(this);
        Register scratch = temps.Acquire();
        uint64_t int_val = bit_cast<uint64_t, double>(value.to_f64_boxed().get_scalar());
        LoadDoubleLiteral(reg.fp(), int_val, scratch);
        break;
      }
      default:
        UNREACHABLE();
    }
}

void LiftoffAssembler::LoadFromInstance(Register dst, uint32_t offset,
                                        int size) {
  DCHECK_LE(offset, kMaxInt);
  LoadP(dst, liftoff::GetInstanceOperand());
  DCHECK(size == 4 || size == 8);
  if (size == 4) {
    LoadW(dst, MemOperand(dst, offset));
  } else {
    LoadP(dst, MemOperand(dst, offset));
  }
}

void LiftoffAssembler::LoadTaggedPointerFromInstance(Register dst,
                                                     uint32_t offset) {
  LoadFromInstance(dst, offset, kTaggedSize);
}

void LiftoffAssembler::SpillInstance(Register instance) {
  StoreP(instance, liftoff::GetInstanceOperand());
}

void LiftoffAssembler::FillInstanceInto(Register dst) {
  LoadP(dst, liftoff::GetInstanceOperand());
}

void LiftoffAssembler::LoadTaggedPointer(Register dst, Register src_addr,
                                         Register offset_reg,
                                         uint32_t offset_imm,
                                         LiftoffRegList pinned) {
  STATIC_ASSERT(kTaggedSize == kInt64Size);
  Load(LiftoffRegister(dst), src_addr, offset_reg, offset_imm,
       LoadType::kI64Load, pinned);
}

void LiftoffAssembler::Load(LiftoffRegister dst, Register src_addr,
                            Register offset_reg, uint32_t offset_imm,
                            LoadType type, LiftoffRegList pinned,
                            uint32_t* protected_load_pc, bool is_load_mem) {
  UseScratchRegisterScope temps(this);
  MemOperand src_op =
        liftoff::GetMemOp(this, &temps, src_addr, offset_reg, offset_imm);
  
  if (protected_load_pc) *protected_load_pc = pc_offset();

  #if defined(V8_TARGET_BIG_ENDIAN)
   if (is_load_mem) {
   switch (type.value()) {
      case LoadType::kI32Load8U: {
        llc(dst.gp(), src_op);
        break; }
      case LoadType::kI64Load8U: {
        llgc(dst.gp(), src_op); //LOAD LOGICAL CHARACTER (64 < 8)
        break; }
      case LoadType::kI32Load8S: {
        lb(dst.gp(), src_op); //LOAD BYTE (32 <- 8)
        break;}
      case LoadType::kI64Load8S:{
        lgb(dst.gp(), src_op);//LOAD BYTE (64 < 8)
        break; }
      case LoadType::kI32Load16U:{
        lrvh(dst.gp(), src_op);
        llhr(dst.gp(), dst.gp());
        break;}
      case LoadType::kI64Load16U:{
        LoadLogicalReversedHalfWordP(dst.gp(), src_op);
        break; }
     case LoadType::kI32Load16S:{
        lrvh(dst.gp(), src_op);
        lhr(dst.gp(), dst.gp());
        break;}
      case LoadType::kI32Load:{
        lrv(dst.gp(), src_op);
        break;}
      case LoadType::kI64Load16S:{
        lrvh(dst.gp(), src_op);
        lghr(dst.gp(), dst.gp());
        break;}
      case LoadType::kI64Load32U:{
        LoadLogicalReversedWordP(dst.gp(), src_op);
        break;}
      case LoadType::kI64Load32S:{
        lrv(dst.gp(), src_op);
        lgfr(dst.gp(), dst.gp());
        break;}
      case LoadType::kI64Load:{
        lrvg(dst.gp(), src_op);
        break;}
      case LoadType::kF32Load:{
        lrv(r0, src_op);
        MovIntToFloat(dst.fp(), r0); //check?
        break;
      }
      case LoadType::kF64Load:{
        lrvg(r0, src_op);
        ldgr(dst.fp(), r0);
        break;}
      case LoadType::kS128Load: {
          MemOperand operand = src_op;
          lrvg(r0, operand);
          lrvg(r1, MemOperand(operand.rx(), operand.rb(),
                                 operand.offset() + kBitsPerByte));
          vlvgp(dst.fp(), r1, r0);
          break;
      }
      default:
        UNREACHABLE();
    }
  } else {
    switch (type.value()) {
      case LoadType::kI32Load8U:
      case LoadType::kI64Load8U:
        llgc(dst.gp(), src_op);
        break;
      case LoadType::kI32Load8S:
      case LoadType::kI64Load8S:
        lgb(dst.gp(), src_op);
        break;
      case LoadType::kI32Load16U:
      case LoadType::kI64Load16U:
        llgh(dst.gp(), src_op);
        break;
      case LoadType::kI32Load16S:
      case LoadType::kI64Load16S:
        lgh(dst.gp(), src_op);
        break;
      case LoadType::kI64Load32U:
        llgf(dst.gp(), src_op);
        break;
      case LoadType::kI32Load: {
        LoadW(dst.gp(), src_op);
        break; }
      case LoadType::kI64Load32S:
        LoadW(dst.gp(), src_op);
        break;
      case LoadType::kI64Load:
        LoadP(dst.gp(), src_op);
        break;
      case LoadType::kF32Load:
        LoadFloat32(dst.fp(), src_op);
        break;
      case LoadType::kF64Load:
        LoadDouble(dst.fp(), src_op);
        break;
      case LoadType::kS128Load:
        vl(dst.fp(), src_op, Condition(0));
        break;
      default:
        UNREACHABLE();
    }
  }
  #else
    switch (type.value()) {
      case LoadType::kI32Load8U:
      case LoadType::kI64Load8U:
        llgc(dst.gp(), src_op);
        break;
      case LoadType::kI32Load8S:
      case LoadType::kI64Load8S:
        lgb(dst.gp(), src_op);
        break;
      case LoadType::kI32Load16U:
      case LoadType::kI64Load16U:
        llgh(dst.gp(), src_op);
        break;
      case LoadType::kI32Load16S:
      case LoadType::kI64Load16S:
        lgh(dst.gp(), src_op);
        break;
      case LoadType::kI64Load32U:
        llgf(dst.gp(), src_op);
        break;
      case LoadType::kI32Load: {
        LoadW(dst.gp(), src_op);
        break; }
      case LoadType::kI64Load32S:
        LoadW(dst.gp(), src_op);
        break;
      case LoadType::kI64Load:
        LoadP(dst.gp(), src_op);
        break;
      case LoadType::kF32Load:
        LoadFloat32(dst.fp(), src_op);
        break;
      case LoadType::kF64Load:
        LoadDouble(dst.fp(), src_op);
        break;
      case LoadType::kS128Load:
        vl(dst.fp(), src_op, Condition(0));
        break;
      default:
        UNREACHABLE();
    }
  
}

void LiftoffAssembler::Store(Register dst_addr, Register offset_reg,
                             uint32_t offset_imm, LiftoffRegister src,
                             StoreType type, LiftoffRegList pinned,
                             uint32_t* protected_store_pc, bool is_store_mem) {
  UseScratchRegisterScope temps(this);
  MemOperand dst_op =
        liftoff::GetMemOp(this, &temps, dst_addr, offset_reg, offset_imm);
  if (protected_store_pc) *protected_store_pc = pc_offset();
  #if defined(V8_TARGET_BIG_ENDIAN)
    if (is_store_mem) {
      switch (type.value()) {
        case StoreType::kI64Store8:
        case StoreType::kI32Store8:
          stc(src.gp(), dst_op);
          break;
        case StoreType::kF32Store: {
          MovFloatToInt(r0, src.fp());
          lrvr(r0, r0); //reverse byte
          MovIntToFloat(kScratchDoubleReg, r0); //restore to a double reg
          StoreFloat32(kScratchDoubleReg, dst_op);
          break; 
        }
        case StoreType::kI32Store: {
          strv(src.gp(), dst_op);
          break;
        }
        case StoreType::kI32Store16:{
          strvh(src.gp(), dst_op);
          break;
        }
        case StoreType::kF64Store:{
          lgdr(r0, src.fp());
          lrvgr(r0, r0);
          ldgr(kScratchDoubleReg, r0);
          StoreDouble(kScratchDoubleReg, dst_op);
          break;
        }
        case StoreType::kI64Store:{
          strvg(src.gp(), dst_op);
          break;
        }
        case StoreType::kI64Store32:{
          strv(src.gp(), dst_op);
          break;
        }
        case StoreType::kI64Store16:{
          strvh(src.gp(), dst_op);
          break;
        }
        case StoreType::kS128Store:{
          MemOperand operand = dst_op;
          vlgv(r0, src.fp(), MemOperand(r0, 1),
              Condition(3));
          vlgv(r1, src.fp(), MemOperand(r0, 0),
              Condition(3));
          strvg(r0, operand);
          strvg(r1, MemOperand(operand.rx(), operand.rb(),
                                    operand.offset() + kBitsPerByte));
          break;
        }
        default:
          UNREACHABLE();
      }
    } else {
    switch (type.value()) {
      case StoreType::kI32Store8:
      case StoreType::kI64Store8:
        stc(src.gp(), dst_op);
        break;
      case StoreType::kI32Store16:
      case StoreType::kI64Store16: 
        StoreHalfWord(src.gp(), dst_op);
        break; 
      case StoreType::kI32Store:
      case StoreType::kI64Store32: 
        StoreW(src.gp(), dst_op);
        break;
      case StoreType::kI64Store:  
        StoreP(src.gp(), dst_op);
        break; 
      case StoreType::kF32Store: 
        StoreFloat32(src.fp(), dst_op);
        break;
      case StoreType::kF64Store: 
        StoreDouble(src.fp(), dst_op);
        break; 
      case StoreType::kS128Store:
        vst(src.fp(), dst_op, Condition(0));
        break;
      default:
        UNREACHABLE();
    }
  }
  #else
    switch (type.value()) {
        case StoreType::kI32Store8:
        case StoreType::kI64Store8:
          stc(src.gp(), dst_op);
          break;
        case StoreType::kI32Store16:
        case StoreType::kI64Store16: 
          StoreHalfWord(src.gp(), dst_op);
          break; 
        case StoreType::kI32Store:
        case StoreType::kI64Store32: 
          StoreW(src.gp(), dst_op);
          break;
        case StoreType::kI64Store:  
          StoreP(src.gp(), dst_op);
          break; 
        case StoreType::kF32Store: 
          StoreFloat32(src.fp(), dst_op);
          break;
        case StoreType::kF64Store: 
          StoreDouble(src.fp(), dst_op);
          break; 
        case StoreType::kS128Store:
          vst(src.fp(), dst_op, Condition(0));
          break;
        default:
          UNREACHABLE();
      }
  #endif
}

void LiftoffAssembler::AtomicLoad(LiftoffRegister dst, Register src_addr,
                                  Register offset_reg, uint32_t offset_imm,
                                  LoadType type, LiftoffRegList pinned) {
  bailout(kAtomics, "AtomicLoad");
}

void LiftoffAssembler::AtomicStore(Register dst_addr, Register offset_reg,
                                   uint32_t offset_imm, LiftoffRegister src,
                                   StoreType type, LiftoffRegList pinned) {
  bailout(kAtomics, "AtomicStore");
}

void LiftoffAssembler::LoadCallerFrameSlot(LiftoffRegister dst,
                                           uint32_t caller_slot_idx,
                                           ValueType type) {
  int32_t offset = (caller_slot_idx + 1) * 8;
  switch (type) {
    case kWasmI32: {
      #if defined(V8_TARGET_BIG_ENDIAN)
        l(dst.gp(), MemOperand(fp, offset + 4)); //constexpr int32_t kLowWordOffset = 4;
      #else
        LoadW(dst.gp(), MemOperand(fp, offset)); 
      #endif
      break;}
    case kWasmI64: {
      LoadP(dst.gp(), MemOperand(fp, offset)); //constexpr int32_t kHighWordOffset = 0;
      break;}
    case kWasmF32:{
      #if defined(V8_TARGET_BIG_ENDIAN)
        LoadFloat32(dst.fp(),MemOperand(fp, offset));
      #else
        LoadFloat32(dst.fp(),MemOperand(fp, offset + 4));
      #endif
      break;}
    case kWasmF64:{
      offset = (caller_slot_idx + 1) * 8 ;
      LoadDouble(dst.fp(), MemOperand(fp, offset));
      break;}
    default:
      UNREACHABLE();
  }
}

void LiftoffAssembler::MoveStackValue(uint32_t dst_offset, uint32_t src_offset,
                                      ValueType type) {
  DCHECK_NE(dst_offset, src_offset);
  LiftoffRegister reg = GetUnusedRegister(reg_class_for(type));
  Fill(reg, src_offset, type);
  Spill(dst_offset, reg, type);
}

void LiftoffAssembler::Move(Register dst, Register src, ValueType type) {
  if (type == kWasmI32) {
   lr(dst, src);
  } else {
    DCHECK_EQ(kWasmI64, type);
    TurboAssembler::Move(dst, src);
  }
}

void LiftoffAssembler::Move(DoubleRegister dst, DoubleRegister src,
                            ValueType type) {
  DDCHECK_NE(dst, src);
  if (type == kWasmF32) {
    ler(dst, src);
  } else if (type == kWasmF64) {
    ldr(dst, src);
  } else {
    DCHECK_EQ(kWasmS128, type);
    vlr(dst, src, Condition(0), Condition(0), Condition(0));
    }
}

void LiftoffAssembler::Spill(int offset, LiftoffRegister reg, ValueType type) {
  RecordUsedSpillOffset(offset);
  MemOperand dst = liftoff::GetStackSlot(offset);
  switch (type) {
    case kWasmI32:
      StoreW(reg.gp(), dst);
      break;
    case kWasmI64:
      StoreP(reg.gp(), dst);
      break;
    case kWasmF32:
      StoreFloat32(reg.fp(), dst);
      break;
    case kWasmF64:
      StoreDouble(reg.fp(), dst);
      break;
    case kWasmS128:
        StoreSimd128(reg.fp(), dst);
        break;
    default:
      UNREACHABLE();
  }
}

void LiftoffAssembler::Spill(int offset, WasmValue value) {
  RecordUsedSpillOffset(offset);
  MemOperand dst = liftoff::GetStackSlot(offset);
  UseScratchRegisterScope temps(this);
  Register src = no_reg;

  if (!is_uint12(abs(dst.offset()))){
    src = GetUnusedRegister(kGpReg).gp();
  } else {
    src = temps.Acquire();
  }
  switch (value.type()) {
    case kWasmI32:  {
      mov(src, Operand(value.to_i32()));
      StoreW(src, dst);
      break;
      }
    case kWasmI64: {
      mov(src, Operand(value.to_i64()));
      StoreP(src, dst);
      break;
    }
    default:
      // We do not track f32 and f64 constants, hence they are unreachable.
      UNREACHABLE();
  }
}

void LiftoffAssembler::Fill(LiftoffRegister reg, int offset, ValueType type) {
  switch (type) {
    case kWasmI32:
      LoadW(reg.gp(), MemOperand(fp, -offset));
      break;
    case kWasmI64:
      LoadP(reg.gp(), liftoff::GetStackSlot(offset));
      break;
    case kWasmF32:
      LoadFloat32(reg.fp(), liftoff::GetStackSlot(offset));
      break;
    case kWasmF64:
      LoadDouble(reg.fp(), liftoff::GetStackSlot(offset));
      break;
    case kWasmS128:
        vl(reg.fp(), liftoff::GetStackSlot(offset), Condition(0));
        break;
    default:
      UNREACHABLE();
  }
}

void LiftoffAssembler::FillI64Half(Register, int offset, RegPairHalf) {
  UNREACHABLE();
}

void LiftoffAssembler::FillStackSlotsWithZero(int start, int size) {
  DCHECK_LT(0, size);
  RecordUsedSpillOffset(start + size);

  // We need a zero reg. Always use r0 for that, and push it before to restore
  // its value afterwards.
  push(r0);
  mov(r0, Operand(0));

  if (size <= 12 * kStackSlotSize) {
    uint32_t remainder = size;
    for (; remainder >= kStackSlotSize; remainder -= kStackSlotSize) {
      StoreP(r0, liftoff::GetHalfStackSlot(start + remainder, kLowWord));
      StoreP(r0, liftoff::GetHalfStackSlot(start + remainder, kHighWord));
    }
    DCHECK(remainder == 4 || remainder == 0);
    if (remainder) {
      StoreW(r0, liftoff::GetStackSlot(start + remainder)); //TODO
    }
  } else {
    // Use r3 for start address (inclusive), r4 for end address (exclusive).
    Push(r3, r4);
    SubP(r3, fp, Operand(start+size));
    mov(r4, Operand(size/4));
 
    Label Loop;
    bind(&Loop);
    StoreW(r0, MemOperand(r3));
    lay(r3, MemOperand(r3, 4));
    BranchOnCount(r4, &Loop);

    pop(r4);
    pop(r3);
  }
  pop(r0);
}

#define I32_BINOP(name, instruction)                             \
  void LiftoffAssembler::emit_##name(Register dst, Register lhs, \
                                     Register rhs) {             \
    instruction(dst, lhs, rhs);                                  \
  }
#define I32_BINOP_I(name, instruction)                           \
  I32_BINOP(name, instruction)                                   \
  void LiftoffAssembler::emit_##name(Register dst, Register lhs, \
                                     int32_t imm) {              \
    instruction(dst, lhs, Operand(imm));                         \
  }
#define I64_BINOP(name, instruction)                                           \
  void LiftoffAssembler::emit_##name(LiftoffRegister dst, LiftoffRegister lhs, \
                                     LiftoffRegister rhs) {                    \
    instruction(dst.gp(), lhs.gp(), rhs.gp());                                 \
  }
#define I64_BINOP_I(name, instruction)                                         \
  I64_BINOP(name, instruction)                                                 \
  void LiftoffAssembler::emit_##name(LiftoffRegister dst, LiftoffRegister lhs, \
                                     int32_t imm) {                            \
    instruction(dst.gp(), lhs.gp(), Operand(imm));                             \
  }
#define FP32_BINOP(name, instruction)                                        \
  void LiftoffAssembler::emit_##name(DoubleRegister dst, DoubleRegister lhs, \
                                     DoubleRegister rhs) {                   \
    ler(dst, lhs);                                                           \
    instruction(dst, rhs);                                                   \
  }
#define FP32_UNOP(name, instruction)                                           \
  void LiftoffAssembler::emit_##name(DoubleRegister dst, DoubleRegister src) { \
    instruction(dst,src);                                                      \
  }
#define FP64_BINOP(name, instruction)                                        \
  void LiftoffAssembler::emit_##name(DoubleRegister dst, DoubleRegister lhs, \
                                     DoubleRegister rhs) {                   \
    ldr(dst, lhs);                                                           \
    instruction(dst, rhs);                                                   \
  }
#define FP64_UNOP(name, instruction)                                           \
  void LiftoffAssembler::emit_##name(DoubleRegister dst, DoubleRegister src) { \
    instruction(dst, src);                                                     \
  }
#define I32_SHIFTOP(name, instruction)                           \
  void LiftoffAssembler::emit_##name(Register dst, Register src, \
                                     Register amount) {          \
    LoadRR(r0, src);                                             \
    LoadRR(r1, amount);                                          \
    And(amount, Operand(0x1f));                                  \
    instruction(dst, r0, amount);                                \
    LoadRR(src, r0 );                                            \
    LoadRR(amount, r1);                                          \
  }                                                              \
  void LiftoffAssembler::emit_##name(Register dst, Register src, \
                                     int32_t amount) {           \
    instruction(dst, src, Operand(amount & 31));                 \
  }
#define I64_SHIFTOP(name, instruction)                                         \
  void LiftoffAssembler::emit_##name(LiftoffRegister dst, LiftoffRegister src, \
                                     Register amount) {                        \
    LoadRR(r0, src.gp());                                                      \
    LoadRR(r1, amount);                                                        \
    AndP(amount, Operand(0x3f));                                               \
    instruction(dst.gp(), r0, amount);                                         \
    LoadRR(src.gp(), r0);                                                      \
    LoadRR(amount, r1);                                                        \
  }                                                                            \
  void LiftoffAssembler::emit_##name(LiftoffRegister dst, LiftoffRegister src, \
                                     int32_t amount) {                         \
    instruction(dst.gp(), src.gp(), Operand(amount & 63));                     \
  }

I32_BINOP_I(i32_add, AddP) //Add32
I32_BINOP(i32_sub, Sub32)
I32_BINOP(i32_mul, Mul)
I32_BINOP_I(i32_and, And)
I32_BINOP_I(i32_or, Or)
I32_BINOP_I(i32_xor, Xor)
I32_SHIFTOP(i32_shl, ShiftLeft)
I32_SHIFTOP(i32_sar, ShiftRightArith)
I32_SHIFTOP(i32_shr, ShiftRight)
I64_BINOP_I(i64_add, AddP)
I64_BINOP(i64_sub, SubP)
I64_BINOP(i64_mul, Mul)
#ifdef V8_TARGET_ARCH_S390X
I64_BINOP_I(i64_and, AndP)
I64_BINOP_I(i64_or, OrP)
I64_BINOP_I(i64_xor, XorP)
#endif
I64_SHIFTOP(i64_shl, ShiftLeftP) 
I64_SHIFTOP(i64_sar, ShiftRightArithP)
I64_SHIFTOP(i64_shr, ShiftRightP) 
FP32_BINOP(f32_add, aebr)
FP32_BINOP(f32_sub, sebr)
FP32_BINOP(f32_mul, meebr)
FP32_BINOP(f32_div, debr)
FP32_UNOP(f32_abs, lpebr)
FP32_UNOP(f32_neg, lcebr)
FP32_UNOP(f32_sqrt, sqebr) 
FP64_BINOP(f64_add, adbr)
FP64_BINOP(f64_sub, sdbr)
FP64_BINOP(f64_mul, mdbr)
FP64_BINOP(f64_div, ddbr)
FP64_UNOP(f64_abs, lpdbr)
FP64_UNOP(f64_neg, lcdbr)
FP64_UNOP(f64_sqrt, sqdbr)

#undef I32_BINOP
#undef I32_BINOP_I
#undef I64_BINOP
#undef I64_BINOP_I
#undef UNIMPLEMENTED_GP_UNOP
#undef FP32_BINOP
#undef FP32_UNOP
#undef FP64_BINOP
#undef UNIMPLEMENTED_FP64_UNOP
#undef UNIMPLEMENTED_FP_UNOP_RETURN_TRUE
#undef I32_SHIFTOP
#undef I64_SHIFTOP

void LiftoffAssembler::emit_i32_clz(Register dst, Register src) {
  llgfr(dst, src);
  flogr(r0, dst);
  Add32(dst, r0, Operand(-32));
}

void LiftoffAssembler::emit_i32_ctz(Register dst, Register src) {
  Label cont;
  Label done;
  Cmp32(src, Operand(0));
  bne(&cont);
  lhi(dst, Operand(32));
  beq(&done);

  bind(&cont);
  llgfr(src, src);
  lcgr(r0, src); //r0 two's complement
  ngr(src, r0); // src.gp -> 00000100 
  flogr(r0, src); // xxxx1aa, number of xi
  lghi(r1, Operand(63));
  SubP(dst, r1, r0);
  bind(&done);
}

bool LiftoffAssembler::emit_i32_popcnt(Register dst, Register src) {
  Popcnt32(dst, src);
  return true;
}

bool LiftoffAssembler::emit_i64_popcnt(LiftoffRegister dst,
                                       LiftoffRegister src) {
  Popcnt64(dst.gp(), src.gp());
  return true;
}

bool LiftoffAssembler::emit_f32_ceil(DoubleRegister dst, DoubleRegister src) {
  fiebra(v8::internal::Assembler::FIDBRA_ROUND_TOWARD_POS_INF,
         dst, src);
  return true;
}

bool LiftoffAssembler::emit_f32_floor(DoubleRegister dst, DoubleRegister src) {
  fiebra(v8::internal::Assembler::FIDBRA_ROUND_TOWARD_NEG_INF,
         dst, src);
  return true;
}

bool LiftoffAssembler::emit_f32_trunc(DoubleRegister dst, DoubleRegister src) {
  fiebra(v8::internal::Assembler::FIDBRA_ROUND_TOWARD_0,
         dst, src);
  return true;
}

bool LiftoffAssembler::emit_f32_nearest_int(DoubleRegister dst,
                                            DoubleRegister src) {
  fiebra(v8::internal::Assembler::FIDBRA_ROUND_TO_NEAREST_TO_EVEN,
         dst, src);
  return true;
}

bool LiftoffAssembler::emit_f64_ceil(DoubleRegister dst, DoubleRegister src) {
  fidbra(v8::internal::Assembler::FIDBRA_ROUND_TOWARD_POS_INF,
                                  dst, src);
  return true;
}

bool LiftoffAssembler::emit_f64_floor(DoubleRegister dst, DoubleRegister src) {
  fidbra(v8::internal::Assembler::FIDBRA_ROUND_TOWARD_NEG_INF,
                                  dst, src);
  return true;
}

bool LiftoffAssembler::emit_f64_trunc(DoubleRegister dst, DoubleRegister src) {
  fidbra(v8::internal::Assembler::FIDBRA_ROUND_TOWARD_0,
                                  dst, src);
  return true;
}

bool LiftoffAssembler::emit_f64_nearest_int(DoubleRegister dst,
                                            DoubleRegister src) {
  fidbra(v8::internal::Assembler::FIDBRA_ROUND_TO_NEAREST_TO_EVEN,
                                  dst, src);
  return true;
}

void LiftoffAssembler::emit_f64_min(DoubleRegister dst, DoubleRegister lhs,
                                    DoubleRegister rhs) {
    DoubleMin(dst, lhs, rhs);
}

void LiftoffAssembler::emit_f64_max(DoubleRegister dst, DoubleRegister lhs,
                                    DoubleRegister rhs) {
    DoubleMax(dst, lhs, rhs);
}

void LiftoffAssembler::emit_f32_min(DoubleRegister dst, DoubleRegister lhs,
                                    DoubleRegister rhs) {
  FloatMin(dst, lhs, rhs);
}

void LiftoffAssembler::emit_f32_max(DoubleRegister dst, DoubleRegister lhs,
                                    DoubleRegister rhs) {
  FloatMax(dst, lhs, rhs);
}


void LiftoffAssembler::emit_i32_divs(Register dst, Register lhs, Register rhs,
                                     Label* trap_div_by_zero,
                                     Label* trap_div_unrepresentable) {
  Label cont;
  Cmp32(rhs, Operand(0));
  b(eq, trap_div_by_zero);
  Cmp32(rhs, Operand(-1));
  bne(&cont);
  Cmp32(lhs, Operand(kMinInt));
  b(eq, trap_div_unrepresentable);
  bind(&cont);
  Div32(dst, lhs, rhs);
}

void LiftoffAssembler::emit_i32_divu(Register dst, Register lhs, Register rhs,
                                     Label* trap_div_by_zero) {
  CmpLogical32(rhs, Operand(0));
  beq(trap_div_by_zero);
  DivU32(dst, lhs, rhs);
}

void LiftoffAssembler::emit_i32_rems(Register dst, Register lhs, Register rhs,
                                     Label* trap_div_by_zero) {
  Label cont;
  Label done;
  Label trap_div_unrepresentable;
  // Check for division by zero.
  Cmp32(rhs, Operand(0));
  beq(trap_div_by_zero);

  //Check kMinInt/-1 case.
  Cmp32(rhs, Operand(-1));
  bne(&cont);
  Cmp32(lhs, Operand(kMinInt));
  beq(&trap_div_unrepresentable);

  bind(&cont); //Continue noraml calculation.
  Mod32(dst, lhs, rhs);
  bne(&done);

  bind(&trap_div_unrepresentable);
  mov(dst, Operand(0));
  bind(&done);
}

void LiftoffAssembler::emit_i32_remu(Register dst, Register lhs, Register rhs,
                                     Label* trap_div_by_zero) {
  CmpLogical32(rhs, Operand(0));
  beq(trap_div_by_zero);
  ModU32(dst, lhs,  rhs);
}

bool LiftoffAssembler::emit_i64_divs(LiftoffRegister dst, LiftoffRegister lhs,
                                     LiftoffRegister rhs,
                                     Label* trap_div_by_zero,
                                     Label* trap_div_unrepresentable) {
  iilf(r0, Operand(0));
  iihf(r0, Operand(kMinInt));

  Label cont;
  // Check for division by zero.
  CmpP(rhs.gp(), Operand(0));
  beq(trap_div_by_zero);

  // Check for kMinInt / -1. This is unrepresentable.
  CmpP(rhs.gp(), Operand(-1));
  bne(&cont);
  CmpP(lhs.gp(), r0);
  b(eq, trap_div_unrepresentable);

  bind(&cont);
  Div64(dst.gp(), lhs.gp(), rhs.gp());
  return true;
}

bool LiftoffAssembler::emit_i64_divu(LiftoffRegister dst, LiftoffRegister lhs,
                                     LiftoffRegister rhs,
                                     Label* trap_div_by_zero) {
  CmpLogicalP(rhs.gp(), Operand(0)); 
  b(eq, trap_div_by_zero);
  DivU64(dst.gp(), lhs.gp(), rhs.gp());
  return true;
}

bool LiftoffAssembler::emit_i64_rems(LiftoffRegister dst, LiftoffRegister lhs,
                                     LiftoffRegister rhs,
                                     Label* trap_div_by_zero) {
  iilf(r0, Operand(0));
  iihf(r0, Operand(kMinInt));

  Label cont;
  Label done;
  Label trap_div_unrepresentable;
  // Check for division by zero.
  CmpP(rhs.gp(), Operand(0));
  beq(trap_div_by_zero);

  // Check for kMinInt / -1. This is unrepresentable.
  CmpP(rhs.gp(), Operand(-1));
  bne(&cont);
  CmpP(lhs.gp(), r0);
  b(eq, &trap_div_unrepresentable);

  bind(&cont);
  Mod64(dst.gp(), lhs.gp(), rhs.gp());
  b(&done);

  bind(&trap_div_unrepresentable);
  mov(dst.gp(), Operand(0));
  bind(&done);
  return true;
}

bool LiftoffAssembler::emit_i64_remu(LiftoffRegister dst, LiftoffRegister lhs,
                                     LiftoffRegister rhs,
                                     Label* trap_div_by_zero) {
  // Check for division by zero.
  CmpLogicalP(rhs.gp(), Operand(0)); 
  beq(trap_div_by_zero);
  ModU64(dst.gp(), lhs.gp(), rhs.gp());
  return true;
}

void LiftoffAssembler::emit_i64_clz(LiftoffRegister dst, LiftoffRegister src) {
  flogr(r0, src.gp());
  LoadRR(dst.gp(), r0);
}

void LiftoffAssembler::emit_i64_ctz(LiftoffRegister dst, LiftoffRegister src) {
  Label cont;
  Label done;
  CmpP(src.gp(), Operand(0));
  bne(&cont);
  lghi(dst.gp(), Operand(64));
  beq(&done);

  bind(&cont);
  lcgr(r0, src.gp()); //r0 two's complement
  ngr(src.gp(), r0); // src.gp -> 00000100 
  flogr(r0, src.gp()); // xxxx1aa, number of xi
  lghi(r1, Operand(63));
  SubP(dst.gp(), r1, r0);
  bind(&done);
}

void LiftoffAssembler::emit_u32_to_intptr(Register dst, Register src) {
#ifdef V8_TARGET_ARCH_S390X
  LoadlW(dst, src);
#else
// This is a nop on s390.
#endif
}

void LiftoffAssembler::emit_f32_copysign(DoubleRegister dst, DoubleRegister lhs,
                                         DoubleRegister rhs) {
  constexpr uint64_t kF64SignBit = uint64_t{1} << 63;
  UseScratchRegisterScope temps(this);
  Register scratch2 = temps.Acquire();
  MovDoubleToInt64(r0, lhs); //scratch
  // Clear sign bit in {scratch}.
  AndP(r0, Operand(~kF64SignBit));

  MovDoubleToInt64(scratch2, rhs);
  // Isolate sign bit in {scratch2}.
  AndP(scratch2, Operand(kF64SignBit));
  // Combine {scratch2} into {scratch}.
  OrP(r0, r0, scratch2);
  MovInt64ToDouble(dst, r0);
}

void LiftoffAssembler::emit_f64_copysign(DoubleRegister dst, DoubleRegister lhs,
                                         DoubleRegister rhs) {
  UseScratchRegisterScope temps(this);
  Register scratch2 = temps.Acquire();
  // Extract sign bit from {rhs} into {scratch2}.
  MovDoubleToInt64(scratch2, rhs);
  ShiftRightP(scratch2, scratch2, Operand(63));
  ShiftLeftP(scratch2, scratch2, Operand(63));
  // Reset sign bit of {lhs} (in {scratch}).
  MovDoubleToInt64(r0, lhs);
  ShiftLeftP(r0, r0, Operand(1));
  ShiftRightP(r0, r0, Operand(1));
  // Combine both values into {scratch} and move into {dst}. 
  OrP(r0, r0, scratch2);
  MovInt64ToDouble(dst, r0);
}

bool LiftoffAssembler::emit_type_conversion(WasmOpcode opcode,
                                            LiftoffRegister dst,
                                            LiftoffRegister src, Label* trap) {
  switch (opcode) {
    case kExprI32ConvertI64:
      lgfr(dst.gp(), src.gp());
      return true;
    case kExprI32SConvertF32: {
      ConvertFloat32ToInt32(dst.gp(),
                            src.fp(),
                            kRoundToZero); // f32 -> i32 round to zero.
     b(Condition(1), trap);
      return true;
    }
    case kExprI32UConvertF32: {
      ConvertFloat32ToUnsignedInt32(dst.gp(),
                                    src.fp()); // f32 -> i32 round to zero.
      // Check underflow and NaN.
      b(Condition(1), trap);
      return true;
    }
    case kExprI32SConvertF64: {
      ConvertDoubleToInt32(dst.gp(),
                           src.fp()); // f64 -> i32 round to zero.
      b(Condition(1), trap);
      return true;
    }
    case kExprI32UConvertF64: {
      ConvertDoubleToUnsignedInt32(dst.gp(), src.fp()); // f64 -> i32 round to zero.
      b(Condition(1), trap);
      return true;
    }
    case kExprI32ReinterpretF32:
      //MovFloatToInt(dst.gp(), src.fp());
      lgdr(dst.gp(), src.fp());
      srlg(dst.gp(), dst.gp(), Operand(32));
      return true;
    case kExprI64SConvertI32:
      LoadW(dst.gp(), src.gp());
      return true;
    case kExprI64UConvertI32:
      llgfr(dst.gp(), src.gp());
      //lr(dst.gp(), src.gp());
      return true;
    case kExprI64ReinterpretF64:
      lgdr(dst.gp(), src.fp());
      return true;
    case kExprF32SConvertI32: {
      ConvertIntToFloat(dst.fp(), src.gp());
      return true;
    }
    case kExprF32ConvertF64:
      ledbr(dst.fp(), src.fp());
      return true;
    case kExprF32ReinterpretI32: {
      sllg(r0, src.gp(), Operand(32));
      ldgr(dst.fp(), r0);
      return true;
    }
    case kExprF64SConvertI32: {
      ConvertIntToDouble(dst.fp(), src.gp());
      return true;
    }
    case kExprF64UConvertI32: {
    ConvertUnsignedIntToDouble(dst.fp(), src.gp());
      return true;
    }
    case kExprF64ConvertF32:
      ldebr(dst.fp(), src.fp());
      return true;
    case kExprF64ReinterpretI64:
      ldgr(dst.fp(), src.gp());
      return true;
    case kExprF64SConvertI64:
      ConvertInt64ToDouble(dst.fp(), src.gp());
      return true;
    case kExprF64UConvertI64:
      ConvertUnsignedInt64ToDouble(dst.fp(), src.gp());
      return true;
    case kExprI64SConvertF32: {
      ConvertFloat32ToInt64(dst.gp(), src.fp());  // f32 -> i64 round to zero.
      b(Condition(1), trap);
      return true;
    }
    case kExprI64UConvertF32: {
      Label done;
      ConvertFloat32ToUnsignedInt64(dst.gp(), src.fp()); // f32 -> i64 round to zero.
      b(Condition(1), trap);
      b(Condition(0xE), &done, Label::kNear);  // normal case
      lghi(dst.gp(), Operand::Zero());
      bind(&done);
      return true;
    }
    case kExprF32SConvertI64:
      ConvertInt64ToFloat(dst.fp(), src.gp());
      return true;
    case kExprF32UConvertI64:
      ConvertUnsignedInt64ToFloat(dst.fp(), src.gp());
      return true;
    case kExprI64SConvertF64: {
      ConvertDoubleToInt64(dst.gp(), src.fp()); // f64 -> i64 round to zero.
      b(Condition(1), trap);
      return true;
   }
    case kExprI64UConvertF64: {
      ConvertDoubleToUnsignedInt64(dst.gp(), src.fp());
      b(Condition(1), trap);
      return true;
    }
    default:
      UNREACHABLE();
  }
  return true;
}

void LiftoffAssembler::emit_i32_signextend_i8(Register dst, Register src) {
  lbr(dst, src);
}

void LiftoffAssembler::emit_i32_signextend_i16(Register dst, Register src) {
  lhr(dst, src);
}

void LiftoffAssembler::emit_i64_signextend_i8(LiftoffRegister dst,
                                              LiftoffRegister src) {
  LoadB(dst.gp(), src.gp());
}

void LiftoffAssembler::emit_i64_signextend_i16(LiftoffRegister dst,
                                               LiftoffRegister src) {
  LoadHalfWordP(dst.gp(), src.gp());
}

void LiftoffAssembler::emit_i64_signextend_i32(LiftoffRegister dst,
                                               LiftoffRegister src) {
  LoadW(dst.gp(), src.gp());
}

void LiftoffAssembler::emit_jump(Label* label) {
  b(al, label);
}

void LiftoffAssembler::emit_jump(Register target) {
  Jump(target);
}

void LiftoffAssembler::emit_cond_jump(Condition cond, Label* label,
                                      ValueType type, Register lhs,
                                      Register rhs) {
  Condition liftoff_cond = liftoff::LiftoffCondToCond(cond);
  switch (type) {
    case kWasmI32: {
      if (rhs == no_reg) {
        if (liftoff_cond == cond) {
           CmpLogical32(lhs, Operand(0));
        } else {
           Cmp32(lhs, Operand(0));
        }
      } else {
        if (liftoff_cond == cond) {
           CmpLogical32(lhs, rhs);
        } else {
           Cmp32(lhs, rhs);
        }
      }
      break; }
    case kWasmI64: {
      if (rhs == no_reg) {
        if (liftoff_cond == cond) {
           CmpLogicalP(lhs, Operand(0));
        } else {
           CmpP(lhs, Operand(0));
        }

      } else {
        if (liftoff_cond == cond) {
           CmpLogicalP(lhs, rhs);
        } else {
           CmpP(lhs, rhs);
        }
      }
      break; }
    default:
      UNREACHABLE();
  }
  b(liftoff_cond, label);
}

void LiftoffAssembler::emit_i32_eqz(Register dst, Register src) {
  emit_i32_clz(dst, src);
  ShiftRight(dst, dst, Operand(5));
}

void LiftoffAssembler::emit_i32_set_cond(Condition cond, Register dst,
                                         Register lhs, Register rhs) {
  Condition liftoff_cond = liftoff::LiftoffCondToCond(cond);
  if (liftoff_cond != cond){
     //Sign Comparaion.
     Cmp32(lhs, rhs);
  } else {
     CmpLogical32(lhs, rhs);
  }
  mov(dst, Operand(0));
  mov(r0, Operand(1));
  locr(liftoff_cond, dst, r0);
}

void LiftoffAssembler::emit_i64_eqz(Register dst, LiftoffRegister src) {
  Label done;
  mov(dst, Operand(0));
  CmpP(src.gp(), Operand(0));
  bne(&done);
  mov(dst, Operand(1));
  bind(&done);
}

void LiftoffAssembler::emit_i64_set_cond(Condition cond, Register dst,
                                         LiftoffRegister lhs,
                                         LiftoffRegister rhs) {
  Condition liftoff_cond = liftoff::LiftoffCondToCond(cond);
  UseScratchRegisterScope temps(this);
  Register scratch = temps.Acquire();
  if (liftoff_cond == cond) {
     CmpLogicalP(lhs.gp(), rhs.gp());
  } else {
     CmpP(lhs.gp(), rhs.gp());
  }
  mov(dst, Operand(0));
  mov(scratch, Operand(1));
  LoadOnConditionP(liftoff_cond, dst, scratch);
}

void LiftoffAssembler::emit_f32_set_cond(Condition cond, Register dst,
                                         DoubleRegister lhs,
                                         DoubleRegister rhs) {
  cond = liftoff::LiftoffCondToCond(cond);
  UseScratchRegisterScope temps(this);
  Register scratch = temps.Acquire();
  mov(dst, Operand(0));
  mov(scratch, Operand(1));
  cebr(lhs, rhs);
  LoadOnConditionP(cond, dst, scratch);
  if (cond != ne){
      mov(scratch, Operand(0));
      LoadOnConditionP(overflow, dst, scratch);
  }
}

void LiftoffAssembler::emit_f64_set_cond(Condition cond, Register dst,
                                         DoubleRegister lhs,
                                         DoubleRegister rhs) {
  cond = liftoff::LiftoffCondToCond(cond);
  UseScratchRegisterScope temps(this);
  Register scratch = temps.Acquire();
  mov(dst, Operand(0));
  mov(scratch, Operand(1));
  cdbr(lhs, rhs);
  LoadOnConditionP(cond, dst, scratch);
  if (cond != ne){
      mov(scratch, Operand(0));
      LoadOnConditionP(overflow, dst, scratch);
  }
}

void LiftoffAssembler::StackCheck(Label* ool_code, Register limit_address) {
  LoadP(limit_address, MemOperand(limit_address));
  CmpLogicalP(sp, limit_address);
  b(le, ool_code);
}

void LiftoffAssembler::CallTrapCallbackForTesting() {
  PrepareCallCFunction(0, 0,  no_reg);
  CallCFunction(ExternalReference::wasm_call_trap_callback_for_testing(), 0);
}

void LiftoffAssembler::AssertUnreachable(AbortReason reason) {
  TurboAssembler::AssertUnreachable(reason);
}

void LiftoffAssembler::PushRegisters(LiftoffRegList regs) {
  MultiPush(regs.GetGpList());
  MultiPushDoubles(regs.GetFpList());
}

void LiftoffAssembler::PopRegisters(LiftoffRegList regs) {
  MultiPop(regs.GetGpList());
  MultiPopDoubles(regs.GetFpList());
}

void LiftoffAssembler::DropStackSlotsAndRet(uint32_t num_stack_slots) {
  Drop(num_stack_slots);
  Ret();
}

void LiftoffAssembler::CallC(wasm::FunctionSig* sig,
                             const LiftoffRegister* args,
                             const LiftoffRegister* rets,
                             ValueType out_argument_type, int stack_bytes,
                             ExternalReference ext_ref) {
  // Reserve space in the stack.
  lay(sp, MemOperand(sp, -stack_bytes));

  int arg_bytes = 0;
  for (ValueType param_type : sig->parameters()) {
    switch (param_type) {
      case kWasmI32:
        StoreW(args->gp(), MemOperand(sp, arg_bytes));
        break;
      case kWasmI64:
        StoreP(args->gp(), MemOperand(sp, arg_bytes));
        break;
      case kWasmF32:
        StoreFloat32(args->fp(), MemOperand(sp, arg_bytes));
        break;
      case kWasmF64:
        StoreDouble(args->fp(), MemOperand(sp, arg_bytes));
        break;
      default:
        UNREACHABLE();
    }
    args++;
    arg_bytes += ValueTypes::MemSize(param_type);
  }
  DCHECK_LE(arg_bytes, stack_bytes);

  // Pass a pointer to the buffer with the arguments to the C function.
  LoadRR(r2, sp);

  // Now call the C function.
  constexpr int kNumCCallArgs = 1;
  PrepareCallCFunction(kNumCCallArgs, no_reg);
  CallCFunction(ext_ref, kNumCCallArgs);

  // Move return value to the right register.
  const LiftoffRegister* result_reg = rets;
  if (sig->return_count() > 0) {
    DCHECK_EQ(1, sig->return_count());
    constexpr Register kReturnReg = r2;
    if (kReturnReg != rets->gp()) {
      Move(*rets, LiftoffRegister(kReturnReg), sig->GetReturn(0));
    }
    result_reg++;
  }

  // Load potential output value from the buffer on the stack.
  if (out_argument_type != kWasmStmt) {
    switch (out_argument_type) {
      case kWasmI32:
        LoadW(result_reg->gp(), MemOperand(sp));
        break;
      case kWasmI64:
        LoadP(result_reg->gp(), MemOperand(sp));
        break;
      case kWasmF32:
        LoadFloat32(result_reg->fp(), MemOperand(sp));
        break;
      case kWasmF64:
        LoadDouble(result_reg->fp(), MemOperand(sp));
        break;
      default:
        UNREACHABLE();
    }
  }
  lay(sp, MemOperand(sp, stack_bytes));
}

void LiftoffAssembler::CallNativeWasmCode(Address addr) {
  Call(addr, RelocInfo::WASM_CALL);
}

void LiftoffAssembler::CallIndirect(wasm::FunctionSig* sig,
                                    compiler::CallDescriptor* call_descriptor,
                                    Register target) {
  DCHECK(target != no_reg);
  Call(target);
}

void LiftoffAssembler::CallRuntimeStub(WasmCode::RuntimeStubId sid) {
  // A direct call to a wasm runtime stub defined in this module.
  // Just encode the stub index. This will be patched at relocation.
  Call(static_cast<Address>(sid), RelocInfo::WASM_STUB_CALL);
}

void LiftoffAssembler::AllocateStackSlot(Register addr, uint32_t size) {
  size = RoundUp(size, 16);
  lay(sp, MemOperand(sp, -size));
  TurboAssembler::Move(addr, sp);
}

void LiftoffAssembler::DeallocateStackSlot(uint32_t size) {
  size = RoundUp(size, 16);
  lay(sp, MemOperand(sp, size));
}

void LiftoffAssembler::DebugBreak() { stop(); }

void LiftoffStackSlots::Construct() {
  for (auto& slot : slots_) {
    const LiftoffAssembler::VarState& src = slot.src_;
    switch (src.loc()) {
      case LiftoffAssembler::VarState::kStack: {
        switch (src.type()) {
          case kWasmI32: {
            UseScratchRegisterScope temps(asm_);
            Register scratch = temps.Acquire();
            asm_->LoadW(scratch, liftoff::GetStackSlot(slot.src_offset_)); 
            asm_->Push(scratch);
            break;
          }
          case kWasmI64: {
            UseScratchRegisterScope temps(asm_);
            Register scratch = temps.Acquire();
            asm_->LoadP(scratch, liftoff::GetStackSlot(slot.src_offset_));
            asm_->Push(scratch);
            break;
          }
          case kWasmF32: {
            asm_->LoadFloat32(kScratchDoubleReg,
                      liftoff::GetStackSlot(slot.src_offset_));
            asm_->push(kScratchDoubleReg);
            break;
          }
          case kWasmF64: {
            asm_->LoadDouble(kScratchDoubleReg, liftoff::GetStackSlot(slot.src_offset_));
            asm_->push(kScratchDoubleReg);
            break;
          }
          default:
            UNREACHABLE();
        }
        break;
      }
      case LiftoffAssembler::VarState::kRegister:
        switch (src.type()) {
          case kWasmI64:
          case kWasmI32:
            asm_->push(src.reg().gp());
            break;
          case kWasmF32:
            asm_->push(src.reg().fp());
            break;
          case kWasmF64:
            asm_->push(src.reg().fp());
            break;
          default:
            UNREACHABLE();
        }
        break;
      case LiftoffAssembler::VarState::kIntConst: {
        DCHECK(src.type() == kWasmI32 || src.type() == kWasmI64);
        UseScratchRegisterScope temps(asm_);
        Register scratch = temps.Acquire();

        switch (src.type()) {
            case kWasmI32 : {
                asm_->mov(scratch, Operand(src.i32_const()));
                break;
            }
            case kWasmI64: {
                asm_->mov(scratch, Operand(int64_t{slot.src_.i32_const()}));
                break;
            }
            default:
              UNREACHABLE();
            }
        asm_->push(scratch);
        break;
      }
    }
  }
}

}  // namespace wasm
}  // namespace internal
}  // namespace v8

#undef BAILOUT

#endif  // V8_WASM_BASELINE_S390_LIFTOFF_ASSEMBLER_S390_H_
