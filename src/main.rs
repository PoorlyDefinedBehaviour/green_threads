#![feature(asm)]
use core::arch::asm;

const STACK_SIZE_IN_BYTES: isize = 48;

/// Represents the CPU state.
///
/// This needs to be #[repr(C)] because
/// Rust doesn't have a stable ABI while C does
/// and we need to be sure that `rsp` will be represented
/// in memory as the first 8 bytes.
#[derive(Debug, Default)]
#[repr(C)]
struct ThreadContext {
  // The stack pointer.
  rsp: u64,
}

fn hello() -> ! {
  println!("hello");

  loop {}
}

/// Sets the stack pointer to the address of `new`
/// and transfers control to the function thats on top
/// of the stack.
///
/// `new` will be the address of the function we want to run.
unsafe fn gt_switch(new: *const ThreadContext) {
  asm!(
    "mov rsp, [{0} + 0x00]",
    "ret",
    in(reg) new,
  );
}

fn main() {
  let mut ctx = ThreadContext::default();

  let mut stack = vec![0_u8; STACK_SIZE_IN_BYTES as usize];

  unsafe {
    let stack_bottom = stack.as_mut_ptr().offset(STACK_SIZE_IN_BYTES);
    let sb_aligned = (stack_bottom as usize & !15) as *mut u8;
    std::ptr::write(sb_aligned.offset(-16) as *mut u64, hello as u64);
    ctx.rsp = sb_aligned.offset(-16) as u64;
    gt_switch(&mut ctx);
  }
}
