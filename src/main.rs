#![feature(asm)]
#![feature(naked_functions)]
use core::arch::asm;

const DEFAULT_STACK_SIZE: usize = 1024 * 1024 * 2;
const MAX_THREADS: usize = 4;
/// Pointer to the current runtime.
static mut RUNTIME: usize = 0;

/// Manages and switches between threads.
struct Runtime {
  // The current thread.
  current: usize,
  threads: Vec<Thread>,
}

impl Runtime {
  fn new() -> Self {
    // We create a thread in the running state to ensure
    // the runtime will keep running until all tasks are finished.
    let base_thread = Thread {
      id: 0,
      stack: vec![0_u8; DEFAULT_STACK_SIZE],
      ctx: ThreadContext::default(),
      state: State::Running,
    };

    let mut threads = Vec::with_capacity(MAX_THREADS);

    threads.push(base_thread);

    for i in 1..MAX_THREADS {
      threads.push(Thread::new(i));
    }

    Self {
      current: 0,
      threads,
    }
  }

  fn init(&self) {
    unsafe {
      let ptr = self as *const Runtime;
      RUNTIME = ptr as usize;
    }
  }

  fn run(&mut self) -> ! {
    while self.thread_yield() {}

    std::process::exit(0);
  }

  /// Returns false when there is no more work to do.
  #[inline(never)]
  fn thread_yield(&mut self) -> bool {
    let mut position = self.current;

    while self.threads[position].state != State::Ready {
      position += 1;

      if position == self.threads.len() {
        position = 0;
      }

      if position == self.current {
        return false;
      }
    }

    if self.threads[self.current].state != State::Available {
      self.threads[self.current].state = State::Ready;
    }

    self.threads[position].state = State::Running;

    let old_position = self.current;

    self.current = position;

    unsafe {
      let old_ctx: *mut ThreadContext = &mut self.threads[old_position].ctx;
      let new_ctx: *const ThreadContext = &self.threads[position].ctx;

      // On Linux, the register holds the first argument to a function
      // and rsi holds the second argument.
      asm!("call switch", in("rdi") old_ctx, in("rsi") new_ctx, clobber_abi("C"));
    }

    self.threads.len() > 0
  }

  /// Called when a thread is finished.
  ///
  /// Marks the current thread as available and yields to another thread.
  fn thread_return(&mut self) {
    if self.current == 0 {
      return;
    }

    self.threads[self.current].state = State::Available;
    self.thread_yield();
  }

  fn spawn(&mut self, f: fn()) {
    let thread = self
      .threads
      .iter_mut()
      .find(|thread| thread.state == State::Available)
      .expect("no threads available");

    println!("spawn thread={}", thread.id);

    let size = thread.stack.len();

    unsafe {
      let stack_ptr = thread.stack.as_mut_ptr().offset(size as isize);

      // Align the memory segment to 16 bytes.
      let stack_ptr = (stack_ptr as usize & !15) as *mut u8;

      // Make sure stack matches the one specified in the psABI stack layout.
      // https://cfsamson.gitbook.io/green-threads-explained-in-200-lines-of-rust/the-stack#how-to-set-up-the-stack
      std::ptr::write(stack_ptr.offset(-16) as *mut u64, guard as u64);
      std::ptr::write(stack_ptr.offset(-24) as *mut u64, skip as u64);
      std::ptr::write(stack_ptr.offset(-32) as *mut u64, f as u64);

      thread.ctx.rsp = stack_ptr.offset(-32) as u64;
    }

    thread.state = State::Ready;
  }
}

/// Called when a thread is finished running its task.
fn guard() {
  unsafe {
    let runtime_ptr = RUNTIME as *mut Runtime;
    (*runtime_ptr).thread_return();
  }
}

/// We use the #[naked] attribute so that this function
/// essentially compiles down to just the ret instruction.
///
/// ret will pop off the next value from the stack and jump to whatever
/// instructions that address points to.
#[naked]
unsafe extern "C" fn skip() {
  asm!("ret", options(noreturn));
}

/// Helper that allows us to yield the current thread from anywhere in our code.
fn yield_thread() {
  unsafe {
    let runtime_ptr = RUNTIME as *mut Runtime;
    (*runtime_ptr).thread_yield();
  }
}

#[naked]
#[no_mangle]
unsafe extern "C" fn switch() {
  asm!(
    "mov [rdi + 0x00], rsp",
    "mov [rdi + 0x08], r15",
    "mov [rdi + 0x10], r14",
    "mov [rdi + 0x18], r13",
    "mov [rdi + 0x20], r12",
    "mov [rdi + 0x28], rbx",
    "mov [rdi + 0x30], rbp",
    "mov rsp, [rsi + 0x00]",
    "mov r15, [rsi + 0x08]",
    "mov r14, [rsi + 0x10]",
    "mov r13, [rsi + 0x18]",
    "mov r12, [rsi + 0x20]",
    "mov rbx, [rsi + 0x28]",
    "mov rbp, [rsi + 0x30]",
    "ret",
    options(noreturn)
  );
}

struct Thread {
  id: usize,
  stack: Vec<u8>,
  ctx: ThreadContext,
  state: State,
}

impl Thread {
  /// Creates a new Thread.
  ///
  /// New threads always start in the available state because
  /// they are ready to pick up a task.
  fn new(id: usize) -> Self {
    Self {
      id,
      stack: vec![0_u8; DEFAULT_STACK_SIZE],
      ctx: ThreadContext::default(),
      state: State::Available,
    }
  }
}

/// Holds data for the registeres that the CPU
/// needs to resume execution on a stack.
///
/// This needs to be #[repr(C)] because
/// Rust doesn't have a stable ABI while C does
/// and we need to be sure that `rsp` will be represented
/// in memory as the first 8 bytes.
#[derive(Debug, Default)]
#[repr(C)]
struct ThreadContext {
  /// The stack pointer.
  rsp: u64,
  r15: u64,
  r14: u64,
  r13: u64,
  r12: u64,
  rbx: u64,
  rbp: u64,
}

/// The thread state.
#[derive(PartialEq, Eq, Debug)]
enum State {
  /// The thread is available and ready to be assigned a task.
  Available,
  /// The thread is running.
  Running,
  /// The thread is ready to resume execution.
  Ready,
}

fn main() {
  let mut runtime = Runtime::new();

  runtime.init();

  runtime.spawn(|| {
    let id = 1;

    println!("THREAD {} STARTING", id);

    for i in 0..10 {
      println!("thread: {} counter: {}", id, i);
      yield_thread();
    }

    println!("THREAD {} FINISHED", id);
  });

  runtime.spawn(|| {
    let id = 2;

    println!("THREAD {} STARTING", id);

    for i in 0..15 {
      println!("thread: {} counter: {}", id, i);
      yield_thread();
    }

    println!("THREAD {} FINISHED", id);
  });

  runtime.run();
}
