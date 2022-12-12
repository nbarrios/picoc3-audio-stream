use core::cell::RefCell;
use cortex_m::interrupt::Mutex;
use heapless::spsc::Queue;
use rp2040_hal::pac::{self};

pub struct UartQueue {
    pub mutex_cell_queue: Mutex<RefCell<Queue<u8, 64>>>,
    pub interrupt: pac::Interrupt,
    pub force_interrupt: bool,
}

impl UartQueue {
    pub const fn new(interrupt: pac::Interrupt, force: bool) -> Self {
        Self {
            mutex_cell_queue: Mutex::new(RefCell::new(Queue::new())),
            interrupt: interrupt,
            force_interrupt: force,
        }
    }

    pub fn is_empty(&self) -> bool {
        cortex_m::interrupt::free(|cs| {
            let cell_queue = self.mutex_cell_queue.borrow(cs);
            let queue = cell_queue.borrow();
            queue.is_empty()
        })
    }

    pub fn read_byte(&self) -> Option<u8> {
        cortex_m::interrupt::free(|cs| {
            let cell_queue = self.mutex_cell_queue.borrow(cs);
            let mut queue = cell_queue.borrow_mut();
            queue.dequeue()
        })
    }

    pub fn peek_byte(&self) -> Option<u8> {
        cortex_m::interrupt::free(|cs| {
            let cell_queue = self.mutex_cell_queue.borrow(cs);
            let queue = cell_queue.borrow();
            queue.peek().cloned()
        })
    }

    pub fn write_bytes_blocking(&self, data: &[u8]) {
        for byte in data.iter() {
            let mut written = false;
            while !written {
                cortex_m::interrupt::free(|cs| {
                    let cell_queue = self.mutex_cell_queue.borrow(cs);
                    let mut queue = cell_queue.borrow_mut();
                    if queue.enqueue(*byte).is_ok() {
                        written = true;
                    }
                });
            }
        }

        if self.force_interrupt {
            pac::NVIC::pend(self.interrupt);
        }
    }
}

impl core::fmt::Write for &UartQueue {
    fn write_str(&mut self, data: &str) -> core::fmt::Result {
        self.write_bytes_blocking(data.as_bytes());
        Ok(())
    }
}
