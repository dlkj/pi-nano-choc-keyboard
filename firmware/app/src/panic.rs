use core::panic::PanicInfo;
use core::sync::atomic::{self, Ordering};
use log::error;

#[inline(never)]
#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    error!("{}", info);

    loop {
        atomic::compiler_fence(Ordering::SeqCst);
    }
}
