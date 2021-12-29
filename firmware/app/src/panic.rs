use core::fmt::Write;
use core::panic::PanicInfo;
use core::sync::atomic::{self, Ordering};
use log::error;

#[inline(never)]
#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    error!("{}", info);

    let mut output = arrayvec::ArrayString::<1024>::new();
    if write!(&mut output, "{}", info).ok().is_some() {

    }

    loop {
        atomic::compiler_fence(Ordering::SeqCst);
    }
}
