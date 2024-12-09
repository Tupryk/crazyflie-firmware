#![no_std]

use panic_halt as _;

extern "C" {
    pub fn vTaskDelay(ticks: u32);
    pub fn consolePutchar(ch: i32) -> i32;
}

// external parameters
extern "C" {
    static app_param1: u8;
}

// external logging values
extern "C" {
    static mut app_log1: u8;
    static mut app_log2: f32;
}

fn console_print(msg: &str) {
    for c in msg.as_bytes() {
        unsafe{ consolePutchar(*c as i32); }
    }
}

#[no_mangle]
pub extern "C" fn appMain() -> i32 {
    console_print("Hello from Rust!\n");

    loop {
        // Example on how to use parameters and logging from Rust
        unsafe {
            if app_param1 == 1 {
                app_log1 += 1;
            } else {
                app_log2 += 1.0;
            }
        }

        unsafe { vTaskDelay(1000); }
    }
}
