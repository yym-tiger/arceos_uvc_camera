#![no_std]
#![no_main]
#![allow(warnings)]

use axalloc::GlobalNoCacheAllocator;
use axhal::{mem::VirtAddr, paging::PageSize};
use driver_usb::{USBSystem, USBSystemConfig};

#[macro_use]
extern crate axstd as std;

#[derive(Clone)]
struct PlatformAbstraction;

impl driver_usb::abstractions::OSAbstractions for PlatformAbstraction {
    type VirtAddr = VirtAddr;
    type DMA = GlobalNoCacheAllocator;

    const PAGE_SIZE: usize = PageSize::Size4K as usize;

    fn dma_alloc(&self) -> Self::DMA {
        axalloc::global_no_cache_allocator()
    }

    fn send_event(&self, event: driver_usb::abstractions::event::USBSystemEvent) {
        println!("收到USB事件");
    }
}

impl driver_usb::abstractions::HALAbstractions for PlatformAbstraction {
    fn force_sync_cache() {}
}

#[no_mangle]
fn main() {
    println!("初始化USB-UVC摄像头系统...");

    let mut usbsystem = driver_usb::USBSystem::new({
        USBSystemConfig::new(0xffff_0000_31a0_8000, 48, 0, PlatformAbstraction)
    })
    .init()
    .init_probe();

    println!("USB系统初始化完成");

    // 驱动所有设备
    println!("开始驱动所有USB设备...");
    usbsystem.drive_all();

    println!("USB-UVC摄像头驱动启动完成");

    // 等待一段时间让设备初始化
    println!("等待摄像头初始化...");
    for i in (1..=5).rev() {
        println!("倒计时 {} 秒...", i);
        axhal::time::busy_wait(core::time::Duration::from_secs(1));
    }

    // 尝试获取摄像头状态
    println!("检查摄像头状态...");

    // 在主线程中运行一段时间
    println!("USB-UVC系统运行中...");
    for _ in 0..10 {
        axhal::time::busy_wait(core::time::Duration::from_secs(1));
        println!(".");
    }

    println!("USB-UVC测试完成");
}
