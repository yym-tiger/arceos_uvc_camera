# USB UVC摄像头驱动使用方法

## 快速开始

1. **编译方法**：
   ```bash
   make A=apps/usb-hid PLATFORM=aarch64-phytium-pi LOG=info chainboot
   ```

2. **查看日志**：
   会在当前目录下生成`minicom_output.log`日志文件，日志文件包含了摄像头数据帧

3. **提取图像**：
   ```bash
   python3 extract_frame_from_log.py minicom_output.log
   ```
   生成的照片会保存在当前目录下`frame1.jpg`（下面就是解析后正常显示的图片）
   ![alt text](image.png)

## 1. USB UVC驱动实现原理

### 1.1 驱动架构概述

ArceOS的USB UVC驱动采用分层架构设计：

```
应用层 (apps/usb-uvc)
    ↓
UVC类驱动层 (uvc_drivers/generic_uvc.rs)
    ↓
USB传输层 (transfer/control.rs, isoch.rs)
    ↓
XHCI主机控制器层 (xhci/mod.rs)
    ↓
硬件抽象层 (DMA allocator)
```

### 1.2 核心组件

#### 1.2.1 GenericUVCDriver

主驱动结构体，负责：
- 设备枚举和初始化
- 视频流控制
- 数据接收和处理
- 状态机管理

```rust
pub struct GenericUVCDriver<O> {
    // 设备信息
    device_slotid: usize,
    vendor_id: u16,
    device_id: u16,
    
    // 图像缓冲区
    image_buffer: Option<SpinNoIrq<DMA<[u8], O::DMA>>>,
    buffer_state: BufferState,
    
    // 摄像头状态
    camera_state: UVCCameraState,
    
    // 视频流配置
    video_format: u8,      // MJPEG格式
    resolution_index: u8,  // 320x240分辨率
    frame_interval: u32,   // 30fps帧率
    
    // 等时传输
    isoc_urbs: Vec<IsocURBInfo<O>>,
    frame_processor: MjpegFrameProcessor,
}
```

#### 1.2.2 状态机管理
摄像头初始化分为多个阶段：

```rust
pub enum UVCInitStage {
    NotStarted,              // 未开始
    DeviceProbed,           // 设备已探测
    VideoControlSet,        // 视频控制已设置
    AltInterfaceSet,        // 接口已设置
    StreamingStarted,       // 流传输已开始
    Completed,              // 初始化完成
}
```

#### 1.2.3 等时传输（Isochronous Transfer）
用于实时视频数据传输：

```rust
pub struct IsochTransfer {
    endpoint_id: usize,
    buffer_addr_len: (usize, usize),
    num_packets: usize,      // 包数量
    packet_size: usize,      // 包大小
    timeout_ms: u64,         // 超时时间
}
```

### 1.3 数据流处理

#### 1.3.1 MJPEG帧处理器
负责从USB数据流中提取完整的JPEG帧：

```rust
pub struct MjpegFrameProcessor {
    current_state: FrameState,
    frame_buffer: Vec<u8>,
    statistics: FrameStatistics,
}
```

帧处理状态：
- `WaitingForHeader`: 等待JPEG帧头(0xFFD8)
- `CollectingData`: 收集帧数据
- `FrameComplete`: 帧接收完成(检测到0xFFD9)

#### 1.3.2 数据回调机制
通过全局缓冲区和回调函数处理视频数据：

```rust
pub static VIDEO_STREAM_BUFFER: SpinNoIrq<Option<Vec<u8>>> = SpinNoIrq::new(None);
pub type DataCallback = fn(&[u8]);
```

### 1.4 初始化流程

1. **设备枚举**：识别USB摄像头设备
2. **获取描述符**：解析设备、配置、接口描述符
3. **设置配置**：选择合适的配置和接口
4. **协商参数**：通过PROBE/COMMIT控制传输设置视频参数
5. **启动流传输**：创建等时传输URB并开始接收数据

## 2. 使用方法

### 2.1 基本使用示例

```rust
#![no_std]
#![no_main]

use driver_usb::{USBSystem, USBSystemConfig};

fn main() {
    // 1. 创建USB系统配置
    let config = USBSystemConfig::new(
        0xffff_0000_31a0_8000,  // XHCI基地址
        48,                      // 中断号
        0,                       // 标志
        PlatformAbstraction      // 平台抽象
    );
    
    // 2. 初始化USB系统
    let mut usb_system = USBSystem::new(config)
        .init()
        .init_probe();
    
    // 3. 驱动所有USB设备
    usb_system.drive_all();
    
    // 4. 等待摄像头初始化
    axhal::time::busy_wait(Duration::from_secs(5));
}
```

### 2.2 平台抽象实现

```rust
struct PlatformAbstraction;

impl driver_usb::abstractions::OSAbstractions for PlatformAbstraction {
    type VirtAddr = VirtAddr;
    type DMA = GlobalNoCacheAllocator;
    
    const PAGE_SIZE: usize = PageSize::Size4K as usize;
    
    fn dma_alloc(&self) -> Self::DMA {
        axalloc::global_no_cache_allocator()
    }
    
    fn send_event(&self, event: USBSystemEvent) {
        // 处理USB事件
        println!("USB事件: {:?}", event);
    }
}
```

### 2.3 获取视频数据

视频数据通过全局缓冲区获取：

```rust
use driver_usb::usb::universal_drivers::uvc_drivers::generic_uvc::VIDEO_STREAM_BUFFER;

// 检查是否有新帧
if let Some(frame_data) = VIDEO_STREAM_BUFFER.lock().as_ref() {
    println!("收到新帧，大小: {} 字节", frame_data.len());
    
    // 处理JPEG数据
    process_jpeg_frame(frame_data);
}
```

### 2.4 自定义数据处理

可以注册自定义回调函数处理视频数据：

```rust
fn my_frame_handler(data: &[u8]) {
    // 验证JPEG格式
    if data.len() > 2 && data[0] == 0xFF && data[1] == 0xD8 {
        println!("收到有效JPEG帧");
        // 保存或处理帧数据
    }
}
```


### 3.2 运行要求

- 硬件：飞腾派开发板
- 摄像头：支持UVC协议的USB摄像头
- 连接方式：摄像头连接第一个USB3.0端口

### 3.3 调试方法

1. **查看串口日志**
```bash
# 通过minicom查看输出
minicom -D /dev/ttyUSB0 -b 115200
```

2. **检查设备识别**
日志中应显示：
```
USB设备已找到: VendorID=xxxx, ProductID=xxxx
UVC摄像头已识别
```

3. **验证数据接收**
```
收到等时传输数据: xxx 字节
MJPEG帧完成: 帧号=x, 大小=xxxxx字节
```

## 4. 高级功能

### 4.1 异步传输支持

驱动支持异步传输模式，提高性能：

```rust
// 创建异步传输
let transfer_future = driver.submit_async_transfer(
    AsyncTransferId::new(),
    transfer_request
);

// 等待传输完成
let result = transfer_future.await;
```

### 4.2 多缓冲区管理

使用多个URB实现流畅的视频流：

```rust
const NUM_URBS: usize = 8;  // 8个URB循环使用
const URB_SIZE: usize = 0x6000;  // 每个URB 24KB
```

## 6. 扩展开发

### 6.1 添加新分辨率

修改`generic_uvc.rs`中的配置：

```rust
// 添加新的分辨率支持
const SUPPORTED_RESOLUTIONS: &[(u16, u16)] = &[
    (320, 240),   // 现有
    (640, 480),   // 新增
    (1280, 720),  // 新增
];
```

### 6.2 支持其他视频格式

扩展帧处理器支持YUV等格式：

```rust
enum VideoFormat {
    MJPEG,
    YUV422,
    H264,
}
```

### TODO
- 异步跟中断有待完善
- URB提交速度太慢，取数据过慢，导致数据被污染
- 添加设备兼容性列表（需要修改端点号，目前支持2款UVC摄像头（VID=0bda, PID=5856））
- 添加其他视频格式支持
- 添加音频格式支持
- 摄像头接口封装
- 实现亮度、对比度等摄像头控制
- 实现零拷贝数据传输
- 实现USB设备热插拔支持
- 改进代码模块化
  