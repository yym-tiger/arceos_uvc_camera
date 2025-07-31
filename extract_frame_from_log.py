#!/usr/bin/env python3
"""
从UVC摄像头驱动日志中提取MJPEG帧数据并保存为JPEG文件

使用方法:
    python3 extract_frame_from_log.py <日志文件> [帧号]
    
示例:
    python3 extract_frame_from_log.py minicom_output.log     # 提取第一帧
    python3 extract_frame_from_log.py minicom_output.log 100 # 提取第100帧
"""

import sys
import re

def extract_frame_data(log_file, frame_number=1):
    """从日志文件中提取指定帧的十六进制数据"""
    
    print(f"正在从 {log_file} 中提取帧 #{frame_number} 的数据...")
    
    with open(log_file, 'r', encoding='utf-8', errors='ignore') as f:
        content = f.read()
    
    # 查找指定帧的开始
    pattern = rf"====== MJPEG帧数据 \(帧#{frame_number}\) ======"
    match = re.search(pattern, content)
    
    if not match:
        print(f"错误: 未找到帧 #{frame_number}")
        return None
    
    # 从匹配位置开始提取
    start_pos = match.end()
    
    # 查找帧数据结束标记
    end_pattern = r"====== 帧数据结束 ======"
    end_match = re.search(end_pattern, content[start_pos:])
    
    if not end_match:
        print("错误: 未找到帧数据结束标记")
        return None
    
    end_pos = start_pos + end_match.start()
    
    # 提取帧数据部分
    frame_data_section = content[start_pos:end_pos]
    
    # 提取十六进制数据（格式: 地址: XX XX XX XX ... |ASCII|）
    hex_pattern = r"[0-9A-Fa-f]{8}: ((?:[0-9A-Fa-f]{2} ){1,16})"
    hex_matches = re.findall(hex_pattern, frame_data_section)
    
    if not hex_matches:
        print("错误: 未找到有效的十六进制数据")
        return None
    
    # 合并所有十六进制数据
    hex_data = ''.join(hex_matches).replace(' ', '')
    
    print(f"找到 {len(hex_data)//2} 字节的数据")
    
    # 转换为二进制
    try:
        binary_data = bytes.fromhex(hex_data)
        return binary_data
    except ValueError as e:
        print(f"错误: 无法转换十六进制数据: {e}")
        return None

def verify_jpeg(data):
    """验证JPEG数据的有效性"""
    if len(data) < 4:
        return False, "数据太短"
    
    # 检查JPEG头（SOI标记）
    if data[0] != 0xFF or data[1] != 0xD8:
        return False, f"无效的JPEG头: 0x{data[0]:02X}{data[1]:02X} (应为 0xFFD8)"
    
    # 检查JPEG尾（EOI标记）- 注意：UVC流可能没有完整的EOI
    has_eoi = len(data) >= 2 and data[-2] == 0xFF and data[-1] == 0xD9
    
    # 查找JPEG标记
    markers = []
    i = 0
    while i < len(data) - 1:
        if data[i] == 0xFF:
            marker = data[i+1]
            if marker == 0xD8:
                markers.append("SOI")
            elif marker == 0xD9:
                markers.append("EOI")
            elif marker == 0xE0:
                markers.append("APP0")
            elif marker == 0xE1:
                markers.append("APP1")
            elif marker == 0xDB:
                markers.append("DQT")
            elif marker == 0xC0:
                markers.append("SOF0")
            elif marker == 0xC4:
                markers.append("DHT")
            elif marker == 0xDA:
                markers.append("SOS")
        i += 1
    
    print(f"JPEG标记: {', '.join(markers)}")
    
    if has_eoi:
        return True, "完整的JPEG文件"
    else:
        return True, "JPEG文件（可能缺少EOI标记，但这在UVC流中是正常的）"

def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)
    
    log_file = sys.argv[1]
    frame_number = int(sys.argv[2]) if len(sys.argv) > 2 else 1
    
    # 提取帧数据
    frame_data = extract_frame_data(log_file, frame_number)
    
    if frame_data is None:
        sys.exit(1)
    
    # 验证JPEG
    is_valid, message = verify_jpeg(frame_data)
    print(f"JPEG验证: {message}")
    
    # 保存文件
    output_file = f"frame{frame_number}.jpg"
    with open(output_file, 'wb') as f:
        f.write(frame_data)
    
    print(f"✅ 已保存到 {output_file}")
    print(f"文件大小: {len(frame_data)} 字节")
    
    # 提示用户如何查看
    print(f"\n可以使用以下命令查看图片:")
    print(f"  - Linux/Mac: open {output_file}")
    print(f"  - 或使用任何图片查看器打开")

if __name__ == "__main__":
    main() 