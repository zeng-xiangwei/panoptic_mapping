import pandas as pd
import numpy as np
import math
import argparse

def rainbow_color_map(h):
    """
    实现与C++ voxblox库中完全相同的rainbowColorMap功能
    将[0,1]范围内的值转换为RGB颜色
    """
    # 确保h在[0,1]范围内
    h = max(0.0, min(1.0, float(h)))
    
    # 复制C++代码逻辑
    s = 1.0  # 饱和度
    v = 1.0  # 明度
    
    # h -= floor(h) 等价于 h % 1.0
    h = h % 1.0
    h *= 6
    i = int(math.floor(h))
    f = h - i
    
    # 如果i是偶数，则f = 1 - f
    if i & 1 == 0:  # 检查i是否为偶数
        f = 1 - f
    
    m = v * (1 - s)
    n = v * (1 - s * f)
    
    # 根据i的值设置RGB
    if i == 6 or i == 0:
        r, g, b = v, n, m
    elif i == 1:
        r, g, b = n, v, m
    elif i == 2:
        r, g, b = m, v, n
    elif i == 3:
        r, g, b = m, n, v
    elif i == 4:
        r, g, b = n, m, v
    elif i == 5:
        r, g, b = v, m, n
    else:
        # 默认情况 - 不应达到这里
        r, g, b = 1.0, 0.5, 0.5  # 对应红色(255,127,127)
    
    # 转换为0-255范围的整数
    return (int(r * 255), int(g * 255), int(b * 255))

class ExponentialOffsetIdColorMap:
    """
    Python实现的ExponentialOffsetIdColorMap，与C++版本逻辑完全一致
    """
    def __init__(self, items_per_revolution=10):
        self.items_per_revolution = items_per_revolution

    def set_items_per_revolution(self, value):
        self.items_per_revolution = value

    def color_lookup(self, value):
        """
        根据ID值返回对应的颜色，使用与C++完全相同的算法
        """
        # C++代码：const size_t revolution = value / items_per_revolution_;
        revolution = value // self.items_per_revolution
        
        # C++代码：const float progress_along_revolution = std::fmod(value / items_per_revolution_, 1.f);
        progress_along_revolution = (value / self.items_per_revolution) % 1.0
        
        # C++代码：Calculate the offset if appropriate
        offset = 0
        if self.items_per_revolution < value + 1:
            # 处理revolution为0的情况，log2(0)未定义
            if revolution == 0:
                current_episode = 0
            else:
                current_episode = int(math.floor(math.log2(revolution)))
            episode_start = int(2 ** current_episode)  # 等同于std::exp2
            episode_num_subdivisions = episode_start
            current_subdivision = revolution - episode_start
            subdivision_step_size = 1 / (self.items_per_revolution * 2 * episode_num_subdivisions)
            offset = (2 * current_subdivision + 1) * subdivision_step_size

        # C++代码：const float normalized_color = progress_along_revolution + offset;
        normalized_color = progress_along_revolution + offset
        
        # 使用与C++完全相同的rainbowColorMap
        return rainbow_color_map(normalized_color)

def process_csv(input_file, output_file, items_per_revolution=10):
    """
    处理CSV文件，根据ID字段重新设置RGB值
    """
    print(f"Reading CSV file: {input_file}")
    
    # 读取CSV文件
    df = pd.read_csv(input_file)
    
    # 检查必要的列是否存在
    required_columns = ['x', 'y', 'z', 'r', 'g', 'b', 'id', 'label', 'changeStatus']
    for col in required_columns:
        if col not in df.columns:
            raise ValueError(f"Required column '{col}' not found in CSV file")
    
    # 创建颜色映射实例
    color_map = ExponentialOffsetIdColorMap(items_per_revolution)
    
    # 创建新列存储新的RGB值
    new_r_values = []
    new_g_values = []
    new_b_values = []
    
    # 遍历每一行，根据ID生成新颜色
    for idx, row in df.iterrows():
        # 获取ID值
        id_val = int(row['id'])
        
        # 使用颜色映射获取新颜色
        r, g, b = color_map.color_lookup(id_val)
        
        new_r_values.append(r)
        new_g_values.append(g)
        new_b_values.append(b)
        
        if idx % 1000 == 0:  # 每1000行打印一次进度
            print(f"Processed {idx} rows...")
    
    # 更新DataFrame中的RGB值
    df['r'] = new_r_values
    df['g'] = new_g_values
    df['b'] = new_b_values
    
    # 保存处理后的CSV文件
    df.to_csv(output_file, index=False)
    print(f"Processed CSV saved to: {output_file}")
    print(f"Total rows processed: {len(df)}")

def main():
    parser = argparse.ArgumentParser(description='Process CSV file and update RGB values based on ID using ExponentialOffsetIdColorMap')
    parser.add_argument('input_file', help='Input CSV file path')
    parser.add_argument('output_file', help='Output CSV file path')
    parser.add_argument('--items-per-revolution', type=int, default=20, 
                       help='Number of items per revolution for color mapping (default: 20)')
    
    args = parser.parse_args()
    
    try:
        process_csv(args.input_file, args.output_file, args.items_per_revolution)
    except Exception as e:
        print(f"Error processing CSV: {e}")
        return 1
    
    return 0

if __name__ == "__main__":
    main()