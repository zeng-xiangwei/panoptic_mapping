int32 id            # 唯一id，用于标识不同 box
string object_name  # 物体名称（如"chair"）
int32[] bbox        # 边界框，格式为[xmin, ymin, xmax, ymax]（4个整数），图像坐标系下（图片左上角为原点，宽方向为x，高方向为y）
string color        # 颜色（如"black"）
string shape        # 形状（如"seat"、"rectangular"）