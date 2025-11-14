# 包含图片、描述、物体名称及模式的消息
int32 image_id                      # 图片id
sensor_msgs/Image image             # 图片数据（ROS标准图片类型）bgr8
string description                  # 图片的文本描述
# string object_name                # 图片中包含的物体名称（单个物体）
string[] object_names               # 若支持多个物体，可改用字符串数组
uint8 mode                          # 模式：0=生成，1=查询，具体见 GetObjectInfoMode.msg