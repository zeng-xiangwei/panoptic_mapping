"""
1、模拟 VLLM 服务的服务端，接收 panoptic-mapper 发出的请求。
2、模拟获取图像服务的客户端，向 panoptic-mapper 发送获取图像的请求。
"""

import rclpy
from rclpy.node import Node
import sys
import time
from sensor_msgs.msg import Image
import numpy as np
import cv2
import os
from cv_bridge import CvBridge
import matplotlib.pyplot as plt

# 服务类型导入 (使用别名以便后续修改)
from panoptic_mapping_msgs.srv import GetSubmapImageData as GetSubmapImageDataSrv
from panoptic_mapping_msgs.srv import VllmProcessing as VllmProcessingSrv
from panoptic_mapping_msgs.msg import BoxInfo as BoxInfoMsg
from panoptic_mapping_msgs.msg import BoxRelationship as BoxRelationshipMsg


class VllmServiceTester(Node):
    """VLLM服务测试类 - 作为服务端接收panoptic-mapper发出的请求"""
    
    def __init__(self):
        super().__init__('vllm_service_tester')
        
        # 创建VLLM服务服务端
        self.vllm_service = self.create_service(
            VllmProcessingSrv, 
            'request_vl_processing', 
            self.vllm_service_callback
        )
        self.get_logger().info("VLLM Service Tester initialized and waiting for requests...")

    def vllm_service_callback(self, request, response):
        """VLLM服务回调函数"""
        self.get_logger().info(f'Received VLLM processing request for image ID: {request.image_id}')
        
        # 设置响应
        response.image_id = request.image_id
        response.success = True
        
        # 创建一些模拟的边界框数据
        if hasattr(response, 'bounding_boxes'):
            # 创建一些示例边界框
            box1 = BoxInfoMsg()
            box1.id = 1
            box1.x = 100
            box1.y = 100
            box1.width = 400
            box1.height = 300
            box1.descriptions = "A red chair"
            
            box2 = BoxInfoMsg()
            box2.id = 2
            box2.x = 300
            box2.y = 200
            box2.width = 150
            box2.height = 180
            box2.descriptions = "A wooden table"
            
            response.bounding_boxes = [box1, box2]
            
        if hasattr(response, 'relationships'):
            # 创建一些示例关系
            rel = BoxRelationshipMsg()
            rel.from_id = 1
            rel.to_id = 2
            rel.type = "on"
            response.relationships = [rel]
        
        self.get_logger().info(f'start sleep, to simulate processing')
        time.sleep(3)
        self.get_logger().info(f'VLLM processing completed for image ID: {request.image_id}')
        self.get_logger().info(f'Generated {len(response.bounding_boxes)} bounding boxes and {len(response.relationships)} relationships')
        return response


class ImageServiceTester(Node):
    """图像服务测试类 - 作为客户端向panoptic-mapper发送获取图像的请求"""
    
    def __init__(self):
        super().__init__('image_service_tester')
        
        # 创建获取图像服务客户端
        self.get_image_client = self.create_client(
            GetSubmapImageDataSrv, 
            'get_submap_image_data'
        )
        
        # 创建cv_bridge实例用于图像转换
        self.bridge = CvBridge()
        
        while not self.get_image_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('GetSubmapImageData service not available, waiting...')

    def test_get_image_service(self, submap_id=0):
        """测试获取图像服务"""
        # 创建请求
        request = GetSubmapImageDataSrv.Request()
        request.submap_id = submap_id
        
        # 发送请求
        future = self.get_image_client.call_async(request)
        
        # 等待响应
        rclpy.spin_until_future_complete(self, future)
        
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'Successfully received image for submap ID: {submap_id}')
                self.get_logger().info(f'Image timestamp: {response.img_timestamp}')
                self.get_logger().info(f'Image size: {response.rgb_image.width}x{response.rgb_image.height}')
                
                # 将图像数据保存到文件
                try:
                    # 使用cv_bridge将ROS图像消息转换为OpenCV格式
                    cv_image = self.bridge.imgmsg_to_cv2(response.rgb_image, "bgr8")
                    
                    # 生成文件名
                    filename = f"submap_{submap_id}_image.png"
                    file_path = os.path.join("/home/xiangweizeng/3D_slam/sematic-mapping/panoptic_mapping_ws/logs", filename)
                    
                    # 保存图像到文件
                    cv2.imwrite(file_path, cv_image)
                    self.get_logger().info(f'Image saved to file: {filename}')
                    
                    # 显示图像
                    self.show_image(response.rgb_image)
                    return True
                except Exception as e:
                    self.get_logger().error(f'Failed to save image to file: {e}')
                    return False
            else:
                self.get_logger().warn(f'Failed to get image for submap ID: {submap_id}')
                return False
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')
            return False
        
    def show_image(self, image_msg):
        """使用matplotlib显示图像"""
        try:
            # 使用cv_bridge将ROS图像消息转换为OpenCV格式
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
            
            # 转换BGR到RGB（因为OpenCV使用BGR，而matplotlib使用RGB）
            rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            
            # 使用matplotlib显示图像
            plt.figure(figsize=(10, 8))
            plt.imshow(rgb_image)
            plt.title("Received Image from Panoptic Mapper")
            plt.axis('off')  # 关闭坐标轴
            plt.tight_layout()
            plt.show()
            
            self.get_logger().info("Image displayed using matplotlib")
        except Exception as e:
            self.get_logger().error(f"Failed to display image: {e}")


def test_vllm_service():
    """测试VLLM服务"""
    print("Testing VLLM Service (Server mode)")
    print("Waiting for VLLM processing requests...")
    print("Press Ctrl+C to stop.")
    
    rclpy.init()
    tester = VllmServiceTester()
    
    try:
        rclpy.spin(tester)
    except KeyboardInterrupt:
        print("\nShutting down VLLM service tester...")
    finally:
        tester.destroy_node()
        rclpy.shutdown()


def test_image_service():
    """测试图像获取服务"""
    print("Testing Image Retrieval Service (Client mode)")
    
    rclpy.init()
    tester = ImageServiceTester()
    
    # 测试获取特定submap的图像
    result = tester.test_get_image_service(submap_id=1)
    
    if result:
        print("Image service test passed!")
    else:
        print("Image service test failed!")
        
    tester.destroy_node()
    rclpy.shutdown()


def main(args=None):
    # 通过注释/取消注释来选择要运行的测试
    
    # 运行VLLM服务测试 (服务端模式)
    # test_vllm_service()
    
    # 运行图像服务测试 (客户端模式)
    test_image_service()


if __name__ == '__main__':
    main()