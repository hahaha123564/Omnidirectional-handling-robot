import cv2  # OpenCV库，用于图像和视频处理
import numpy as np  # NumPy库，用于数组和矩阵计算
import torch  # PyTorch库，用于深度学习模型
import math
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from models.experimental import attempt_load  # 导入YOLOv5模型
from utils.general import check_img_size, non_max_suppression, scale_boxes  # 导入辅助函数
from utils.plot3 import Annotator, colors  # 导入辅助函数
from utils.augmentations import letterbox  # 导入辅助函数
from utils.torch_utils import select_device  # 导入辅助函数
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
from geometry_msgs.msg import Point
weights = '8.4_take_best.pt'  # 模型权重路径
device = 'cpu'  # 设备号，0表示使用第一个GPU；设置为'cpu'，表示使用CPU
img_size = 640  # 图像大小
stride = 32  # 步长
half = False  # 是否使用半精度浮点数减少内存占用，需要GPU支持

device = select_device(device)  # 设置设备
half &= device.type != 'cpu'  # 如果设备为CPU，则禁用半精度模式

# 导入YOLOv5模型
model = attempt_load(weights, device=device)
img_size = check_img_size(img_size, s=stride)  # 检查图像大小是否合法
names = model.names  # 类别名称列表
def increase_brightness(img, value=30):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(hsv)

    v = cv2.add(v, value)
    v = np.clip(v, 0, 255)
    final_hsv = cv2.merge((h, s, v))
    img_brightness = cv2.cvtColor(final_hsv, cv2.COLOR_HSV2BGR)

    return img_brightness
class image_converter:
    def __init__(self):  
        self.image_pub = rospy.Publisher("cv_bridge_image", Image, queue_size=1)
        self.object_pub = rospy.Publisher("objects1", Detection2DArray, queue_size=1)
        self.cube_list = Detection2DArray()
        self.bridge = CvBridge()
        self.cam_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.image_CB)

    def image_CB(self, data):
        cap = self.bridge.imgmsg_to_cv2(data, "bgr8")  
        img0 = cap.copy()
        #cv2.imshow("777",cap)
        # cv2.waitKey(1)
        # 对图像进行自适应图像缩放
        img0 = increase_brightness(img0, value=80)
        img = letterbox(img0, img_size, stride=stride, auto=True)[0]  # HWC转为CHW，BGR转为RGB
        # 转换图像格式
        img = img.transpose((2, 0, 1))[::-1]
        img = np.ascontiguousarray(img)
        img = torch.from_numpy(img).to(device)
        img = img.float() / 255.0  # 像素值归一化到[0.0, 1.0]
        img = img[None]  # [h w c] -> [1 h w c]

        # 模型推理
        pred = model(img)[0]
        pred = non_max_suppression(pred, conf_thres=0.25, iou_thres=0.45, max_det=1000)

        # 绘制边框和标签
        det = pred[0]  # 检测结果
        annotator = Annotator(img0.copy(), line_width=3, example=str(names))
        cub_list = []

        if len(det):   # 画所有的检测框（如果帧里有方块）
            det[:, :4] = scale_boxes(img.shape[2:], det[:, :4], img0.shape).round()   # 将边框坐标缩放到原始图像大小
            for *xyxy, conf, cls in reversed(det):  # 遍历每一帧上的方块
                c = int(cls)  # 类别索引
                label = f'{names[c]} {conf:.2f}'   # 类别标签和置信度
                x_mid, y_mid = annotator.box_label(xyxy, label, color=colors(c, True))
                cub_list.append([(c+1), [x_mid, y_mid, 0]])

        # 写入视频帧
        im0 = annotator.result()
        cv2.imshow("777",im0)
        cv2.waitKey(1)
        self.cube_list.detections = []
        print(cub_list)
        for item in cub_list:
            detection = Detection2D()
            detection.header = data.header
            object_hypothesis = ObjectHypothesisWithPose()
            object_hypothesis.id = item[0]
            object_hypothesis.pose.pose.position = Point(x=item[1][0], y=item[1][1], z=item[1][2])
            detection.results.append(object_hypothesis)
            self.cube_list.detections.append(detection)
        if self.cube_list == [] :
            pass
        else:           		
            self.object_pub.publish(self.cube_list)
if __name__=='__main__':
    rospy.init_node('cv_bridge_test1')
    try:
        #print(6666)    
        image_converter()    	
        rospy.spin()
    except KeyboardInterrupt:
        print("ShutDown")
    cv2.destroyAllWindows()

