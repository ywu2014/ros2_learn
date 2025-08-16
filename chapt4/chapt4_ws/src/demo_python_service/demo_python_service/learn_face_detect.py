import face_recognition
import cv2
from ament_index_python.packages import get_package_share_directory

def main():
    # 获取图片真实路径
    # 通过功能包名字获取该功能包的安装目录
    default_img_path = get_package_share_directory('demo_python_service') + '/resource/default.jpg'
    print('default_img_path:', default_img_path)
    # 使用opencv加载图片
    image = cv2.imread(default_img_path)
    # 查找图像中的所有人脸
    face_locations = face_recognition.face_locations(image, number_of_times_to_upsample=1, model='hog')

    # 绘制每个人脸的边框
    for top, right, bottom, left in face_locations:
        # 绘制边框
        cv2.rectangle(image, (left, top), (right, bottom), (255, 0, 0), 4)
    # 显示图像
    cv2.imshow('Face Detect', image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()