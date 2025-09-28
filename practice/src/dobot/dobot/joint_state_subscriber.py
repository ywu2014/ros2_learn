import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import mujoco
import numpy as np
import glfw
import cv2

class JointStateSubscriber(Node):
    def __init__(self):
        super().__init__("joint_state_subscriber")
        self.subscription = self.create_subscription(
            JointState,
            "/joint_states",
            self.callback,
            10
        )
        
        # 加载模型
        self.model = mujoco.MjModel.from_xml_path('/home/linkedata/projects/ros2/ros2_test/ros2_learn/practice/src/dobot/mjcf/scene_check.xml')
        self.data = mujoco.MjData(self.model)

        # 打印所有 body 的 ID 和名称
        print("All bodies in the model:")
        for i in range(self.model.nbody):
            body_name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, i)
            print(f"ID: {i}, Name: {body_name}")

        # 初始化 GLFW
        if not glfw.init():
            return

        self.window = glfw.create_window(1200, 900, 'Dobot Arm Control', None, None)
        if not self.window:
            glfw.terminate()
            return

        glfw.make_context_current(self.window)

        # 设置鼠标滚轮回调函数
        # glfw.set_scroll_callback(self.window, self.scroll_callback)

        # 初始化渲染器
        # self.cam = mujoco.MjvCamera()
        # self.opt = mujoco.MjvOption()
        # mujoco.mjv_defaultCamera(self.cam)
        # mujoco.mjv_defaultOption(self.opt)
        self.camera = mujoco.MjvCamera()
        camID = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_CAMERA, "depth_camera")
        self.camera.fixedcamid = camID
        self.camera.type = mujoco.mjtCamera.mjCAMERA_FIXED

        self.pert = mujoco.MjvPerturb()
        self.con = mujoco.MjrContext(self.model, mujoco.mjtFontScale.mjFONTSCALE_150)

        self.scene = mujoco.MjvScene(self.model, maxgeom=10000)

        # 找到末端执行器的 body id
        self.end_effector_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, 'Link6')
        print(f"End effector ID: {self.end_effector_id}")
        if self.end_effector_id == -1:
            print("Warning: Could not find the end effector with the given name.")
            glfw.terminate()
            return

        # 初始关节角度
        self.initial_q = self.data.qpos[:6].copy()
        print(f"Initial joint positions: {self.initial_q}")

        self.timer1 = self.create_timer(
            0.01,  # 100ms周期
            self.timer_callback1
        )

        self.sample_count = 0

    def scroll_callback(self, window, xoffset, yoffset):
        # 调整相机的缩放比例
        self.cam.distance *= 1 - 0.1 * yoffset

    def limit_angle(self, angle):
        while angle > np.pi:
            angle -= 2 * np.pi
        while angle < -np.pi:
            angle += 2 * np.pi
        return angle
    
    def get_image(self, w, h):
        # 定义视口大小
        viewport = mujoco.MjrRect(0, 0, w, h)
        # 更新场景
        mujoco.mjv_updateScene(
            self.model, self.data, mujoco.MjvOption(), 
            None, self.camera, mujoco.mjtCatBit.mjCAT_ALL, self.scene
        )
        # 渲染到缓冲区
        mujoco.mjr_render(viewport, self.scene, self.con)
        # 读取 RGB 数据（格式为 HWC, uint8）
        rgb = np.zeros((h, w, 3), dtype=np.uint8)
        depth = np.zeros((h, w), dtype=np.float64)
        mujoco.mjr_readPixels(rgb, depth, viewport, self.con)
        cv_image = cv2.cvtColor(np.flipud(rgb), cv2.COLOR_RGB2BGR)

        # 参数设置
        min_depth_m = 0.0  # 最小深度（0米）
        max_depth_m = 8.0  # 最大深度（8米）
        near_clip = 0.1    # 近裁剪面（米）
        far_clip = 50.0    # 远裁剪面（米）
        # 将非线性深度缓冲区值转换为线性深度（米）
        # 公式: linear_depth = far * near / (far - (far - near) * depth)
        linear_depth_m = far_clip * near_clip / (far_clip - (far_clip - near_clip) * depth)
        # 裁剪深度到0-8米范围
        depth_clipped = np.clip(linear_depth_m, min_depth_m, max_depth_m)
        # 映射0-8米到0-255像素值（距离越小越亮）
        # 反转映射：距离越小值越大（越亮）
        inverted_depth = max_depth_m - depth_clipped
        # 计算缩放因子：255/(max_depth_m - min_depth_m)
        scale = 255.0 / (max_depth_m - min_depth_m)
        depth_visual = (inverted_depth * scale).astype(np.uint8)
        # 翻转图像（MuJoCO坐标系到OpenCV坐标系）
        depth_visual = np.flipud(depth_visual)
        return cv_image, depth_visual

    def timer_callback1(self):
        if not glfw.window_should_close(self.window):
    
            # 获取当前末端执行器位置
            mujoco.mj_forward(self.model, self.data)
            self.end_effector_pos = self.data.body(self.end_effector_id).xpos

            # 设置关节目标位置
            self.data.qpos[:6] = self.positions

            # 模拟一步
            mujoco.mj_step(self.model, self.data)

            img, depth_img = self.get_image(640,480)
            cv2.imshow("img", img)
            cv2.imshow("depth_img", depth_img)
            key = cv2.waitKey(1)
            if key == ord('c'):
                cv2.imwrite(f"./img/board_pos_{self.sample_count}.png", img)
                print(f"end effector pos {self.sample_count}: {self.end_effector_pos}")
                self.sample_count += 1

            # 更新渲染场景
            # viewport = mujoco.MjrRect(0, 0, 1200, 900)
            # mujoco.mjv_updateScene(self.model, self.data, self.opt, self.pert, self.cam, mujoco.mjtCatBit.mjCAT_ALL.value, self.scene)
            # mujoco.mjr_render(viewport, self.scene, self.con)

            # 交换前后缓冲区
            glfw.swap_buffers(self.window)
            glfw.poll_events()

    def callback(self, msg: JointState):
        # 过滤Panda机械臂的6个关节（名称以panda_joint开头）
        # panda_joints = [name for name in msg.name if "panda_joint" in name]
        # self.positions = [msg.position[i] for i, name in enumerate(msg.name) if "panda_joint" in name]

        panda_joints = [name for name in msg.name]
        self.positions = [msg.position[i] for i, name in enumerate(msg.name)]

        # for name, pos in zip(panda_joints, self.positions):
        #     self.get_logger().info(f"{name}: {pos:.4f}")

def main(args=None):
    rclpy.init(args=args)
    node = JointStateSubscriber()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
    # 清理资源
    glfw.terminate()       