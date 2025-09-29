"""
urdf转mjcf
或者使用命令行:
```
compile urdf/cr3_robot_1.urdf mjcf/cr3_robot.xml
```
"""

import mujoco
import os
import numpy as np

def urdf_to_mujoco(urdf_path, output_path, resource_path=None):
    """
    将URDF文件转换为MuJoCo XML格式
    
    参数:
        urdf_path: URDF文件路径
        output_path: 输出的MuJoCo XML文件路径
        resource_path: 资源文件(如meshes)的路径
    """
    # 设置资源路径
    if resource_path:
        os.environ['MUJOCO_PY_MUJOCO_PATH'] = resource_path
    
    try:
        # 加载URDF模型
        model = mujoco.MjModel.from_xml_path(urdf_path)
        
        # 修改一些模型参数以适应MuJoCo
        # 例如，调整阻尼和刚度
        model.dof_damping *= 0.1
        model.dof_frictionloss *= 0.1
        
        # 保存为MuJoCo XML格式
        mujoco.mj_saveLastXML(output_path, model)
        
        print(f"成功将 {urdf_path} 转换为 {output_path}")
        return True
    except Exception as e:
        print(f"转换失败: {str(e)}")
        return False

if __name__ == "__main__":
    mjcf_dir = "/home/linkedata/projects/ros2/ros2_test/ros2_learn/practice/src/dobot/mjcf"
    urdf_dir = "/home/linkedata/projects/ros2/ros2_test/ros2_learn/practice/src/dobot/urdf"

    urdf_file_name = "cr3_robot_1.urdf"
    mjcf_file_name = urdf_file_name.replace(".urdf", ".xml")
    
    mesh_dir = "/home/linkedata/projects/ros2/ros2_test/ros2_learn/practice/src/dobot/mesh"
    
    urdf_to_mujoco(
        urdf_path=f'{urdf_dir}/{urdf_file_name}',
        output_path=f'{mjcf_dir}/{mjcf_file_name}',
        resource_path=mesh_dir
    )
