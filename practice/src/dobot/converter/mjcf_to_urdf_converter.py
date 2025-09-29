"""
mjcf转urdf
https://github.com/Yasu31/mjcf_urdf_simple_converter
"""

from mjcf_urdf_simple_converter import convert

mjcf_dir = "/home/linkedata/projects/ros2/ros2_test/ros2_learn/practice/src/dobot/mjcf"
urdf_dir = "/home/linkedata/projects/ros2/ros2_test/ros2_learn/practice/src/dobot/urdf"

mjcf_file_name = "scene_check.xml"
urdf_file_name = mjcf_file_name.replace(".xml", ".urdf")

convert(f"{mjcf_dir}/{mjcf_file_name}", f"{urdf_dir}/{urdf_file_name}")
# or, if you are using it in your ROS package and would like for the mesh directories to be resolved correctly, set meshfile_prefix, for example:
# convert("model.xml", "model.urdf", asset_file_prefix="package://your_package_name/model/")