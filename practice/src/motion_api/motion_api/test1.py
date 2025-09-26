import time
import rclpy
from rclpy.logging import get_logger
from moveit.core.robot_state import RobotState
from moveit.planning import (
    MoveItPy,
    MultiPipelinePlanRequestParameters,
)
from moveit_configs_utils import MoveItConfigsBuilder

def plan_and_execute(
        robot,
        planning_component,
        logger,
        single_plan_parameters=None,
        multi_plan_parameters=None,
        sleep_time=0.0,
):
    logger.info("Planning trajectory")
    if multi_plan_parameters:
        plan_result = planning_component.plan(
            multi_plan_parameters=multi_plan_parameters
        )
    elif single_plan_parameters:
        plan_result = planning_component.plan(
            single_plan_parameters=single_plan_parameters
        )
    else:
        plan_result = planning_component.plan()
    
    if plan_result:
        logger.info("Executing trajectory")
        robot_trajectory = plan_result.trajectory
        robot.execute(robot_trajectory, controllers=[])
    else:
        logger.info("Planning failed")
        
    time.sleep(sleep_time)

if __name__ == "__main__":
    rclpy.init()
    logger = get_logger("moveit_py.pose_goal")

    path = __file__.split('motion_api/test1')[0] + 'config/moveit_cpp.yaml'
    print(f'moveit_cpp.yaml path: {path}')

    moveit_config = (
        MoveItConfigsBuilder(
            robot_name="arm", package_name="motion_arm_config"
        ).moveit_cpp(path)
        .to_moveit_configs()
    )

    params=moveit_config.to_dict()

    robot = MoveItPy(
        node_name="moveit_py",
        config_dict=params,
    )
    arm_group = robot.get_planning_component("arm") # 获取规划组
    hand_group = robot.get_planning_component("hand") # 获取规划组
    logger.info("MoveItPy initialized")

    print(f'robot: {robot}')
    ########################### 通过预定义位置规划 ############################

    # 设置起始为预定义的stand位置
    arm_group.set_start_state(configuration_name="stand")

    # 设置目标位姿为预定义的ready位置
    arm_group.set_goal_state(configuration_name="ready")

    # 进行路径规划并执行
    plan_and_execute(
        robot=robot,
        planning_component=arm_group,
        logger=logger,
        sleep_time=3.0,
    )