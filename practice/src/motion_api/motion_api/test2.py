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

    path = __file__.split('motion_api/test2')[0] + 'config/moveit_cpp.yaml'
    print(f'moveit_cpp.yaml path: {path}')

    moveit_config = (
        MoveItConfigsBuilder(
            robot_name="arm", package_name="robot_arm_config"
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
    
    ########################### 通过关节角设置位置 ############################
    robot_model = robot.get_robot_model()
    robot_state = RobotState(robot_model)
    
    # 指定关节位置, 并将其设置为目标状态
    robot_state.set_joint_group_positions("arm", [0.0, -0.4, -0.79, -0.79, -1.10, 1.55])
    arm_group.set_goal_state(robot_state=robot_state)

    # 设置机械臂起始位置为当前状态位置
    arm_group.set_start_state_to_current_state()

    # 进行规划和执行
    plan_and_execute(robot, arm_group, logger, sleep_time=3.0)