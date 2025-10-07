import time
import rclpy
from rclpy.logging import get_logger
from moveit.core.robot_state import RobotState
from moveit.planning import (
    MoveItPy,
    MultiPipelinePlanRequestParameters,
)
from moveit_configs_utils import MoveItConfigsBuilder
from geometry_msgs.msg import PoseStamped

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

    path = __file__.split('motion_api/test3')[0] + 'config/moveit_cpp.yaml'
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
    
    ########################### 多种路径规划 ############################
    # 设置机械臂起始位置为当前状态位置
    arm_group.set_start_state_to_current_state()

    arm_group.set_goal_state(configuration_name="ready")

    multi_pipeline_plan_request_params = MultiPipelinePlanRequestParameters(
        robot, ["ompl_rrtc", "pilz_lin", "chomp_planner"]
    )

    # 进行运动规划
    plan_and_execute(robot, arm_group, logger, multi_plan_parameters=multi_pipeline_plan_request_params, sleep_time=3.0)