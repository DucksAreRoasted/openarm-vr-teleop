from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch


def launch_setup(context, *args, **kwargs):
    compliant = LaunchConfiguration("compliant_timeout").perform(context).lower() in (
        "1",
        "true",
        "yes",
    )
    controller_file = (
        "config/moveit_controllers_compliant.yaml"
        if compliant
        else "config/moveit_controllers.yaml"
    )
    moveit_config = (
        MoveItConfigsBuilder("openarm", package_name="openarm_moveit_config")
        .trajectory_execution(file_path=controller_file)
        .to_moveit_configs()
    )
    return generate_move_group_launch(moveit_config).entities


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "compliant_timeout",
                default_value="true",
                description="MoveIt 把轨迹超时留到机械臂侧判定",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
