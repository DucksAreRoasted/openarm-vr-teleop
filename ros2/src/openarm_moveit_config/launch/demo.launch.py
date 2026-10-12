from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
    Shutdown,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launch_utils import DeclareBooleanLaunchArg


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("openarm", package_name="openarm_moveit_config").to_moveit_configs()
    package = moveit_config.package_path
    use_real = LaunchConfiguration("use_real")
    launch = LaunchDescription()
    launch.add_action(DeclareBooleanLaunchArg("use_rviz", default_value=True))
    launch.add_action(DeclareBooleanLaunchArg("use_real", default_value=False))
    launch.add_action(DeclareLaunchArgument("left_can", default_value="can1"))
    launch.add_action(DeclareLaunchArgument("right_can", default_value="can0"))
    launch.add_action(
        DeclareLaunchArgument(
            "error_log_dir",
            default_value="",
        )
    )
    launch.add_action(
        DeclareLaunchArgument(
            "gravity_yaml",
            default_value="",
        )
    )
    launch.add_action(DeclareLaunchArgument("publish_frequency", default_value="15.0"))
    launch.add_action(
        LogInfo(
            msg=[
                "[moveit] use_real=",
                use_real,
                " left_can=",
                LaunchConfiguration("left_can"),
                " right_can=",
                LaunchConfiguration("right_can"),
            ]
        )
    )
    for name in ("rsp.launch.py", "move_group.launch.py"):
        launch.add_action(
            IncludeLaunchDescription(PythonLaunchDescriptionSource(str(package / "launch" / name)))
        )
    launch.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(package / "launch" / "moveit_rviz.launch.py")),
            condition=IfCondition(LaunchConfiguration("use_rviz")),
        )
    )
    real_arm = Node(
        package="openarm_moveit_config",
        executable="real_arm",
        output="screen",
        parameters=[
            {
                "left_can": ParameterValue(LaunchConfiguration("left_can"), value_type=str),
                "right_can": ParameterValue(LaunchConfiguration("right_can"), value_type=str),
                "error_log_dir": ParameterValue(
                    LaunchConfiguration("error_log_dir"), value_type=str
                ),
                "gravity_yaml": ParameterValue(
                    LaunchConfiguration("gravity_yaml"), value_type=str
                ),
            },
        ],
        condition=IfCondition(use_real),
    )
    launch.add_action(real_arm)
    launch.add_action(
        RegisterEventHandler(
            OnProcessExit(
                target_action=real_arm,
                on_exit=[
                    LogInfo(msg="[moveit] 真机节点退出，整个 MoveIt 停止。请检查 CAN 是否已配置。"),
                    Shutdown(reason="real_arm exited"),
                ],
            )
        )
    )
    launch.add_action(
        Node(
            package="openarm_moveit_config",
            executable="fake_arm.py",
            output="screen",
            condition=UnlessCondition(use_real),
        )
    )
    return launch
