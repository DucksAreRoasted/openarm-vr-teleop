import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launch_utils import DeclareBooleanLaunchArg


def rviz_environment():
    renderer = os.environ.get("OPENARM_MOVEIT_RVIZ_RENDERER", "auto").strip().lower()
    if renderer == "auto":
        return {}
    if renderer != "software":
        raise ValueError("OPENARM_MOVEIT_RVIZ_RENDERER must be software or auto")
    # Scope the GLX workaround to RViz; planning and hardware nodes keep their
    # original environments. Software rendering is an opt-in diagnostic fallback.
    return {
        "QT_QPA_PLATFORM": "xcb",
        "__GLX_VENDOR_LIBRARY_NAME": "mesa",
        "LIBGL_ALWAYS_SOFTWARE": "1",
        "GALLIUM_DRIVER": "llvmpipe",
        "MESA_LOADER_DRIVER_OVERRIDE": "swrast",
        "__NV_PRIME_RENDER_OFFLOAD": "0",
        "DRI_PRIME": "0",
        "LIBGL_ALWAYS_INDIRECT": "0",
    }


def generate_launch_description():
    render_env = rviz_environment()
    moveit_config = MoveItConfigsBuilder("openarm", package_name="openarm_moveit_config").to_moveit_configs()
    launch = LaunchDescription()
    launch.add_action(LogInfo(msg="[moveit] RViz renderer=" + ("Mesa software" if render_env else "system default")))
    launch.add_action(DeclareBooleanLaunchArg("debug", default_value=False))
    launch.add_action(
        DeclareLaunchArgument(
            "rviz_config",
            default_value=str(moveit_config.package_path / "config/moveit.rviz"),
        )
    )
    launch.add_action(
        Node(
            package="rviz2",
            executable="rviz2",
            additional_env=render_env,
            output="log",
            arguments=["-d", LaunchConfiguration("rviz_config")],
            parameters=[
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
                moveit_config.planning_pipelines,
                moveit_config.joint_limits,
            ],
        )
    )
    return launch
