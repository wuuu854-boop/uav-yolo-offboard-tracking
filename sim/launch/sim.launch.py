"""Bridge the Gazebo camera into ROS 2.

The Gazebo topic name depends on the spawned model name and on the sensor name
inside that model. Discover the real one with::

    gz topic -l | grep -i image

then edit the three constants below. They are plain Python constants instead of
launch arguments on purpose: a beginner can fix this in one place, and a wrong
default is obvious instead of being hidden behind a substitution chain.
"""

from launch import LaunchDescription
from launch_ros.actions import Node

WORLD = "target_tracking"
MODEL = "x500_mono_cam_0"  # PX4 appends _0 to the model name
SENSOR = "camera"
LINK = "camera_link"

IMAGE_TOPIC = f"/world/{WORLD}/model/{MODEL}/link/{LINK}/sensor/{SENSOR}/image"
INFO_TOPIC = f"/world/{WORLD}/model/{MODEL}/link/{LINK}/sensor/{SENSOR}/camera_info"


def generate_launch_description() -> LaunchDescription:
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="camera_bridge",
        output="screen",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            f"{IMAGE_TOPIC}@sensor_msgs/msg/Image[gz.msgs.Image",
            f"{INFO_TOPIC}@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
        ],
        remappings=[
            (IMAGE_TOPIC, "/camera/image_raw"),
            (INFO_TOPIC, "/camera/camera_info"),
        ],
    )

    return LaunchDescription([bridge])
