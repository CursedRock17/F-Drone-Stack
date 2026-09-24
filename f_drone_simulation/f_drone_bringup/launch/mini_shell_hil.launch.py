"""Run mini-shell sensors and motors with the existing Teensy serial controller."""

from pathlib import Path
from tempfile import TemporaryDirectory
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch.substitutions import LaunchConfiguration


def start_hil(context):
    description = Path(get_package_share_directory('f_drone_description'))
    gazebo = Path(get_package_share_directory('f_drone_gazebo'))
    world = ET.parse(gazebo / 'worlds/mini_shell_hil.sdf')
    model = ET.parse(description / 'models/mini_shell/model_hil.sdf').getroot().find('model')

    # Embed a configured copy so the installed model and serial default stay intact.
    scene = world.getroot().find('world')
    include = scene.find('include')
    model.set('name', include.findtext('name'))
    model.insert(0, include.find('pose'))
    scene.remove(include)
    scene.append(model)
    serial = model.find("plugin[@name='f_drone_gazebo::DroneSerial']")
    serial.find('port').text = LaunchConfiguration('serial_port').perform(context)

    # Keep the generated world alive until launch shuts down.
    temporary = TemporaryDirectory(prefix='mini-shell-hil-')
    world_path = Path(temporary.name) / 'world.sdf'
    world.write(world_path, encoding='unicode', xml_declaration=True)
    command = ['gz', 'sim', '-r', str(world_path)]
    if LaunchConfiguration('gazebo_gui').perform(context).lower() == 'false':
        command.extend(['-s', '--headless-rendering'])

    return [
        RegisterEventHandler(OnShutdown(on_shutdown=[
            OpaqueFunction(function=lambda context: temporary.cleanup())])),
        ExecuteProcess(cmd=command, output='screen'),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('gazebo_gui', default_value='true', choices=['true', 'false']),
        OpaqueFunction(function=start_hil),
    ])
