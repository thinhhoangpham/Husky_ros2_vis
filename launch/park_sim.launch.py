# Copyright 2023 Clearpath Robotics, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# @author Roni Kreinin (rkreinin@clearpathrobotics.com)

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.actions import TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution


ARGUMENTS = [
    DeclareLaunchArgument('rviz', default_value='false',
                          choices=['true', 'false'], description='Start rviz.'),
    DeclareLaunchArgument('world', default_value='warehouse',
                          choices=[
                              'construction',
                              'office',
                              'orchard',
                              'pipeline',
                              'solar_farm',
                              'warehouse',
                              'park',
                              'lake',
                              'warehouse_ext',
                              'warehouse_ramp',
                          ],
                          description='Gazebo World'),
    DeclareLaunchArgument('setup_path',
                          default_value=[EnvironmentVariable('HOME'), '/clearpath/'],
                          description='Clearpath setup path'),
    DeclareLaunchArgument('use_sim_time', default_value='true',
                          choices=['true', 'false'],
                          description='use_sim_time'),
    # Sequences the robot spawn after the GUI has had time to finish loading
    # its scene, rather than racing it. This is NOT a readiness poll/sleep -
    # nothing here checks whether loading is done - it is one-shot launch
    # sequencing: park is heavy enough (~97 models, ~221 MB of textures
    # including a 46 MB normal map used 16x) that the GUI can still be
    # building its scene when the robot spawn message arrives, and
    # gz-sim's GUI silently misses model-creation events that land mid-load
    # (see CLAUDE.md and .claude/agents/sim-operator.md). Default 0.0 is a
    # no-op for every world that has not shown this failure; sim.py sets it
    # to a positive value for park only.
    DeclareLaunchArgument('spawn_delay', default_value='0.0',
                          description='Seconds to wait before spawning the '
                                       'robot, to let a heavy world\'s GUI '
                                       'finish loading its scene first.'),
]

for pose_element in ['x', 'y', 'yaw']:
    ARGUMENTS.append(DeclareLaunchArgument(pose_element, default_value='0.0',
                     description=f'{pose_element} component of the robot pose.'))

ARGUMENTS.append(DeclareLaunchArgument('z', default_value='0.3',
                 description='z component of the robot pose.'))

# Authored spawn poses for worlds whose ground is not at z=0. Clearpath's
# defaults put the robot at the origin at z=0.3, which is below park's terrain
# (z~=2.99) and lake's (3.5-5.9): the robot materialises under the ground and
# falls out of the world. Values are from the original ROS 1 launch files
# natural_enviroment/launch/add_husky_<world>_1.launch.
WORLD_SPAWN_POSES = {
    'park': {'x': '45.64', 'y': '0.02', 'z': '3.3', 'yaw': '2.6132'},
    'lake': {'x': '-47.0', 'y': '-15.0', 'z': '4.0', 'yaw': '0.0'},
}

# The declared defaults above. A pose element still holding its declared
# default is taken as "not set by the caller" and is replaced by the world's
# authored value; anything else the caller passed wins.
STOCK_POSE_DEFAULTS = {'x': '0.0', 'y': '0.0', 'yaw': '0.0', 'z': '0.3'}


def generate_launch_description():
    # Directories
    pkg_clearpath_gz = get_package_share_directory(
        'clearpath_gz')

    # Paths
    gz_sim_launch = '/home/thinhpham/Documents/Husky_viz/launch/gz_sim.launch.py'
    robot_spawn_launch = PathJoinSubstitution(
        [pkg_clearpath_gz, 'launch', 'robot_spawn.launch.py'])

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gz_sim_launch]),
        launch_arguments=[
            ('world', LaunchConfiguration('world')),
            ('setup_path', LaunchConfiguration('setup_path')),
        ]
    )

    def spawn_with_world_pose(context):
        world = LaunchConfiguration('world').perform(context)
        pose = dict(WORLD_SPAWN_POSES.get(world, {}))
        resolved = {}
        for element in ('x', 'y', 'z', 'yaw'):
            given = LaunchConfiguration(element).perform(context)
            if given == STOCK_POSE_DEFAULTS[element] and element in pose:
                resolved[element] = pose[element]
            else:
                resolved[element] = given
        return [IncludeLaunchDescription(
            PythonLaunchDescriptionSource([robot_spawn_launch]),
            launch_arguments=[
                ('use_sim_time', LaunchConfiguration('use_sim_time')),
                ('setup_path', LaunchConfiguration('setup_path')),
                ('world', LaunchConfiguration('world')),
                ('rviz', LaunchConfiguration('rviz')),
                ('x', resolved['x']),
                ('y', resolved['y']),
                ('z', resolved['z']),
                ('yaw', resolved['yaw'])]
        )]

    # TimerAction, not a readiness poll: it fires once after a fixed launch-time
    # delay regardless of what the GUI is doing, it does not check or wait for
    # any signal. It is the accepted mechanism here only because gz-sim
    # Harmonic 8.11 exposes no GUI-side "scene finished loading" topic or
    # service to actually wait on (server-side /world/<w>/scene/info reflects
    # the server's scene graph, not what the GUI drew - see CLAUDE.md gotcha
    # and the renderer-gate report). A timer is the fallback, not the ideal.
    robot_spawn = TimerAction(
        period=LaunchConfiguration('spawn_delay'),
        actions=[OpaqueFunction(function=spawn_with_world_pose)],
    )

    # Create launch description and add actions
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(gz_sim)
    ld.add_action(robot_spawn)
    return ld
