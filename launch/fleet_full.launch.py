"""
fleet_full.launch.py — the whole centralised fleet, one command.

    ros2 launch hospital_robot_description fleet_full.launch.py
    ros2 launch hospital_robot_description fleet_full.launch.py demo:=true

WHAT COMES UP
─────────────
Everything in this file runs on ONE machine. That is the architecture, not a
simulation shortcut: on hardware this is the Raspberry Pi / Jetson, and the
only thing that moves off it is rover_link's far end.

    Gazebo Classic + hospital_large.world      (the ward, and 4 rover bodies)
    robot_state_publisher x4                    on the brain
    Nav2 stack x4  (AMCL, planner, controller)  on the brain
    rover_link x4                               the ESP32 boundary
    fleet_brain                                 the single decision-maker
    task_console                                where work is submitted
    RViz

Startup is deliberately staggered: four Nav2 stacks is a heavy cold start,
and bringing them up together makes lifecycle activation race the transforms.

FLEET CONFIGURATION comes from config/fleet_brain.yaml, read here and handed
to the brain. Stations, weights, energy model and rover hardware profiles are
edited in that one file — never in this launch file, and never on a rover.
"""

import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, SetEnvironmentVariable,
                            TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

ROVERS = ['rover1', 'rover2', 'rover3', 'rover4']


def generate_launch_description():
    pkg = get_package_share_directory('hospital_robot_description')
    nav2_bringup = get_package_share_directory('nav2_bringup')
    gazebo_ros = get_package_share_directory('gazebo_ros')

    urdf = os.path.join(pkg, 'urdf', 'robot.urdf.xacro')
    world = os.path.join(pkg, 'worlds', 'hospital_large.world')
    map_yaml = os.path.join(pkg, 'maps', 'hospital_large.yaml')
    rviz_cfg = os.path.join(pkg, 'config', 'fleet.rviz')
    brain_yaml = os.path.join(pkg, 'config', 'fleet_brain.yaml')

    demo = LaunchConfiguration('demo')

    # ── fleet configuration: one file, read once, shared by every node ────
    cfg = yaml.safe_load(open(brain_yaml))
    brain_p = dict(cfg['fleet_brain']['ros__parameters'])
    st = cfg['fleet_stations']['ros__parameters']
    rv = cfg['fleet_rovers']['ros__parameters']

    station_names = list(st['names'])
    station_data = [float(v) for n in station_names for v in st[n]]

    rover_docks = [rv[r]['dock'] for r in ROVERS]
    rover_dock_poses = [float(v) for r in ROVERS for v in rv[r]['dock_pose']]
    rover_batteries = [float(rv[r]['battery_wh']) for r in ROVERS]

    mp = yaml.safe_load(open(map_yaml))
    brain_p.pop('map_yaml', None)
    drain_mul = brain_p.pop('sim_drain_multiplier', 12.0)
    brain_p.update({
        'rovers': ROVERS,
        'station_names': station_names,
        'station_data': station_data,
        'rover_docks': rover_docks,
        'rover_dock_poses': rover_dock_poses,
        'rover_batteries': rover_batteries,
        'map_pgm': os.path.join(pkg, 'maps', mp['image']),
        'map_resolution': float(mp['resolution']),
        'map_origin': [float(mp['origin'][0]), float(mp['origin'][1])],
        # The brain must predict energy on the same scale rover_link burns
        # it, or it will happily dispatch missions the pack cannot cover.
        # Both are 1.0 on real hardware.
        'energy_scale': float(drain_mul),
    })

    actions = [
        DeclareLaunchArgument('demo', default_value='false',
                              description='submit the scripted task sequence'),
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', os.path.join(pkg, '..')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_ros, 'launch', 'gazebo.launch.py')),
            launch_arguments={'world': world, 'verbose': 'true'}.items()),
    ]

    # ── robot_state_publisher + spawn, one per rover ──────────────────────
    for i, rid in enumerate(ROVERS):
        hx, hy, hyaw = rv[rid]['dock_pose']
        actions.append(Node(
            package='robot_state_publisher', executable='robot_state_publisher',
            namespace=rid, output='screen',
            parameters=[{
                'robot_description': ParameterValue(
                    Command(['xacro ', urdf, f' robot_name:={rid}']), value_type=str),
                'use_sim_time': True,
                'frame_prefix': f'{rid}/'}]))
        actions.append(TimerAction(period=8.0 + 2.0 * i, actions=[Node(
            package='gazebo_ros', executable='spawn_entity.py',
            arguments=['-topic', f'/{rid}/robot_description', '-entity', rid,
                       '-x', str(hx), '-y', str(hy), '-z', '0.15',
                       '-Y', str(hyaw)],
            output='screen')]))

    # ── TF bridge across the four namespaces ──────────────────────────────
    actions.append(Node(
        package='hospital_robot_description', executable='multi_robot_tf_bridge.py',
        name='multi_robot_tf_bridge', output='screen',
        parameters=[{'use_sim_time': True, 'rover_names': ROVERS}]))

    # ── Nav2 stacks — all four on the brain, staggered ────────────────────
    for i, rid in enumerate(ROVERS):
        actions.append(TimerAction(period=22.0 + 3.0 * i, actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_bringup, 'launch', 'bringup_launch.py')),
                launch_arguments={
                    'namespace': rid, 'map': map_yaml,
                    'params_file': os.path.join(pkg, 'config', f'nav2_params_{rid}.yaml'),
                    'use_sim_time': 'true', 'autostart': 'true',
                    'use_namespace': 'true'}.items())]))

    # ── seed AMCL at each rover's dock ────────────────────────────────────
    # set_initial_pose in the params file covers the normal case; this is the
    # belt-and-braces republish for the runs where AMCL activates before its
    # transform tree is ready and silently ignores the configured pose.
    for i, rid in enumerate(ROVERS):
        hx, hy, hyaw = rv[rid]['dock_pose']
        import math
        qz, qw = math.sin(hyaw / 2.0), math.cos(hyaw / 2.0)
        actions.append(TimerAction(period=46.0 + float(i), actions=[ExecuteProcess(
            cmd=['ros2', 'topic', 'pub', '--once', f'/{rid}/initialpose',
                 'geometry_msgs/msg/PoseWithCovarianceStamped',
                 ('{header: {frame_id: "map"}, pose: {pose: {'
                  f'position: {{x: {hx}, y: {hy}, z: 0.0}},'
                  f'orientation: {{z: {qz:.6f}, w: {qw:.6f}}}}},'
                  'covariance: [0.25,0,0,0,0,0, 0,0.25,0,0,0,0,'
                  ' 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0.068]}}')],
            output='screen')]))

    # ── rover_link x4: the only thing that touches the wheels ─────────────
    links = []
    for i, rid in enumerate(ROVERS):
        hx, hy, _ = rv[rid]['dock_pose']
        links.append(Node(
            package='hospital_robot_description', executable='rover_link.py',
            name=f'{rid}_link', output='screen',
            parameters=[{
                'use_sim_time': True,
                'rover_id': rid,
                'mode': 'sim',                     # 'hardware' talks UDP to an ESP32
                'esp_endpoint': rv[rid]['esp_endpoint'],
                'listen_port': 9101 + i,   # distinct per rover; see rover_link
                'battery_wh': float(rv[rid]['battery_wh']),
                'dock_x': float(rv[rid]['dock_pose'][0]),
                'dock_y': float(rv[rid]['dock_pose'][1]),
                'energy_wh_per_m': float(brain_p['energy_wh_per_m']),
                'energy_wh_per_rad': float(brain_p['energy_wh_per_rad']),
                'energy_idle_w': float(brain_p['energy_idle_w']),
                'sim_drain_multiplier': float(drain_mul)}]))
    actions.append(TimerAction(period=52.0, actions=links))

    # ── the brain ─────────────────────────────────────────────────────────
    actions.append(TimerAction(period=55.0, actions=[Node(
        package='hospital_robot_description', executable='fleet_brain.py',
        name='fleet_brain', output='screen', parameters=[brain_p])]))

    actions.append(TimerAction(period=58.0, actions=[Node(
        package='hospital_robot_description', executable='task_console.py',
        name='task_console', output='screen',
        parameters=[{'use_sim_time': True,
                     'station_names': station_names,
                     'station_data': station_data,
                     'demo': demo}])]))

    # ── RViz ──────────────────────────────────────────────────────────────
    # Launched through `env -u` because VS Code's snap injects LOCPATH,
    # GTK_PATH and friends pointing into /snap/code/..., which makes rviz2
    # load the snap's glibc locale data and abort with exit 127 before it
    # draws anything. Stripping those leaves the system libraries in place.
    actions.append(TimerAction(period=60.0, actions=[ExecuteProcess(
        cmd=['env', '-u', 'LOCPATH', '-u', 'GTK_PATH', '-u', 'GTK_EXE_PREFIX',
             '-u', 'GDK_PIXBUF_MODULEDIR', '-u', 'GDK_PIXBUF_MODULE_FILE',
             '-u', 'SNAP', '-u', 'GSETTINGS_SCHEMA_DIR',
             'rviz2', '-d', rviz_cfg, '--ros-args', '-p', 'use_sim_time:=true'],
        output='screen')]))

    return LaunchDescription(actions)
