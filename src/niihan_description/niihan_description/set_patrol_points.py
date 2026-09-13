#!/usr/bin/env python3
"""CLI tool to set patrol waypoints for the NIIHAN waypoint patrol system.

Usage examples:
  ros2 run niihan_description set_patrol_points --preset perimeter --auto-start
  ros2 run niihan_description set_patrol_points --points 'A:-5,3 B:5,3 C:5,-3 D:-5,-3'
  ros2 run niihan_description set_patrol_points --interactive
  ros2 run niihan_description set_patrol_points --from-file my_route.yaml --auto-start
  ros2 run niihan_description set_patrol_points --save current_route.yaml
"""

import argparse
import os
import sys
import time

import yaml

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseArray
from std_msgs.msg import Header, String


# ── Built-in presets (also stored in patrol_presets.yaml) ────────────
PRESETS = {
    'perimeter': {
        'description': 'Large rectangular perimeter patrol',
        'waypoints': [
            ('A', -15.0, -10.0),
            ('B', 15.0, -10.0),
            ('C', 15.0, 10.0),
            ('D', -15.0, 10.0),
        ],
    },
    'inner': {
        'description': 'Patrol around the central foundation',
        'waypoints': [
            ('A', -5.0, -4.0),
            ('B', 5.0, -4.0),
            ('C', 5.0, 4.0),
            ('D', -5.0, 4.0),
        ],
    },
    'cross': {
        'description': 'Diamond / cross patrol pattern',
        'waypoints': [
            ('A', -12.0, 0.0),
            ('B', 0.0, -8.0),
            ('C', 12.0, 0.0),
            ('D', 0.0, 8.0),
        ],
    },
    'dock': {
        'description': 'Dock to center and back',
        'waypoints': [
            ('A', -8.0, 0.0),
            ('B', 0.0, 0.0),
            ('C', 8.0, 0.0),
            ('D', 0.0, 5.0),
        ],
    },
}


def _parse_points_string(s: str) -> list[tuple[str, float, float]]:
    """Parse 'A:x,y B:x,y C:x,y D:x,y' format."""
    waypoints = []
    for token in s.strip().split():
        if ':' not in token:
            raise ValueError(f"Invalid point format '{token}', expected 'LABEL:x,y'")
        label, coords = token.split(':', 1)
        parts = coords.split(',')
        if len(parts) != 2:
            raise ValueError(f"Invalid coords in '{token}', expected 'x,y'")
        x, y = float(parts[0]), float(parts[1])
        waypoints.append((label.upper(), x, y))
    return waypoints


def _load_yaml(path: str) -> list[tuple[str, float, float]]:
    """Load waypoints from a YAML file."""
    with open(path, 'r') as f:
        data = yaml.safe_load(f)

    # Support two formats:
    # 1. {'waypoints': {'A': {'x': ..., 'y': ...}, ...}}
    # 2. {'presets': {'name': {'waypoints': {'A': ...}}}}
    if 'waypoints' in data:
        wp_data = data['waypoints']
    elif 'presets' in data:
        # Use first preset
        first_key = next(iter(data['presets']))
        wp_data = data['presets'][first_key]['waypoints']
    else:
        raise ValueError('YAML must contain "waypoints" or "presets" key')

    result = []
    for label, coords in sorted(wp_data.items()):
        result.append((str(label), float(coords['x']), float(coords['y'])))
    return result


def _save_yaml(path: str, waypoints: list[tuple[str, float, float]]):
    """Save waypoints to a YAML file."""
    wp_dict = {}
    for label, x, y in waypoints:
        wp_dict[label] = {'x': x, 'y': y}
    data = {'waypoints': wp_dict}
    with open(path, 'w') as f:
        yaml.dump(data, f, default_flow_style=False, sort_keys=True)
    print(f'Saved {len(waypoints)} waypoints to {path}')


def _interactive_mode() -> list[tuple[str, float, float]]:
    """Interactively prompt user for waypoints."""
    waypoints = []
    labels = 'ABCDEFGHIJKLMNOPQRSTUVWXYZ'
    print('\n=== Interactive Waypoint Entry ===')
    print('Enter coordinates for each waypoint (x y), or press Enter to finish.\n')
    for i, label in enumerate(labels):
        try:
            raw = input(f'  Point {label} (x y): ').strip()
        except (EOFError, KeyboardInterrupt):
            print()
            break
        if not raw:
            break
        parts = raw.replace(',', ' ').split()
        if len(parts) != 2:
            print(f'    ⚠ Expected two numbers, skipping.')
            continue
        try:
            x, y = float(parts[0]), float(parts[1])
        except ValueError:
            print(f'    ⚠ Invalid numbers, skipping.')
            continue
        waypoints.append((label, x, y))
        print(f'    ✓ {label} = ({x:.2f}, {y:.2f})')

    if not waypoints:
        print('No waypoints entered.')
    return waypoints


class SetPatrolPointsNode(Node):
    """Temporary ROS 2 node to publish patrol waypoints."""

    def __init__(self):
        super().__init__('set_patrol_points')
        self._wp_pub = self.create_publisher(PoseArray, '/patrol/set_waypoints', 10)
        self._cmd_pub = self.create_publisher(String, '/patrol/command', 10)

    def _wait_for_subscribers(self, topic: str, timeout: float = 10.0) -> bool:
        """Allow DDS discovery to complete before publishing a one-shot message."""
        deadline = time.monotonic() + timeout
        while self.count_subscribers(topic) == 0 and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.count_subscribers(topic) > 0

    def _flush(self):
        """Keep this short-lived node alive long enough for delivery."""
        for _ in range(5):
            rclpy.spin_once(self, timeout_sec=0.1)

    def publish_waypoints(self, waypoints: list[tuple[str, float, float]]):
        """Publish a PoseArray with the given waypoints."""
        if not self._wait_for_subscribers('/patrol/set_waypoints'):
            self.get_logger().warn(
                'No waypoint subscriber discovered; is waypoint_patrol running?')
        pa = PoseArray()
        pa.header = Header(frame_id='map', stamp=self.get_clock().now().to_msg())
        for label, x, y in waypoints:
            pose = Pose()
            pose.position.x = x
            pose.position.y = y
            pose.position.z = 0.0
            pose.orientation.w = 1.0
            pa.poses.append(pose)
        self._wp_pub.publish(pa)
        self._flush()
        self._wp_pub.publish(pa)
        self._flush()

    def send_command(self, cmd: str):
        """Send a patrol command."""
        if not self._wait_for_subscribers('/patrol/command'):
            self.get_logger().warn(
                'No patrol command subscriber discovered; is waypoint_patrol running?')
        msg = String()
        msg.data = cmd
        self._cmd_pub.publish(msg)
        self._flush()
        self._cmd_pub.publish(msg)
        self._flush()


def main(args=None):
    parser = argparse.ArgumentParser(
        description='Set patrol waypoints for NIIHAN robot',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog='''
Examples:
  ros2 run niihan_description set_patrol_points --preset perimeter --auto-start
  ros2 run niihan_description set_patrol_points --points "A:-5,3 B:5,3 C:5,-3 D:-5,-3"
  ros2 run niihan_description set_patrol_points --interactive
  ros2 run niihan_description set_patrol_points --from-file route.yaml --auto-start
  ros2 run niihan_description set_patrol_points --list-presets
        ''')

    group = parser.add_mutually_exclusive_group()
    group.add_argument('--preset', type=str, choices=list(PRESETS.keys()),
                       help='Use a preset patrol route')
    group.add_argument('--points', type=str,
                       help='Set specific points: "A:x,y B:x,y C:x,y D:x,y"')
    group.add_argument('--interactive', action='store_true',
                       help='Interactive mode to enter waypoints')
    group.add_argument('--from-file', type=str, metavar='FILE',
                       help='Load waypoints from a YAML file')
    group.add_argument('--list-presets', action='store_true',
                       help='List available preset routes')

    parser.add_argument('--save', type=str, metavar='FILE',
                        help='Save the waypoints to a YAML file')
    parser.add_argument('--auto-start', action='store_true',
                        help='Automatically start patrol after setting points')
    parser.add_argument('--command', type=str,
                        choices=['start', 'stop', 'pause', 'resume',
                                 'clear', 'reverse', 'skip'],
                        help='Send a patrol command (no waypoints needed)')

    parsed_args = parser.parse_args()

    # ── List presets ────────────────────────────────────────────────
    if parsed_args.list_presets:
        print('\nAvailable patrol presets:\n')
        for name, info in PRESETS.items():
            wps = info['waypoints']
            wp_str = ' → '.join(f'{l}({x},{y})' for l, x, y in wps)
            print(f'  {name:12s}: {info["description"]}')
            print(f'                 {wp_str}\n')
        return

    # ── Determine waypoints ─────────────────────────────────────────
    waypoints = None

    if parsed_args.preset:
        preset = PRESETS[parsed_args.preset]
        waypoints = preset['waypoints']
        print(f'\n✓ Using preset: {parsed_args.preset} – {preset["description"]}')

    elif parsed_args.points:
        try:
            waypoints = _parse_points_string(parsed_args.points)
        except ValueError as e:
            print(f'Error: {e}', file=sys.stderr)
            sys.exit(1)

    elif parsed_args.interactive:
        waypoints = _interactive_mode()
        if not waypoints:
            sys.exit(0)

    elif parsed_args.from_file:
        try:
            waypoints = _load_yaml(parsed_args.from_file)
        except (OSError, ValueError, yaml.YAMLError) as e:
            print(f'Error loading file: {e}', file=sys.stderr)
            sys.exit(1)

    # ── Save if requested ───────────────────────────────────────────
    if parsed_args.save and waypoints:
        _save_yaml(parsed_args.save, waypoints)

    # ── Check if there's anything to publish ────────────────────────
    if waypoints is None and parsed_args.command is None:
        parser.print_help()
        sys.exit(1)

    # ── ROS 2 publish ───────────────────────────────────────────────
    rclpy.init(args=args)
    node = SetPatrolPointsNode()

    try:
        if waypoints:
            print(f'\nPublishing {len(waypoints)} waypoints:')
            for label, x, y in waypoints:
                print(f'  {label}: ({x:.2f}, {y:.2f})')
            node.publish_waypoints(waypoints)
            print('✓ Waypoints published to /patrol/set_waypoints')

        if parsed_args.command:
            node.send_command(parsed_args.command)
            print(f'✓ Command "{parsed_args.command}" sent to /patrol/command')

        if parsed_args.auto_start and waypoints:
            node.send_command('start')
            print('✓ Auto-start command sent')

        print()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
