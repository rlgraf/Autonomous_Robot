#!/usr/bin/env python3
"""
Data Logger Node - Optimized
Logs experiment data to CSV on shutdown
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Bool
import csv
import os
from ament_index_python.packages import get_package_share_directory
import yaml


class DataLoggerNode(Node):

    def __init__(self):
        super().__init__('data_logger')
        
        # Load configuration
        pkg = get_package_share_directory('mobile_robot')
        battery_yaml = os.path.join(pkg, 'parameters', 'battery_tunable_parameters.yaml')
        
        with open(battery_yaml, 'r') as f:
            config = yaml.safe_load(f)
        
        bat_params = config['battery_node']['ros__parameters']
        rech_params = config['auto_recharge_node']['ros__parameters']
        
        # Store configuration data
        self.charging_stations = [tuple(s) for s in bat_params['charging_stations']]
        self.max_linear = rech_params['max_linear']
        self.max_angular = rech_params['max_angular']
        
        # Load cylinder positions from cache
        self.cylinder_positions = self._load_cylinder_positions()
        
        # Runtime data
        self.visited_cylinders = []  
        self.recharge_count = 0
        self.last_recharge_active = False
        
        # Track robot position to detect dwelling
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.dwelling_position = None  # (x, y) where robot is currently dwelling

        self.create_subscription(
            Float32MultiArray, '/visited_cylinders',
            self._visited_callback, 10
        )

        self.create_subscription(
            Bool, '/recharge_active',
            self._recharge_callback, 10
        )
        
        
        # CSV file setup
        data_dir = os.path.expanduser('~/Autonomous_Robot/src/mobile_robot/data')
        os.makedirs(data_dir, exist_ok=True)
        self.csv_file = os.path.join(data_dir, 'experiment.csv')
        
        self.get_logger().info(f'Data logger ready. Tracking {len(self.cylinder_positions)} cylinders')
    

    def _load_cylinder_positions(self):
        """Load cylinder positions from cache file"""
        data_dir = os.path.expanduser('~/Autonomous_Robot/src/mobile_robot/data')
        cache_file = os.path.join(data_dir, 'cylinder_positions.txt')
        cylinders = []
        
        if not os.path.exists(cache_file):
            self.get_logger().warn(f'Cylinder cache not found: {cache_file}')
            return cylinders
        
        try:
            with open(cache_file, 'r') as f:
                for line in f:
                    x, y = line.strip().split(',')
                    cylinders.append((float(x), float(y)))
            
            self.get_logger().info(f'Loaded {len(cylinders)} cylinders from cache')
            
        except Exception as e:
            self.get_logger().error(f'Failed to load cylinders: {e}')
        
        return cylinders

    def _visited_callback(self, msg: Float32MultiArray):
        """Record when robot completes dwelling at a cylinder"""
        if len(msg.data) >= 2:
            x, y = msg.data[0], msg.data[1]
            self.visited_cylinders.append((x, y))
            self.get_logger().info(
                f'Cylinder visited at ({x:.2f}, {y:.2f}) - '
                f'Total: {len(self.visited_cylinders)}'
            )
    
    def _recharge_callback(self, msg: Bool):
        """Count recharge cycles"""
        if msg.data and not self.last_recharge_active:
            self.recharge_count += 1
            self.get_logger().info(f'Recharge cycle #{self.recharge_count}')
        
        self.last_recharge_active = msg.data
    
    def save_data(self):
        """Write all collected data to CSV file - one coordinate per row"""
        try:
            with open(self.csv_file, 'w', newline='') as f:
                writer = csv.writer(f)
            
                # Summary section
                writer.writerow(['EXPERIMENT SUMMARY'])
                writer.writerow(['Max Linear Speed (m/s)', f'{self.max_linear:.3f}'])
                writer.writerow(['Max Angular Speed (rad/s)', f'{self.max_angular:.3f}'])
                writer.writerow(['Recharge Cycles', self.recharge_count])
                writer.writerow(['Cylinders Visited', f'{len(self.visited_cylinders)}/{len(self.cylinder_positions)}'])
                writer.writerow([])  # Blank line
            
                # Recharge stations section
                writer.writerow(['RECHARGE STATIONS'])
                writer.writerow(['Station #', 'X', 'Y'])
                for i, (x, y) in enumerate(self.charging_stations):
                    writer.writerow([i+1, f'{x:.3f}', f'{y:.3f}'])
                writer.writerow([])  # Blank line
            
                # All cylinders section
                writer.writerow(['ALL CYLINDERS'])
                writer.writerow(['Cylinder #', 'X', 'Y'])
                for i, (x, y) in enumerate(self.cylinder_positions):
                    writer.writerow([i+1, f'{x:.3f}', f'{y:.3f}'])
                writer.writerow([])  # Blank line
            
                # Visited cylinders section
                visited_list = sorted(list(self.visited_cylinders))
                writer.writerow(['VISITED CYLINDERS'])
                writer.writerow(['Visit #', 'X', 'Y'])
                for i, (x, y) in enumerate(visited_list):
                    writer.writerow([i+1, f'{x:.3f}', f'{y:.3f}'])
        
            self.get_logger().info(f'✓ Data saved to {self.csv_file}')
            self.get_logger().info(
                f'Summary: {len(self.visited_cylinders)}/{len(self.cylinder_positions)} '
                f'cylinders visited, {self.recharge_count} recharge cycles'
            )
        
        except Exception as e:
            self.get_logger().error(f'Failed to save data: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = DataLoggerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\n[Data Logger] Saving experiment data...')
    finally:
        node.save_data()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()