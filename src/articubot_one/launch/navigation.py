import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
import torch.nn.functional as F
from torch.distributions import Normal
import rclpy
from rclpy.node import Node
import json

from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Twist, Point, Pose
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from nav_msgs.msg import Odometry

from typing import List, Dict, Tuple

class PPONetwork(nn.Module):
    def __init__(self, state_dim, action_dim):
        super().__init__()
        # Actor network
        self.actor = nn.Sequential(
            nn.Linear(state_dim, 128),
            nn.ReLU(),
            nn.Linear(128, 64),
            nn.ReLU(),
            nn.Linear(64, action_dim * 2)  # Mean and std for each action
        )
        
        # Critic network
        self.critic = nn.Sequential(
            nn.Linear(state_dim, 128),
            nn.ReLU(),
            nn.Linear(128, 64),
            nn.ReLU(),
            nn.Linear(64, 1)
        )
    
    def forward(self, state):
        # Actor output
        actor_output = self.actor(state)
        mean, std = torch.chunk(actor_output, 2, dim=1)
        std = F.softplus(std) + 1e-5  # Ensure positive standard deviation
        
        # Critic output
        value = self.critic(state)
        
        return mean, std, value

class MultiAgentRLNavigation(Node):
    def __init__(self):
        super().__init__('multi_agent_rl_navigation')
        
        # Hyperparameters
        self.learning_rate = 0.001
        self.gamma = 0.99  # Discount factor
        self.eps_clip = 0.2  # PPO clip parameter
        self.epochs = 3
        self.num_robots = 4
        
        # Robot Database
        self.robot_database = {
            robot_id: {
                'position': None,
                'obstacles': None,
                'scan_ranges': None,
                'status': 'idle'  # idle, navigating, completed
            } for robot_id in range(1, self.num_robots + 1)
        }
        
        # Initialize networks for each robot
        self.networks = {
            robot_id: PPONetwork(state_dim=20, action_dim=2) 
            for robot_id in range(1, self.num_robots + 1)
        }
        
        # Optimizers
        self.optimizers = {
            robot_id: optim.Adam(net.parameters(), lr=self.learning_rate)
            for robot_id, net in self.networks.items()
        }
        
        # Goal tracking
        self.global_goals = {}
        
        # ROS Subscribers
        # Subscribe to environment data from the mapping node
        self.environment_data_sub = self.create_subscription(
            String, 'environment_data', self.update_environment_data, 10
        )
        
        # Subscribe to robot data from previous mapping node
        self.robot_data_sub = self.create_subscription(
            String, 'robot_data', self.update_robot_database, 10
        )
        
        # Publishers for navigation commands
        self.cmd_vel_publishers = {
            robot_id: self.create_publisher(Twist, f'/R{robot_id}/cmd_vel_unstamped', 10)
            for robot_id in range(1, self.num_robots + 1)
        }
        
        # Goal publishers
        self.goal_publishers = {
            robot_id: self.create_publisher(PoseStamped, f'/R{robot_id}/goal', 10)
            for robot_id in range(1, self.num_robots + 1)
        }
        
        self.get_logger().info('Multi-Agent RL Navigation Node Initialized')
    
    def update_environment_data(self, msg: String):
        """Update environment data (obstacles and robot positions)"""
        try:
            environment_data = json.loads(msg.data)
            robot_id = environment_data['local_id']  # Extract robot ID
            
            # Update obstacles
            obstacles = environment_data['new_data']['obstacles']
            
            # Update the robot database with the obstacles
            self.robot_database[robot_id]['obstacles'] = obstacles
            self.get_logger().info(f'Updated obstacles for robot {robot_id}: {obstacles}')
        
        except Exception as e:
            self.get_logger().error(f'Error processing environment data: {e}')
    
    def update_robot_database(self, msg: String):
        """Update robot database with latest information"""
        try:
            robot_data = json.loads(msg.data)
            robot_id = robot_data['robot_id']  # Extract robot ID
            
            # Update robot database with the latest position and other details
            self.robot_database[robot_id].update({
                'position': {
                    'x': robot_data['X'],
                    'y': robot_data['Y']
                },
                'scan_ranges': robot_data['scan_ranges']
            })
            
            # Trigger navigation logic if needed
            self.check_and_trigger_navigation(robot_id)
        
        except Exception as e:
            self.get_logger().error(f'Error processing robot data: {e}')
    
    def check_and_trigger_navigation(self, robot_id: int):
        """Check if robot can start navigation to assigned goal"""
        robot_info = self.robot_database[robot_id]
        
        # Conditions for navigation
        if (robot_id in self.global_goals and 
            robot_info['status'] == 'idle' and 
            robot_info['position'] is not None):
            
            # Prepare navigation
            goal = self.global_goals[robot_id]
            current_pos = robot_info['position']
            
            # Check if goal is reachable and no obstacles
            if not robot_info['obstacles']:
                self.start_navigation(robot_id, current_pos, goal)
    
    def start_navigation(self, robot_id: int, current_pos: dict, goal: Point):
        """Initiate navigation for a specific robot"""
        # Update robot status
        self.robot_database[robot_id]['status'] = 'navigating'
        
        # Prepare goal message
        goal_msg = PoseStamped()
        goal_msg.pose.position.x = goal.x
        goal_msg.pose.position.y = goal.y
        
        # Publish goal
        self.goal_publishers[robot_id].publish(goal_msg)
        self.get_logger().info(f'Robot {robot_id} navigating to goal: {goal}')
    
    def set_global_goal(self, goal_point: Point):
        """Assign goal to the most suitable available robot"""
        # Find available robots
        available_robots = [
            rid for rid, info in self.robot_database.items() 
            if info['status'] == 'idle' and info['position'] is not None
        ]
        
        if not available_robots:
            self.get_logger().warn("No robots available for goal!")
            return
        
        # Select closest robot based on Euclidean distance
        def distance(robot_id):
            pos = self.robot_database[robot_id]['position']
            return np.sqrt(
                (pos['x'] - goal_point.x)**2 + 
                (pos['y'] - goal_point.y)**2
            )
        
        closest_robot = min(available_robots, key=distance)
        
        # Assign goal
        self.global_goals[closest_robot] = goal_point
        self.get_logger().info(f"Robot {closest_robot} assigned to goal: {goal_point}")
    
    def run_reinforcement_navigation(self):
        """Run RL navigation for robots with assigned goals"""
        for robot_id, goal in self.global_goals.items():
            robot_info = self.robot_database[robot_id]
            
            # Ensure we have necessary information
            if (robot_info['position'] is None or 
                robot_info['scan_ranges'] is None):
                continue
            
            # Convert scan ranges to state
            state = self.process_scan_to_state(robot_info['scan_ranges'])
            
            # Run PPO policy
            self.navigate_robot(robot_id, state, goal)
    
    def process_scan_to_state(self, scan_ranges):
        """Convert scan ranges to normalized state"""
        ranges = np.array(scan_ranges)
        normalized_ranges = (ranges - ranges.min()) / (ranges.max() - ranges.min())
        return torch.FloatTensor(normalized_ranges)
    
    def navigate_robot(self, robot_id: int, state: torch.Tensor, goal: Point):
        """Navigate robot using learned PPO policy"""
        network = self.networks[robot_id]
        
        # Get action from policy
        mean, std, value = network(state)
        dist = Normal(mean, std)
        
        # Sample action
        action = dist.sample()
        
        # Convert action to velocity command
        twist = self.action_to_twist(action)
        
        # Publish velocity command
        self.cmd_vel_publishers[robot_id].publish(twist)
    
    def action_to_twist(self, action) -> Twist:
        """Convert RL action to ROS twist message"""
        twist = Twist()
        twist.linear.x = float(np.clip(action[0].item(), -0.5, 0.5))
        twist.angular.z = float(np.clip(action[1].item(), -0.5, 0.5))
        return twist

def main(args=None):
    rclpy.init(args=args)
    node = MultiAgentRLNavigation()
    
    try:
        # Example of setting a global goal for demonstration
        goal = Point()
        goal.x = 5.0
        goal.y = 5.0
        node.set_global_goal(goal)
        
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutdown requested')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
