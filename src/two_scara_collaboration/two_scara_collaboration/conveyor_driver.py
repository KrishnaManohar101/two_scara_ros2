#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetEntityState
from geometry_msgs.msg import Twist
import time

class ConveyorDriver(Node):
    def __init__(self):
        super().__init__('conveyor_driver')
        
        self.set_state_client = self.create_client(SetEntityState, '/gazebo/set_entity_state')
        
        from gazebo_msgs.msg import ModelStates
        self.create_subscription(
            ModelStates,
            '/gazebo/model_states',
            self.model_states_callback,
            10
        )
        
        self.block_targets = []
        self.belt_velocity = 0.5
        
        # 10 Hz Control Loop
        self.create_timer(0.1, self.drive_conveyor)
        
        self.get_logger().info('Conveyor Driver Initialized')

    def model_states_callback(self, msg):
        self.block_targets = []
        for i, name in enumerate(msg.name):
            if 'cylinder_block' in name:
                z = msg.pose[i].position.z
                x = msg.pose[i].position.x
                
                # Active if on belt (Z < 0.5)
                if z < 0.5:
                     # Stop Sensor Logic:
                     # Stop if inside reach [0.5 to 2.5] (Robot Base X=2.0, Reach 1.8)
                     if x > 0.5 and x < 2.5:
                         target_vel = 0.0
                     elif x >= 2.5:
                         target_vel = self.belt_velocity # Move out after processing
                     else:
                         target_vel = self.belt_velocity # Move towards workspace
                         
                     self.block_targets.append((name, target_vel))
        
        if self.block_targets:
            print(f"DEBUG: Driving {len(self.block_targets)} blocks. First: {self.block_targets[0][0]} -> {self.block_targets[0][1]} m/s")

    def drive_conveyor(self):
        for name, vel in self.block_targets:
            self.set_block_velocity(name, vel)

    def set_block_velocity(self, name, velocity):
        req = SetEntityState.Request()
        req.state = ModelState()
        req.state.model_name = name
        req.state.twist.linear.x = float(velocity)
        req.state.twist.angular.z = 0.0
        req.state.reference_frame = 'world'
        self.set_state_client.call_async(req)

def main(args=None):
    rclpy.init(args=args)
    node = ConveyorDriver()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
