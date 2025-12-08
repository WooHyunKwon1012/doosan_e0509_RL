#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray


class JointStateBridge(Node):
    def __init__(self):
        super().__init__('joint_state_bridge')
        
        # /joint_states 구독
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        # /joint_input 발행
        self.publisher = self.create_publisher(
            Float32MultiArray,
            '/joint_input',
            10
        )
        
        self.get_logger().info('Joint State Bridge started: /joint_states -> /joint_input')

    def joint_state_callback(self, msg: JointState):
        # JointState에서 position만 추출
        # 필요시 특정 joint만 필터링 가능
        # e0509는 joint_1 ~ joint_6
        
        joint_positions = Float32MultiArray()
        
        # joint 순서대로 정렬 (joint_1, joint_2, ..., joint_6)
        sorted_positions = []
        joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
        
        for joint_name in joint_names:
            if joint_name in msg.name:
                idx = msg.name.index(joint_name)
                sorted_positions.append(msg.position[idx])
            else:
                sorted_positions.append(0.0)  # joint가 없으면 0으로
        
        joint_positions.data = sorted_positions
        self.publisher.publish(joint_positions)
        
        # 로그 (필요시 주석 처리)
        # self.get_logger().info(f'Published to /joint_input: {sorted_positions}')


def main(args=None):
    rclpy.init(args=args)
    node = JointStateBridge()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()