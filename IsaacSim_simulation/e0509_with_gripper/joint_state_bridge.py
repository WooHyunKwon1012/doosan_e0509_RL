#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray, Float32


class JointStateBridge(Node):
    def __init__(self):
        super().__init__('joint_state_bridge')
        
        # 그리퍼 값 저장
        self.gripper_value = 0.0
        
        # /joint_states 구독 (MoveIt2에서 로봇 팔 6개 joint)
        self.joint_subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        # /gripper_input 구독 (GUI에서 그리퍼 값, 라디안)
        self.gripper_subscription = self.create_subscription(
            Float32,
            '/gripper_input',
            self.gripper_callback,
            10
        )
        
        # /joint_input 발행 (Isaac Sim으로 10개 DOF)
        self.publisher = self.create_publisher(
            Float32MultiArray,
            '/joint_input',
            10
        )
        
        # 마지막 joint 값 저장 (그리퍼 값만 변경될 때 사용)
        self.last_joint_positions = [0.0] * 6
        
        self.get_logger().info('Joint State Bridge started')
        self.get_logger().info('  Subscribing: /joint_states (6 joints) + /gripper_input (gripper)')
        self.get_logger().info('  Publishing: /joint_input (10 DOF)')
    
    def gripper_callback(self, msg: Float32):
        """그리퍼 값 업데이트 및 즉시 발행"""
        self.gripper_value = msg.data
        self.get_logger().info(f'Gripper value updated: {self.gripper_value} rad')
        
        # 그리퍼 값이 변경되면 즉시 /joint_input 발행
        self.publish_joint_input(self.last_joint_positions)

    def publish_joint_input(self, joint_positions):
        """10개 DOF 배열 생성 및 발행"""
        joint_array = Float32MultiArray()
        
        # 로봇 팔 6개 joint + 그리퍼 관련 4개
        full_positions = list(joint_positions)  # 6개 (index 0~5)
        full_positions.append(0.0)               # 7번째 (index 6)
        full_positions.append(self.gripper_value) # 8번째 (index 7) - 그리퍼 (라디안)
        full_positions.append(0.0)               # 9번째 (index 8)
        full_positions.append(0.0)               # 10번째 (index 9)
        
        joint_array.data = full_positions
        self.publisher.publish(joint_array)

    def joint_state_callback(self, msg: JointState):
        # JointState에서 position만 추출
        # e0509 로봇 팔: joint_1 ~ joint_6 (6개)
        
        # joint 순서대로 정렬 (joint_1 ~ joint_6)
        sorted_positions = []
        joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
        
        for joint_name in joint_names:
            if joint_name in msg.name:
                idx = msg.name.index(joint_name)
                sorted_positions.append(msg.position[idx])
            else:
                sorted_positions.append(0.0)  # joint가 없으면 0으로
        
        # 마지막 joint 값 저장
        self.last_joint_positions = sorted_positions
        
        # 10개 DOF 발행
        self.publish_joint_input(sorted_positions)


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