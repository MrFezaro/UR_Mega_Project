import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration


JOINT_NAMES = [
    'shoulder_pan_joint',
    'shoulder_lift_joint',
    'elbow_joint',
    'wrist_1_joint',
    'wrist_2_joint',
    'wrist_3_joint',
]

HOME_POSITIONS = [0.0, -1.5708, 1.5708, -1.5708, -1.5708, 0.0]


class HomePositionNode(Node):
    def __init__(self):
        super().__init__('home_position_node')
        self._client = ActionClient(
            self,
            FollowJointTrajectory,
            '/scaled_joint_trajectory_controller/follow_joint_trajectory',
        )
        self.get_logger().info('Waiting for controller...')
        self._client.wait_for_server()
        self.send_home()

    def send_home(self):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINT_NAMES
        goal.trajectory.points = [
            JointTrajectoryPoint(
                positions=HOME_POSITIONS,
                time_from_start=Duration(sec=5),
            )
        ]
        self.get_logger().info('Sending home goal...')
        future = self._client.send_goal_async(goal)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal REJECTED!')
            return
        self.get_logger().info('Goal ACCEPTED, executing...')
        goal_handle.get_result_async().add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result().result
        if result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().info('Reached home position.')
        else:
            self.get_logger().error(f'Failed with error code: {result.error_code}  {result.error_string}')


def main(args=None):
    rclpy.init(args=args)
    node = HomePositionNode()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
