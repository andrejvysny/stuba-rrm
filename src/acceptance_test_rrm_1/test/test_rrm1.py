import unittest
import rostest
import rospy
from rrm_msgs.srv import Move
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, Point, Quaternion
from copy import deepcopy
import math
import tf


class TestRRM1(unittest.TestCase):
    def __init__(self, *args, **kwargs):
        super(TestRRM1, self).__init__(*args, **kwargs)
        rospy.init_node('test_rrm1_robot_control')
        rospy.wait_for_service("/move_absolute", timeout=1.0)
        rospy.wait_for_service("/move_absolute", timeout=1.0)
        self.move_absolute = rospy.ServiceProxy('/move_absolute', Move)
        self.move_relative = rospy.ServiceProxy('/move_relative', Move)

        self.lower_limits = [-1.62, -0.96, -0.96, -3.14, -2.2, 0]
        self.upper_limits = [1.62, 2.182, 2.182, 3.14, 2.2, 0.1]

    def test_joint_names(self):
        expected = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
        msg = rospy.wait_for_message("/joint_states", JointState, timeout=1.0)
        self.assertEqual(len(expected), len(msg.name))
        for i in range(0, len(expected)):
            self.assertEqual(msg.name[i], expected[i])

    def test_move(self):
        target = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
        result = self.move_absolute(target)
        self.assertTrue(result.success)
        msg = rospy.wait_for_message("/joint_states", JointState, timeout=1.0)
        self.assertEqual(len(target), len(msg.position))
        for i in range(0, len(target)):
            self.assertAlmostEqual(msg.position[i], target[i], 3, f"joint_{i} failed")

    def test_move_less_joints(self):
        target = [0.1, 0.1]
        result = self.move_absolute(target)
        self.assertFalse(result.success)

    def test_move_more_joints(self):
        target = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7]
        result = self.move_absolute(target)
        self.assertFalse(result.success)

    def test_lower_limits(self):
        valid_target = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
        for i in range(0, len(self.lower_limits)):
            invalid_target = deepcopy(valid_target)
            invalid_target[i] = self.lower_limits[i] - 0.01
            result = self.move_absolute(invalid_target)
            self.assertFalse(result.success, 'joint_' + str(i+1) + ' must be out of range!')

    def test_upper_limits(self):
        valid_target = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
        for i in range(0, len(self.upper_limits)):
            invalid_target = deepcopy(valid_target)
            invalid_target[i] = self.upper_limits[i] + 0.01
            result = self.move_absolute(invalid_target)
            self.assertFalse(result.success, 'joint_' + str(i+1) + ' must be out of range!')

    def test_move_relative(self):
        self.move_absolute([-0.1, 0.2, -0.2, 0.0, -0.3, 0.0])
        default_pose = rospy.wait_for_message("/joint_states", JointState, timeout=1.0)
        relative_target = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
        result = self.move_relative(relative_target)
        current_pose = rospy.wait_for_message("/joint_states", JointState, timeout=1.0)

        self.assertTrue(result.success)
        self.assertEqual(len(relative_target), len(current_pose.position))
        for i in range(0, len(relative_target)):
            self.assertAlmostEqual(current_pose.position[i], default_pose.position[i] + relative_target[i], 3, f"joint_{i} failed")

    def test_move_relative_less_joints(self):
        target = [0.1, 0.1]
        result = self.move_relative(target)
        self.assertFalse(result.success)

    def test_move_relative_more_joints(self):
        target = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7]
        result = self.move_relative(target)
        self.assertFalse(result.success)

    def test_lower_limits_relative_move(self):
        valid_target = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
        for i in range(0, len(self.lower_limits)):
            invalid_target = deepcopy(valid_target)
            invalid_target[i] = self.lower_limits[i] - 0.01
            result = self.move_relative(invalid_target)
            self.assertFalse(result.success, 'joint_' + str(i+1) + ' must be out of range!')

    def test_upper_limits_relative_move(self):
        valid_target = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
        for i in range(0, len(self.upper_limits)):
            invalid_target = deepcopy(valid_target)
            invalid_target[i] = self.upper_limits[i] + 0.01
            result = self.move_relative(invalid_target)
            self.assertFalse(result.success, 'joint_' + str(i+1) + ' must be out of range!')

    def test_fk_1(self):
        target = [0, 0, 0, 0, 0, 0]
        expected_fk = Pose(position=Point(0, 0, 0.606), orientation=Quaternion(0, 0, 0, 1.0))

        self.move_absolute(target)
        fk = rospy.wait_for_message("/tool_pose", Pose, timeout=1.0)
        self.assertAlmostEqual(expected_fk.position.x, fk.position.x, 3)
        self.assertAlmostEqual(expected_fk.position.y, fk.position.y, 3)
        self.assertAlmostEqual(expected_fk.position.z, fk.position.z, 3)
        self.assertAlmostEqual(expected_fk.orientation.x, fk.orientation.x, 2)
        self.assertAlmostEqual(expected_fk.orientation.y, fk.orientation.y, 2)
        self.assertAlmostEqual(expected_fk.orientation.z, fk.orientation.z, 2)
        self.assertAlmostEqual(expected_fk.orientation.w, fk.orientation.w, 2)

    def test_fk_2(self):
        target = [0, 0, 0, 0, math.pi/2, 0]
        expected_fk = Pose(position=Point(0.15, 0, 0.4561), orientation=Quaternion(0, 0.7071, 0, 0.7071))

        self.move_absolute(target)
        fk = rospy.wait_for_message("/tool_pose", Pose, timeout=1.0)
        self.assertAlmostEqual(expected_fk.position.x, fk.position.x, 3)
        self.assertAlmostEqual(expected_fk.position.y, fk.position.y, 3)
        self.assertAlmostEqual(expected_fk.position.z, fk.position.z, 3)
        self.assertAlmostEqual(expected_fk.orientation.x, fk.orientation.x, 2)
        self.assertAlmostEqual(expected_fk.orientation.y, fk.orientation.y, 2)
        self.assertAlmostEqual(expected_fk.orientation.z, fk.orientation.z, 2)
        self.assertAlmostEqual(expected_fk.orientation.w, fk.orientation.w, 2)

    def test_fk_3(self):
        target = [math.pi/2, -0.2, 0.707, -math.pi/2, math.pi/2, 0.1]
        expected_fk = Pose(position=Point(0.25, 0.08252, 0.42013), orientation=Quaternion(-0.17734, 0.68451, -0.17734, 0.68451))

        self.move_absolute(target)
        fk = rospy.wait_for_message("/tool_pose", Pose, timeout=1.0)
        self.assertAlmostEqual(expected_fk.position.x, fk.position.x, 3)
        self.assertAlmostEqual(expected_fk.position.y, fk.position.y, 3)
        self.assertAlmostEqual(expected_fk.position.z, fk.position.z, 3)
        self.assertAlmostEqual(expected_fk.orientation.x, fk.orientation.x, 2)
        self.assertAlmostEqual(expected_fk.orientation.y, fk.orientation.y, 2)
        self.assertAlmostEqual(expected_fk.orientation.z, fk.orientation.z, 2)
        self.assertAlmostEqual(expected_fk.orientation.w, fk.orientation.w, 2)

    def test_tf(self):
        listener = tf.TransformListener()
        listener.waitForTransform("/base_link", "/tool0_calculated", rospy.Time(), rospy.Duration(1.0))
        listener.lookupTransform('/base_link', '/tool0_calculated', rospy.Time(0))

if __name__ == "__main__":
    rostest.rosrun("test_rrm1", "test_rrm1", TestRRM1)

