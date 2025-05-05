#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Pose, Twist
from tf.transformations import quaternion_from_euler
import math

def main():
    rospy.init_node('relative_pose_publisher')

    mur620b_UR10l_pub = rospy.Publisher('/mur620b/UR10_l/relative_pose_offset', Pose, queue_size=10)
    mur620b_UR10r_pub = rospy.Publisher('/mur620b/UR10_r/relative_pose_offset', Pose, queue_size=10)
    mur620c_UR10l_pub = rospy.Publisher('/mur620c/UR10_l/relative_pose_offset', Pose, queue_size=10)
    mur620c_UR10r_pub = rospy.Publisher('/mur620c/UR10_r/relative_pose_offset', Pose, queue_size=10)
    mur620b_UR10l_virtual_object_pub = rospy.Publisher('/mur620b/UR10_l/virtual_object/object_cmd_vel', Twist, queue_size=10)
    mur620b_UR10r_virtual_object_pub = rospy.Publisher('/mur620b/UR10_r/virtual_object/object_cmd_vel', Twist, queue_size=10)
    mur620c_UR10l_virtual_object_pub = rospy.Publisher('/mur620c/UR10_l/virtual_object/object_cmd_vel', Twist, queue_size=10)
    mur620c_UR10r_virtual_object_pub = rospy.Publisher('/mur620c/UR10_r/virtual_object/object_cmd_vel', Twist, queue_size=10)

    rate = rospy.Rate(10)  # 100 Hz
    duration = 20.0  # seconds
    steps = int(duration * 100)  # 500 steps
    wait = int(duration * 1)  # 500 steps
    d_start = 0.0
    d_end = 0.02
    d_step = (d_end - d_start) / steps
    phi_start = 0.0
    phi_end = 60.0
    omega = ((phi_end - phi_start) / 180 * math.pi) / duration

    for i in range(wait):
        pose_msg = Pose()
        pose_msg.position.z = 0
        pose_msg.orientation.x = 0
        pose_msg.orientation.y = 0
        pose_msg.orientation.z = 0
        pose_msg.orientation.w = 1


        # Mur620b UR10l
        pose_msg.position.x = d_start 
        pose_msg.position.y = 0
        mur620b_UR10l_pub.publish(pose_msg)

        # Mur620b UR10r
        pose_msg.position.x = 0
        pose_msg.position.y = d_start 
        mur620b_UR10r_pub.publish(pose_msg)

        # Mur620c UR10l
        pose_msg.position.x = -d_start
        pose_msg.position.y = 0
        mur620c_UR10l_pub.publish(pose_msg)

        # Mur620c UR10r
        pose_msg.position.x = 0
        pose_msg.position.y = -d_start
        mur620c_UR10r_pub.publish(pose_msg)


        rate.sleep()

    rospy.loginfo("Bewegung abgeschlossen.")

    input("Drücke Enter, um die Bewegung zu starten...")

    for i in range(steps):
        pose_msg = Pose()
        pose_msg.position.z = 0
        pose_msg.orientation.x = 0
        pose_msg.orientation.y = 0
        pose_msg.orientation.z = 0
        pose_msg.orientation.w = 1
        twist_msg = Twist()


        # Mur620b UR10l
        pose_msg.position.x = d_start + i * d_step
        pose_msg.position.y = 0
        mur620b_UR10l_pub.publish(pose_msg)
        twist_msg.angular.x = omega
        mur620b_UR10l_virtual_object_pub.publish(twist_msg)
        # Mur620b UR10r
        pose_msg.position.x = 0
        pose_msg.position.y = d_start + i * d_step
        mur620b_UR10r_pub.publish(pose_msg)
        twist_msg.angular.x = omega * 0.5
        mur620b_UR10r_virtual_object_pub.publish(twist_msg)

        # Mur620c UR10l
        pose_msg.position.x = -d_start - i * d_step
        pose_msg.position.y = 0
        mur620c_UR10l_pub.publish(pose_msg)
        twist_msg.angular.x = 0.0
        mur620c_UR10l_virtual_object_pub.publish(twist_msg)

        # Mur620c UR10r
        pose_msg.position.x = 0
        pose_msg.position.y = -d_start - i * d_step
        mur620c_UR10r_pub.publish(pose_msg)
        twist_msg.angular.x = omega * 0.5
        mur620c_UR10r_virtual_object_pub.publish(twist_msg)
        


        if rospy.is_shutdown():
            break

        rate.sleep()
    twist_msg = Twist()
    mur620c_UR10l_virtual_object_pub.publish(twist_msg)
    rospy.sleep(0.5)
    rospy.signal_shutdown("Bewegung abgeschlossen.")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
