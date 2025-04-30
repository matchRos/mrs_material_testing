#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler

def main():
    rospy.init_node('relative_pose_publisher')
    pub = rospy.Publisher('/mur620b/UR10_r/relative_pose', PoseStamped, queue_size=10)

    rate = rospy.Rate(100)  # 100 Hz
    duration = 5.0  # seconds
    steps = int(duration * 100)  # 500 steps
    wait = int(duration * 1)  # 500 steps
    y_start = 0.1
    y_end = 0.35
    y_step = (y_end - y_start) / steps

    # Startposition
    x = 0.0007672071499775082
    z = -0.0010315731500602277

    # Euler zu Quaternion
    roll = 3.13431331818871
    pitch = 1.57079632679
    yaw = 1.57079632679
    quat = quaternion_from_euler(roll, pitch, yaw)

    for i in range(wait):
        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = "base_link"  # oder passe es an dein Setup an

        pose_msg.pose.position.x = x
        pose_msg.pose.position.y = y_start
        pose_msg.pose.position.z = z

        pose_msg.pose.orientation.x = quat[0]
        pose_msg.pose.orientation.y = quat[1]
        pose_msg.pose.orientation.z = quat[2]
        pose_msg.pose.orientation.w = quat[3]

        pub.publish(pose_msg)
        rate.sleep()

    rospy.loginfo("Bewegung abgeschlossen.")

    input("Drücke Enter, um die Bewegung zu starten...")

    for i in range(steps):
        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = "base_link"  # oder passe es an dein Setup an

        pose_msg.pose.position.x = x
        pose_msg.pose.position.y = y_start + i * y_step
        pose_msg.pose.position.z = z

        pose_msg.pose.orientation.x = quat[0]
        pose_msg.pose.orientation.y = quat[1]
        pose_msg.pose.orientation.z = quat[2]
        pose_msg.pose.orientation.w = quat[3]

        pub.publish(pose_msg)
        rate.sleep()

    rospy.loginfo("Bewegung abgeschlossen.")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
