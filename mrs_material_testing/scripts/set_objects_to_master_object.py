#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped

class VirtualObjectReplicator:
    def __init__(self, follower_namespaces, arms):
        self.follower_topics = [
            f"/{robot}/{arm}/virtual_object/set_pose"
            for robot in follower_namespaces
            for arm in arms
        ]
        self.publishers = {
            topic: rospy.Publisher(topic, PoseStamped, queue_size=10)
            for topic in self.follower_topics
        }

        self.master_pose = None
        rospy.Subscriber("/virtual_object/object_pose", PoseStamped, self.master_pose_callback)

    def master_pose_callback(self, msg):
        self.master_pose = msg
        self.replicate_pose()

    def replicate_pose(self):
        if self.master_pose is not None:
            for topic, pub in self.publishers.items():
                pub.publish(self.master_pose)
                rospy.sleep(0.5)  # Optional: Sleep to allow for message processing
                rospy.loginfo(f"Published pose to {topic}")
        

if __name__ == "__main__":
    rospy.init_node("virtual_object_pose_replicator")

    # Roboter-IDs und Arme definieren
    follower_namespaces = ["mur620b", "mur620c"]
    arms = ["UR10_l", "UR10_r"]

    replicator = VirtualObjectReplicator(follower_namespaces, arms)

    rospy.spin()
