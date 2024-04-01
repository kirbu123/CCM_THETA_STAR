#!/usr/bin/env python3
import rospy
import tf2_ros
import argparse
from sensor_msgs.msg import PointCloud2 
import numpy as np
import ros_numpy 

def get_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--velodyne_topic", type=str, default="/velodyne_points"
    )
    parser.add_argument(
        "--velodyne_topic_fusion",
        type=str,
        default="/points_fusion"
    )
    parser.add_argument(
        "--frame_id",
        type=str,
        default="velodyne"
    )
    return parser

class PointcloudIsaacNode:

    def __init__(self, args) -> None:
        rospy.init_node('pcd_node')

        self.sub_pcd = rospy.Subscriber(
            args.velodyne_topic,
            PointCloud2, 
            self.on_pcds
        )
        self.pcd_pub = rospy.Publisher(
            args.velodyne_topic_fusion,
            PointCloud2,
            queue_size=10
        )
        self.timer = rospy.Timer(rospy.Duration(1.5), self.on_publish)
        self.frame = args.frame_id
        self.pcd2 = None
        self.time =rospy.Time()

    def on_pcds(self, msg):
        if self.pcd2 is None:
            self.pcd2 = ros_numpy.numpify(msg)
        else:
            self.pcd2 = np.concatenate((self.pcd2, ros_numpy.numpify(msg)))
            self.frame = msg.header.frame_id
            self.time = msg.header.stamp

    def on_publish(self, event=None):
        if not self.pcd2 is None:
            msg_pub = ros_numpy.msgify(PointCloud2, self.pcd2)
            msg_pub.header.frame_id = self.frame
            msg_pub.header.stamp = self.time
            self.pcd_pub.publish(msg_pub)
            self.pcd2 = None
            
def main(args):
    node = PointcloudIsaacNode(args)
    rospy.spin()


if __name__ == '__main__':
    parser = get_parser()
    args = parser.parse_args()
    main(args)