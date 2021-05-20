#!/bin/python3

import rospy
from geometry_msgs.msg import PoseStamped, PointStamped, Quaternion
from nav_msgs.srv import GetPlan
import tf

import numpy as np

class Tracker:
    # parameters
    # follower
    MINIMAL_DISTANCE = 0.5
    HOP_DISTANCE = 0.1
    START_HOP = 0.2
    YAW_RANGE = np.pi / 2  # +/- 90 deg

    # plan pruner
    MAX_LENGTH_FACTOR = 3.0

    # misc
    DURATION_TOLERANCE = 0.01

    def __init__(self) -> None:
        self.tf_listener = tf.TransformListener()

        self.target_subscriber = rospy.Subscriber("clicked_point", PointStamped, self.on_target_received)
        self.goal_publisher = rospy.Publisher("move_base_simple/goal", PoseStamped, queue_size=1)

    def find_target(self, target_in_map: PointStamped):
        dx = self.START_HOP
        while True:
            dx += self.HOP_DISTANCE
            print("dx = {}".format(dx))

            # get base position
            try:
                target_in_map.header.stamp = rospy.Time.now() - rospy.Duration(self.DURATION_TOLERANCE)
                target_in_base = self.tf_listener.transformPoint("/base_link", target_in_map)
            except Exception as e:
                return None

            # prune by angle
            yaw = np.arctan2(target_in_base.point.y, target_in_base.point.x)
            if np.abs(yaw) >= self.YAW_RANGE:
                return None

            # calc target
            dist = np.linalg.norm([target_in_base.point.x, target_in_base.point.y])
            if dist - dx <= self.MINIMAL_DISTANCE:
                return None

            # make a plan
            pose_start = PoseStamped()
            pose_start.header.frame_id = "base_link"
            pose_start.header.stamp = rospy.Time.now() - rospy.Duration(self.DURATION_TOLERANCE)

            scaled_dist = dist - dx

            pose_end = PoseStamped()
            pose_end.header.frame_id = "base_link"
            pose_end.header.stamp = rospy.Time.now() - rospy.Duration(self.DURATION_TOLERANCE)

            pose_end.pose.position.x = target_in_base.point.x / dist * scaled_dist
            pose_end.pose.position.y = target_in_base.point.y / dist * scaled_dist
            pose_end.pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0, 0, yaw))

            try:
                pose_start_map = self.tf_listener.transformPose("map", pose_start)
                pose_end_map = self.tf_listener.transformPose("map", pose_end)
            except:
                continue

            rospy.wait_for_service("/move_base/make_plan")
            get_plan = rospy.ServiceProxy("/move_base/make_plan", GetPlan)
            plan = get_plan(pose_start_map, pose_end_map, 0).plan

            # check plan
            if not plan.poses:
                print("no plan")
                continue

            plan_len = sum([np.linalg.norm([
                plan.poses[i].pose.position.x - plan.poses[i - 1].pose.position.x,
                plan.poses[i].pose.position.y - plan.poses[i - 1].pose.position.y]) for i in range(1, len(plan.poses))])
            min_len = np.linalg.norm([pose_end.pose.position.x, pose_end.pose.position.y])

            if plan_len >= self.MAX_LENGTH_FACTOR * min_len:
                continue

            return pose_end_map

    def on_target_received(self, target: PointStamped):
        # get target
        track_pos = self.find_target(target)
        print(track_pos)

        if track_pos:
            self.goal_publisher.publish(track_pos)


def main():
    rospy.init_node("tracker")

    tracker = Tracker()
    rospy.spin()


if __name__ == "__main__":
    main()
