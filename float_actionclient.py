#!/usr/bin/env python

import rospy
import actionlib
#from my_package.msg import check_distanceAction,check_distanceActionGoal,RealTimeDistanceAction, RealTimeDistanceGoal
from my_package.msg import check_distanceAction, check_distanceActionGoal

class RealTimeDistanceClient:
    def __init__(self):
        rospy.init_node('real_time_distance_client')
        self.actionclient = actionlib.SimpleActionClient('real_time_distance_action', check_distanceAction)
        self.actionclient.wait_for_server()

    def send_goal(self, cmd_distance):
        goal = check_distanceActionGoal()
        goal.cmd_distance = cmd_distance
        print('set cmd_distance')

        self.actionclient.send_goal(goal)

        # wait for the result
        self.client.wait_for_result()

        # print result
        print(self.actionclient.get_result())


    def run(self):
        cmd_distance = 500.0  # Set the desired cmd_distance value here

        self.send_goal(cmd_distance)

        rospy.spin()


if __name__ == '__main__':
    client = RealTimeDistanceClient()
    client.run()
