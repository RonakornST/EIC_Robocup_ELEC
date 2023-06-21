#!/usr/bin/env python

import socket
import struct

import rospy
import actionlib
from my_package.msg import check_distanceAction, check_distanceActionResult, check_distanceActionFeedback
from std_msgs.msg import Float32

class RealTimeDistanceServer:
    def __init__(self, host, port_receive, port_send):
        rospy.init_node('real_time_distance_server')
        self.actionserver = actionlib.SimpleActionServer('real_time_distance_action', check_distanceAction, execute_cb=self.execute, auto_start=False)
        self.actionserver.start()
        self.pub = rospy.Publisher('real_time_distance', Float32, queue_size=10)
        self.cmd_distance = -1.0
        self.host = host
        self.port_receive = port_receive
        self.port_send = port_send
        self.sock = None
        self.client_socket = None
        self.setup_sockets()

    def setup_sockets(self):
        try:
            self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.client_socket.connect((self.host, self.port_send))
            self.sock = socket.socket()
            self.sock.connect((self.host, self.port_receive))
            rospy.loginfo('Connected to server')
            
            # Send initial message
            message = "Hello World".encode()
            self.sock.send(message)
            
            # Send initial data
            d = [0, 0]
            data_byte = struct.pack('!{}f'.format(len(d)), *d)
            self.client_socket.send(data_byte)
            
        except socket.error as e:
            rospy.logwarn(f"Socket connection error: {e}")
            self.client_socket = None
            self.sock = None

    def execute(self, goal):
        r = rospy.Rate(10)
        rospy.loginfo('Starting execution')
        feedback = check_distanceActionFeedback()
        result = check_distanceActionResult()

        while not rospy.is_shutdown():
            try:
                real_time_distance = self.sock.recv(4)
                distance = struct.unpack('f', real_time_distance)[0]
                rospy.loginfo('Received distance: %f', distance)
                self.pub.publish(distance)

                if self.actionserver.is_preempt_requested():
                    rospy.loginfo('%s: Preempted', self.actionserver.server_name)
                    self.actionserver.set_preempted()
                    break

                feedback.feedback_distance = distance
                self.actionserver.publish_feedback(feedback)
                r.sleep()

                if distance >= goal.cmd_distance:
                    result.distance = distance
                    self.actionserver.set_succeeded(result)
                    break

                # Send cmd_distance to the client
                data_bytes = struct.pack('!f', self.cmd_distance)
                self.client_socket.send(data_bytes)

            except socket.error as e:
                rospy.logwarn("Socket receive error: %s. Reconnecting...", e)
                self.setup_sockets()
                continue

            except KeyboardInterrupt:
                rospy.loginfo("Action server interrupted by user.")
                self.client_socket.close()
                self.sock.close()
                break

    def run(self):
        #self.actionserver.start()
        rospy.loginfo('Action server started')
        rospy.spin()


if __name__ == '__main__':
    host = "192.168.1.2"
    port_receive = 8080
    port_send = 9090

    server = RealTimeDistanceServer(host, port_receive, port_send)
    server.run()
