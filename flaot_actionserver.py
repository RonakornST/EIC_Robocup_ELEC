#!/usr/bin/env python

import socket
import struct

import rospy
import actionlib
from my_package.msg import check_distanceAction, check_distanceActionResult, check_distanceActionFeedback
from std_msgs.msg import Float32

# rospy.get_name() = blahblah from rospy.init_node('blahblah')
class RealTimeDistanceServer:
    def __init__(self, host, port_receive, port_send):
        rospy.init_node('real_time_distance_server')
        self.actionserver = actionlib.SimpleActionServer('real_time_distance_action', check_distanceAction, execute_cb = self.execute, auto_start = False)
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
        print('create socket')
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client_socket.connect((self.host, self.port_send))
        self.sock = socket.socket()
        self.sock.connect((self.host, self.port_receive))
        print('port connect')

    def execute(self, goal):
        r = rospy.Rate(10)
        print('in exe')
        feedback = check_distanceActionFeedback()
        result = check_distanceActionResult()

        # Set the cmd_distance received from the goal
        #self.cmd_distance = goal.cmd_distance
        success = True
        message = "Hello World".encode()
        self.sock.send(message)
        d = [0, 0]
        data_byte = struct.pack('!{}f'.format(len(d)), *d)
        self.client_socket.send(data_byte)
        print('send req')
        while not rospy.is_shutdown():
            
            try:
                real_time_distance = self.sock.recv(4)
                distance = struct.unpack('f', real_time_distance)[0]
                print('distance',distance)
                self.pub.publish(distance)

                if self._as.is_preempt_requested():
                       rospy.loginfo('%s: Preempted' % self._action_name)
                       self._as.set_preempted()
                       success = False
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

            except (socket.error, ConnectionResetError):
                rospy.logwarn("Connection lost. Reconnecting...")
                self.setup_sockets()
                continue

            except KeyboardInterrupt:
                rospy.loginfo("Action server interrupted by user.")
                self.client_socket.close()
                self.sock.close()
                break

    def run(self):
    
        rospy.spin()


if __name__ == '__main__':
    host = "192.168.1.2"
    port_receive = 8080
    port_send = 9090

    server = RealTimeDistanceServer(host, port_receive, port_send)
    server.run()
