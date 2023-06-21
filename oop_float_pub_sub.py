#!/usr/bin/env python
import socket
import struct
from time import sleep

import rospy
from std_msgs.msg import Float32


class RealTimeDistanceSender:
    def __init__(self, host, port_receive, port_send):
        rospy.init_node('listener', anonymous=True)
        self.pub = rospy.Publisher('real_time_distance', Float32, queue_size=10)
        self.cmd_distance = [-1.0]
        self.host = host
        self.port_receive = port_receive
        self.port_send = port_send
        self.sock = socket.socket()
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.setup_sockets()

    def setup_sockets(self):
        self.client_socket.connect((self.host, self.port_send))
        self.sock.connect((self.host, self.port_receive))

    def callback(self, data: Float32):
        self.cmd_distance = [data.data]

    def send_real_time_distance(self):
        array_size = 4
        data = bytearray()
        i = 0

        message = "Hello World".encode()
        self.sock.send(message)
        d = [0, 0]
        data_byte = struct.pack('!{}f'.format(len(d)), *d)
        self.client_socket.send(data_byte)

        while True:
            try:
                real_time_distance = self.sock.recv(array_size)
                data.extend(real_time_distance)

                if len(data) >= array_size:
                    receive_value = struct.unpack('f', data[:array_size])[0]
                    print("Receive:", receive_value, i)
                    self.pub.publish(receive_value)
                    data = data[array_size:]

                cmd_distance_float = self.cmd_distance[0]
                data_bytes = struct.pack('!f', cmd_distance_float)
                self.client_socket.send(data_bytes)
                print("Sent:", cmd_distance_float, i)

                i += 1
                sleep(0.01)
                
            except (socket.error, ConnectionResetError):
                print("Connection lost. Reconnecting...")
                sleep(1)
                self.setup_sockets()
                continue

            except KeyboardInterrupt:
                self.client_socket.close()
                self.sock.close()
                break

    def run(self):
        rospy.Subscriber("cmd_distance_data", Float32, self.callback)
        while not rospy.is_shutdown():
            self.send_real_time_distance()
            sleep(0.01)
        rospy.spin()


if __name__ == '__main__':
    host = "192.168.1.2"
    port_receive = 8080
    port_send = 9090

    sender = RealTimeDistanceSender(host, port_receive, port_send)
    sender.run()
