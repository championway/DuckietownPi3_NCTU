#!/usr/bin/env python
import rospy
import numpy as np
from duckietown_msgs.msg import RobotName, Twist2DStamped, BoolStamped
import sys
import math
from sensor_msgs.msg import Joy
import time
from __builtin__ import True
import socket
if "__main__" == __name__:
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM);
        print("create socket succ!")
        sock.bind(('', 50007))
        print("bind socket succ!")
        sock.listen(5)
        print("listen succ!")
    except:
        print("init socket err!")
    while True:
        print("listen for client...")
        conn, addr = sock.accept()
        print("get client")
        print(addr)
        conn.settimeout(5)
        szBuf = conn.recv(1024)
        print("recv:" + szBuf)
        if "0" == szBuf:
            conn.send('exit')
        else:
            conn.send('welcome client!')
        conn.close();
        print("end of sevice")
