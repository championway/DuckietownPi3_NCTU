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
con = False 
if "__main__" == __name__:
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        print("create socket succ!")
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(('', 8001))
        print("bind socket succ!")
        sock.listen(5)
        print("listen succ!")
    except:
        print("init socket err!")
    while True:
        if con==False
            print("listen for client...")
            conn, addr = sock.accept()
            print("get client")
            #print(addr)
        #conn.setblocking(1)
        #conn.send('d')
        #conn.settimeout(5)
        szBuf = conn.recv(1024)
        print("recv:" + szBuf)
        try:
        if szBuf == "0":
            #conn.send("exit")
            print("disconnect")
            conn.close()
        else:
            print("receive mess")
            #conn.send("welcome client!'")'''
        #except socket.timeout:
            #continue
        #conn.close();
        #print("end of sevice")
