#!/usr/bin/env python3

import bluetooth
import rospy
from shipbot_ros.msg import ChassisState
from std_srvs.srv import Empty
import threading

rospy.init_node('telemetry_server_node', anonymous=True)

# Initialize bluetooth
port = 1
backlog = 1
size = 1024
s = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
s.bind(('', port))
s.listen(backlog)

x = 0
y = 0
theta = 0
lock = threading.Lock()

def state_callback(msg):
  global x
  global y
  global theta

  lock.acquire()
  x = msg.x
  y = msg.y
  theta = msg.yaw
  lock.release()

state_sub = rospy.Subscriber('/shipbot/chassis_state', ChassisState, state_callback)

start_mission_client = rospy.ServiceProxy('/mission_control_node/start_mission', Empty)
stop_mission_client = rospy.ServiceProxy('/mission_control_node/stop_mission', Empty)
data = None

rate = rospy.Rate(50)
try:
  client, clientInfo = s.accept()
  client.setblocking(False)

  while not rospy.is_shutdown():
    try: 
      data = client.recv(size)
    except:
      data = None

    if data:
      data = data.decode('utf-8')
      if data == '<start>':
        start_mission_client()
      if data == '<stop>':
        stop_mission_client()
      print(data)

    lock.acquire()
    # Currently we just send robot state over bluetooth
    client.send('<' + str(x) + ' ' + str(y) + ' ' + str(theta) + '>')
    lock.release()
    rate.sleep()
except Exception as err: 
  print(err)
finally:
  print("Closing socket")
  client.close()
  s.close()
