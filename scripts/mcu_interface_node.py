#!/usr/bin/env python2

import serial
import rospy
from shipbot_ros.msg import ChassisFeedback
from shipbot_ros.msg import ChassisCommand
from std_msgs.msg import Bool
import threading

lock = threading.Lock()
cmd_vx = 0
cmd_vy = 0
cmd_w = 0
grip = 0
# vx and vy should be in inches per second
def command_callback(msg):
  global cmd_vx
  global cmd_vy
  global cmd_w

  # Convert commands from m/s to inches per second. We
  # also need to switch around x and y because of 
  # inconsistency in frame definitions lol
  lock.acquire()
  cmd_vx = round(msg.vy/0.0254, 1)
  cmd_vy = round(-msg.vx/0.0254, 1)
  cmd_w = round(msg.w, 2)
  lock.release()

def grip_callback(msg):
  global grip

  lock.acquire()
  grip = int(msg.data)
  lock.release()

rospy.init_node('mcu_interface_node', anonymous=True)
mcu = serial.Serial('/dev/ttyACM0', 9600, timeout=1)

chassis_pub = rospy.Publisher('/shipbot/chassis_feedback', ChassisFeedback, queue_size=1)
chassis_msg = ChassisFeedback()
chassis_msg.tofs = [2, 2, 2, 2]

cmd_sub = rospy.Subscriber('/shipbot/chassis_command', ChassisCommand, command_callback)
grip_sub = rospy.Subscriber('/shipbot/grip', Bool, grip_callback)

# Each line of data should be "yaw dist0 dist1 dist2 dist3\n"
rospy.sleep(1)

rate = rospy.Rate(50) # 50 Hz
while not rospy.is_shutdown():
  try:
    data = mcu.readline()
    if data:
      data = data.decode('utf-8').split()
      if len(data) == 5:
        data = [float(d) for d in data]
        chassis_msg.header.stamp = rospy.Time.now()
        chassis_msg.yaw = data[0]

         # Convert tof values from cm to meters
        chassis_msg.tofs[0] = data[1]/100
        chassis_msg.tofs[1] = data[2]/100
        chassis_msg.tofs[2] = data[3]/100
        chassis_msg.tofs[3] = data[4]/100 - 0.023
        chassis_pub.publish(chassis_msg)
      else:
        print('Data line should contain 5 elements, and it does not')
    else:
      print('No data')

    lock.acquire()
    mcu.write('<' + str(cmd_vx) + ' ' + str(cmd_vy) + ' ' + str(cmd_w) + ' ' + str(grip) + '>')
    lock.release()

    rate.sleep()

  except Exception as e:
    print(e)
    mcu.close()
    mcu = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
    rospy.sleep(1)

