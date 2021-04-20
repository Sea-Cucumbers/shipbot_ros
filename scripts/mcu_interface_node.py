#!/usr/bin/env python2

import serial
import rospy
from shipbot_ros.msg import ChassisFeedback
from shipbot_ros.msg import ChassisCommand

cmd_vx = 0
cmd_vy = 0
cmd_w = 0
def command_callback(msg):
  global cmd_vx
  global cmd_vy
  global cmd_w

  cmd_vx = msg.vx
  cmd_vy = msg.vy
  cmd_w = msg.w

rospy.init_node('mcu_interface_node', anonymous=True)
mcu = serial.Serial('/dev/ttyACM0', 9600, timeout=1)

chassis_pub = rospy.Publisher('/shipbot/chassis_feedback', ChassisFeedback, queue_size=1)
chassis_msg = ChassisFeedback()
chassis_msg.tofs = [200, 200, 200, 200]

cmd_sub = rospy.Subscriber('/shipbot/chassis_command', ChassisCommand, command_callback)

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
        chassis_msg.tofs[0] = data[1]
        chassis_msg.tofs[1] = data[2]
        chassis_msg.tofs[2] = data[3]
        chassis_msg.tofs[3] = data[4]
        chassis_pub.publish(chassis_msg)
      else:
        print('Data line should contain 5 elements, and it does not')
    else:
      print('No data')

    mcu.write(str(cmd_vx) + ' ' + str(cmd_vy) + ' ' + str(cmd_w) + '\n')

    rate.sleep()

  except Exception as e:
    print(e)
    mcu.close()
    break

