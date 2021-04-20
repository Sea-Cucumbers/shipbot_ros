#!/usr/bin/env python2

import serial
import rospy
from shipbot_ros.msg import ChassisFeedback

rospy.init_node('mcu_interface_node', anonymous=True)
mcu = serial.Serial('/dev/ttyACM0', 9600, timeout=1)

chassis_pub = rospy.Publisher('/shipbot/chassis_feedback', ChassisFeedback, queue_size=1)
chassis_msg = ChassisFeedback()
chassis_msg.tofs = [200, 200, 200, 200]

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

    rate.sleep()

  except Exception as e:
    print(e)
    mcu.close()
    break

