#!/usr/bin/env python

import dbus
import bluetooth
import threading
import rospy
from std_srvs.srv import Empty
from geometry_msgs.msg import TransformStamped
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import tf2_ros
import numpy as np

def proxyobj(bus, path, interface):
  """ commodity to apply an interface to a proxy object """
  obj = bus.get_object('org.bluez', path)
  return dbus.Interface(obj, interface)


def filter_by_interface(objects, interface_name):
  """ filters the objects based on their support
      for the specified interface """
  result = []
  for path in objects.keys():
    interfaces = objects[path]
    for interface in interfaces.keys():
      if interface == interface_name:
        result.append(path)
  return result


bus = dbus.SystemBus()

# we need a dbus object manager
manager = proxyobj(bus, "/", "org.freedesktop.DBus.ObjectManager")
objects = manager.GetManagedObjects()

# once we get the objects we have to pick the bluetooth devices.
# They support the org.bluez.Device1 interface
devices = filter_by_interface(objects, "org.bluez.Device1")

# now we are ready to get the informations we need
bt_devices = []
for device in devices:
  obj = proxyobj(bus, device, 'org.freedesktop.DBus.Properties')
  try:
    bt_devices.append({
        "alias": str(obj.Get("org.bluez.Device1", "Alias")),
        "addr": str(obj.Get("org.bluez.Device1", "Address"))
    })
  except:
    pass

sc_idx = 0
for i, device in enumerate(bt_devices):
  if device['alias'] == 'sea-cucumber':
    sc_idx = i
    break

print(bt_devices[sc_idx])
bd_addr = bt_devices[sc_idx]['addr']
port = 1
sock = bluetooth.BluetoothSocket( bluetooth.RFCOMM )
sock.connect((bd_addr, port))
print('connected')

start = False
lock = threading.Lock()
def handle_start(req):
  global start
  lock.acquire()
  start = True
  lock.release()

stop = False
def handle_stop(req):
  global stop
  lock.acquire()
  stop = True
  lock.release()

rospy.init_node('telemetry_client_node', anonymous=True)
rate = rospy.Rate(50)

start_server = rospy.Service('/telemetry_client_node/start', Empty, handle_start)
stop_server = rospy.Service('/telemetry_client_node/stop', Empty, handle_stop)

br = tf2_ros.TransformBroadcaster()
robot_tf = TransformStamped()
x = 0
y = 0
theta = 0

robot_tf.header.stamp = rospy.Time.now()
robot_tf.header.frame_id = 'world'
robot_tf.child_frame_id = 'shipbot'
robot_tf.transform.translation.x = 0
robot_tf.transform.translation.y = 0
robot_tf.transform.translation.z = 0
robot_tf.transform.rotation.x = 0
robot_tf.transform.rotation.y = 0
robot_tf.transform.rotation.z = 0
robot_tf.transform.rotation.w = 1
br.sendTransform(robot_tf)

guiderail_marker = Marker()
guiderail_marker.header.frame_id = 'world'
guiderail_marker.type = Marker.LINE_LIST
guiderail_marker.action = Marker.ADD
guiderail_marker.pose.position.x = 0
guiderail_marker.pose.position.y = 0
guiderail_marker.pose.position.z = 0
guiderail_marker.pose.orientation.w = 1
guiderail_marker.pose.orientation.x = 0
guiderail_marker.pose.orientation.y = 0
guiderail_marker.pose.orientation.z = 0
guiderail_marker.scale.x = 0.01
guiderail_marker.color.r = 0
guiderail_marker.color.g = 1
guiderail_marker.color.b = 0
guiderail_marker.color.a = 1
guiderail_marker.points = [Point(1.524, 0, 0), Point(0, 0, 0), Point(0, 0, 0), Point(0, 0.9144, 0)]
guiderail_pub = rospy.Publisher('/shipbot/guiderail', Marker, queue_size=1)

try:
  while not rospy.is_shutdown():
    if start:
      sock.send('<start>')
      start = False

    if stop:
      sock.send('<stop>')
      stop = False

    data = sock.recv(1024)
    if data:
      data = data.decode('utf-8')
      start_idx = data.find('<')
      end_idx = data.find('>')
      if start_idx != -1 and end_idx != -1:
        state = data[start_idx + 1:end_idx].split(' ')
        x = float(state[0])
        y = float(state[1])
        theta = float(state[2])

    guiderail_pub.publish(guiderail_marker)

    lock.acquire()
    robot_tf.transform.translation.x = x
    robot_tf.transform.translation.y = y
    robot_tf.transform.rotation.z = np.sin(theta/2)
    robot_tf.transform.rotation.w = np.cos(theta/2)
    lock.release()
    robot_tf.header.stamp = rospy.Time.now()
    br.sendTransform(robot_tf)

    rate.sleep()
except Exception as err:
  print(err)
finally:
  print('Closing socket')
  sock.close()
