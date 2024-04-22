#!/usr/bin/env python
# license removed for brevity
import rospy
import serial  # 引用pySerial模組
from std_msgs.msg import String



def header():
  COM_PORT = '/dev/ttyACM0'    # 指定通訊埠名稱

  BAUD_RATES = 9600    # 設定傳輸速率
  ser = serial.Serial(COM_PORT, BAUD_RATES)   # 初始化序列通訊埠
  pub = rospy.Publisher('khatter', String, queue_size=10)
  rospy.init_node('header', anonymous=True)
  rate = rospy.Rate(10) # 10hz
  while not rospy.is_shutdown() :
    data_raw = ser.readline()  # 讀取一行
    data = data_raw.decode()   # 用預設的UTF-8解碼
    rospy.loginfo(data)
    pub.publish(data)
    rate.sleep()

if __name__ == '__main__':
    try:
        header()
    except rospy.ROSInterruptException:
        pass






