#!/usr/bin/env python
import rospy
import time
import serial
from serial import SerialException
from threed_viz.msg import robotiq3fctrl
from threed_viz.msg import robotiq3f_feedback
import threading

class Robotiq3f_Serial_Node:
    def __init__(self):
        rospy.init_node('robotiq3f_serial')
        
        # 定义串口参数
        self.port = rospy.get_param('~port', '/dev/ttyUSB2')
        self.baudrate = rospy.get_param('~baudrate', 115200)
        self.timeout = rospy.get_param('~timeout', 1)
        # 串口初始化
        try:
            self.ser = serial.Serial(self.port, baudrate=self.baudrate, timeout=self.timeout)
            rospy.loginfo("robotiq串口已打开")
        except SerialException as e:
            rospy.logerr("无法打开robotiq串口: %s", e)

        # 发送的数据，这里是一个示例二进制数据
        self.request_data = bytearray([0x09,0x03,0x07,0xD2,0x00,0x04])
        self.ctrl_data=bytearray([0x09, 0x10, 0x03,0xE8,0x00,0x03,0x06,0x09,0x00,0x00,0x00,0x00,0x00])
        self.ctrl_data[10]=0  #postion
        self.ctrl_data[11]=0  #speed
        self.ctrl_data[12]=0  #current

        self.recv_request = b''
        # 构建Modbus RTU帧
        self.modbus_frame_ctrl = self.build_modbus_frame(self.ctrl_data)
        self.modbus_frame_req = self.build_modbus_frame(self.request_data)

        self.ctrl_command = 0;

        self.ctrl_sub = rospy.Subscriber('/robotiq3fctrl', robotiq3fctrl, self.ctrl_callback)
        self.fd_pub = rospy.Publisher("/robotiq3f_feedback", robotiq3f_feedback, queue_size=10)

        self.feedback_msg=robotiq3f_feedback()#反馈的消息变量
        self.ctrl_msg=robotiq3fctrl()#控制的消息变量

    def ctrl_callback(self, msg):#控制消息订阅回调函数

        self.ctrl_data[10]=msg.position  #postion
        self.ctrl_data[11]=msg.speed  #speed
        self.ctrl_data[12]=msg.force  #current

        self.modbus_frame_ctrl = self.build_modbus_frame(self.ctrl_data)

        self.ctrl_command = 1 ;
        
        # rospy.loginfo("订阅到消息: %d %d %d", msg.position,msg.speed,msg.force)

    def proc(self):
        rate = rospy.Rate(100)  # 10Hz
        while not rospy.is_shutdown():
            if self.ser.isOpen():
                try:
                    # start_time = time.perf_counter()# 循环开始计时

                    if self.ctrl_command :
                        self.ser.write(self.modbus_frame_ctrl)
                        # 接收数据
                        received_ctrl = self.ser.read(8)#ser.inWaiting()
                        if not received_ctrl:  # 如果received_req为空字节串
                            rospy.loginfo("ctrl没有接收到数据")
                        # print(f"ctrl接收到的数据:{received_ctrl.hex()}")
                        self.ctrl_command = 0

                    self.ser.write(self.modbus_frame_req)
                    # 接收数据
                    received_req = self.ser.read(13)#ser.inWaiting()
                    if not received_req:  # 如果received_req为空字节串
                        rospy.loginfo("req没有接收到数据")
                    else:
                        # print(f"req接收到的数据:{received_req.hex()}")
                        self.feedback_msg.A_position = received_req[3]
                        self.feedback_msg.A_current  = received_req[4]
                        self.feedback_msg.B_position = received_req[6]
                        self.feedback_msg.B_current  = received_req[7]
                        self.feedback_msg.C_position = received_req[9]
                        self.feedback_msg.C_current  = received_req[10]
                    
                    # 发布
                    self.fd_pub.publish(self.feedback_msg)
                    # rospy.loginfo("发布数据: %d", 123)

                    # end_time = time.perf_counter()
                    # elapsed_time = end_time - start_time
                    # print(f"robotiq循环耗时:{elapsed_time*1000.0:.3f} ms")

                    # rate.sleep()
                    
                except SerialException as e:
                    rospy.logerr("robotiq串口通信错误: %s", e)
            else:
                rospy.logwarn("robotiq串口未打开,尝试重新连接...")
                time.sleep(1)

    # 定义Modbus CRC计算和帧构建的函数
    def calculate_modbus_crc(self,data):
        crc = 0xFFFF
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if (crc & 0x0001):
                    crc >>= 1
                    crc ^= 0xA001
                else:
                    crc >>= 1
        return crc
    def build_modbus_frame(self,data):
        crc = self.calculate_modbus_crc(data)
        frame = data + crc.to_bytes(2, byteorder='little')
        return frame

if __name__ == '__main__':
    try:
        robotiq_node = Robotiq3f_Serial_Node()

        # 启动串口处理循环
        threading.Thread(target=robotiq_node.proc).start()
        # robotiq_node.proc()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        if 'ser' in locals() and robotiq_node.ser.isOpen():
            robotiq_node.ser.close()
            rospy.loginfo("robotiq串口已关闭")