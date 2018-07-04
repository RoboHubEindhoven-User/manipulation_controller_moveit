#! /usr/bin/env python

import rospy
import serial
import glob
import serial.tools.list_ports
from manipulation_controller_moveit.srv import *
from sensor_msgs.msg import JointState as JointStateGripper

class Gripper(object):
    def __init__(self):
        rospy.init_node('gripper_service_server')
        
        led_service = rospy.Service('led_control', Gripperv6Led, self.control_led)

        self.mode_selector = rospy.get_param("~mode")
	    print self.mode_selector
        if (self.mode_selector == "real"):
	    print "Trying to connect to gripper arduino"
            gripper_service = rospy.Service('/gripper_control', Gripperv6, self.control_gripper)

            #Start Serial port
            self.ports = self.serial_ports()
            self.ser = serial.Serial(self.ports[0], 115200)

            # Start publisher
            self.joint_states_pub = rospy.Publisher('/arm_controller/follow_joint_trajectory', JointStateGripper, queue_size=10)
            print "Publisher started on /arm_controller/follow_joint_trajectory"
            
        elif (self.mode_selector == "simulated"):
            
            gripper_service = rospy.Service('/gripper_control', Gripperv6, self.publish_fake_joint_states)
            # Start publisher
            self.joint_states_pub = rospy.Publisher('/move_group/fake_controller_joint_states', JointStateGripper, queue_size=10)
            print "Publisher started on /move_group/fake_controller_joint_states"
            self.last_angle = 0

        print "Ready to execute task."

        rospy.spin()

    def serial_ports(self):

        # this excludes your current terminal "/dev/tty"
        ports = glob.glob('/dev/ttyACM[0-9]*')

        result = []
        for port in ports:
            print port
            try:
                ser = serial.Serial(port)
                ser.close()
                result.append(port)
            except (OSError, serial.SerialException):
                pass
        return result

        

    def control_gripper(self, request_gripper):
        print "Requesting for the gripper"
        print ("Size to open is equal to " + str(request_gripper.data) + " cm.")

        if request_gripper.data < 0:
            print "Too small!"
            request_gripper.data = 0
        elif request_gripper.data > 120:
            print "Too big!"
            request_gripper.data = 120

        try:
            self.ser.write(chr(request_gripper.data))
            self.ser.timeout = 0.5
            while True:
                back_dat = self.ser.read(size=2)
                if (ord(back_dat[0]) is 250):
                    break
                print ("Data from arduino " + str(ord(back_dat[0])))
                #print ("Data from arduino " + str(ord(back_dat[1])))
                self.publish_joint_states(ord(back_dat[1]))
            print "Sended: request_gripper.data =  ", request_gripper.data
            return 1
        except:
            print "Not sended because of unknown reason."
            return 0

    def control_led(self, request_led):
        print "Requesting for the LED"
        if request_led.LED_on is True:
            print "Turning LED on"
            dat = 200 & 0xFF
            if (self.mode_selector == "real"):
                self.ser.write(dat)
                print "Sended: request_gripper.data = "
            print request_led.LED_on
            return True
        elif (request_led.LED_on is False):
            print "Turning LED off"
            dat = 210 & 0xFF
            if (self.mode_selector == "real"):
                self.ser.write(dat)
                print "Sended: request_gripper.data = "
            print request_led.LED_on
            return True
        else:
            print "Not sended because of an unknown reason."
            return False

    def valmap(self, value, max_in, min_in):
        istart = min_in
        istop = max_in
        ostart = 0.0
        ostop = 1.985
        return ostart + (ostop - ostart) * ((value - istart) / (istop - istart))

    def publish_joint_states(self, servo_1_angle):
        # Construct message & publish joint states
        #servo_1_angle is a value between 50-170
        if (servo_1_angle > 170):
            print servo_1_angle
            print "Angle cant be more then 120 so distance cant be more then 185"
            return

        max_input = 170.0
        min_input = 50.0
        angle = self.valmap(servo_1_angle, max_input, min_input )
        print angle
        msg = JointStateGripper()
        msg.name = ['joint_left', 'Joint_finger_left', 'joint_right', 'joint_finger_right', 'joint_finger_two_right', 'joint_finger_two_left']
        msg.position = [-angle, angle, angle, -angle, angle, -angle]
        msg.velocity = [0,0,0,0,0,0]
        msg.effort = [0,0,0,0,0,0]   
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'base_link_gripper'
        self.joint_states_pub.publish(msg)

    def publish_fake_joint_states(self, angle_object):
        angle_servo = angle_object.data
        if (angle_servo > 120):
            print angle_servo
            print "Angle cant be more then 120 so distance cant be more then 185"
            return 0

        while (angle_servo != self.last_angle):
            if (angle_servo < self.last_angle):
                self.last_angle = self.last_angle - 1
            elif (angle_servo > self.last_angle):
                self.last_angle = self.last_angle + 1
                
            max_input = 120.0
            min_input = 0.0
            angle = self.valmap(self.last_angle, max_input, min_input )
            print angle
            msg = JointStateGripper()
            msg.name = ['joint_left', 'Joint_finger_left', 'joint_right', 'joint_finger_right', 'joint_finger_two_right', 'joint_finger_two_left']
            msg.position = [-angle, angle, angle, -angle, angle, -angle]
            msg.velocity = [0,0,0,0,0,0]
            msg.effort = [0,0,0,0,0,0]   
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = 'base_link_gripper'
            self.joint_states_pub.publish(msg)
            rospy.sleep(0.01)
        self.last_angle = angle_servo
        return 1

        


if __name__ == "__main__":
    s = Gripper()
