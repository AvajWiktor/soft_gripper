import threading

import mab.pyCandle as pyCandle
import time
import sys
import math
from threading import Thread, currentThread
from std_msgs.msg import Float32

import rospy


class MainModel(pyCandle.Candle):
    # working PID values 14.0 / 0.01 / 0.3
    def __init__(self, root):
        super().__init__(pyCandle.CAN_BAUD_1M, True)
        self.root = root
        self.motor_id = 101
        self.kp = 10.0
        self.temp_kp = self.kp
        self.ki = 100.0
        self.kd = 0.1
        self.windup = 0.0
        self.control_mode = pyCandle.POSITION_PID

        self.torque_publisher = rospy.Publisher("torque", Float32, queue_size=10)
        self.startup_procedure()

        #self.set_target_position(0)
        self.start_publish_thread()
        print("Model created!")

    def update(self):
        while threading.main_thread().is_alive():
            self.torque_publisher.publish(-self.md80s[0].getTorque())
            rospy.sleep(1)

    def start_publish_thread(self):
        updater = Thread(name="torque_updater", target=self.update)
        updater.start()

    def set_encoders_to_zero(self):
        return self.controlMd80SetEncoderZero(self.md80s[0])

    def startup_procedure(self):
        self.addMd80(self.motor_id)
        self.controlMd80Mode(self.motor_id, self.control_mode)
        self.controlMd80Enable(self.motor_id, True)
        self.md80s[0].setPositionControllerParams(self.kp, self.ki, self.kd, self.windup)
        self.md80s[0].setMaxTorque(0.1)
        self.set_max_velocity(10.2)

        #self.set_encoders_to_zero()

        self.begin()
        #self.set_max_velocity(0.2)
    def set_max_velocity(self, max_velocity):
        self.md80s[0].setMaxVelocity(max_velocity)

    def set_target_position(self, position):
        self.md80s[0].setTargetPosition(position)

    def set_target_velocity(self, velocity):
        self.md80s[0].setTargetVelocity(velocity)

    def set_torque(self, torque):
        self.md80s[0].setTorque(torque)

    def get_torque(self):
        return self.md80s[0].getTorque()

    def get_position(self):
        return self.md80s[0].getPosition()

    def change_gains(self, kp, ki, kd):
        self.md80s[0].setPositionControllerParams(kp, ki, kd, self.windup)


    def change_pid(self):
        self.temp_kp -= 1.0
        print(self.temp_kp)
        self.md80s[0].setPositionControllerParams(self.temp_kp, self.ki, self.kd, self.windup)

