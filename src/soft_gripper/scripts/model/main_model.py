import threading

import mab.pyCandle as pyCandle
import time
import sys
import math
from threading import Thread, currentThread
import numpy as np
import skfuzzy as fuzz
import matplotlib.pyplot as plt

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
        self.max_torque = 0.48
        self.torque_publisher = rospy.Publisher("torque", Float32, queue_size=10)
        self.startup_procedure()

        # self.set_target_position(0)
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

        # self.set_encoders_to_zero()

        self.begin()
        # self.set_max_velocity(0.2)

    def close_gripper(self):
        self.md80s[0].setTargetPosition(0.0)
        tmp_flag = False
        while True:
            if (-0.1 < self.get_position() < 0.1) or tmp_flag:
                break
            print("top")
            if self.max_torque < self.get_torque():
                #self.set_target_position(self.position_value(self.get_torque()))
                self.set_target_position(self.position_value(self.max_torque))


            # while 0.1>abs(self.get_torque() - self.max_torque) > 0.05:
            #     print("inside")
            #     if (-0.1 < self.get_position() < 0.1):
            #         break
            #     tmp_flag = True
            #     if self.max_torque > self.get_torque():
            #         self.set_target_position(self.get_position() + 0.1)
            #     if self.max_torque < self.get_torque():
            #         self.set_target_position(self.get_position() - 0.1)

    def open_gripper(self):
        self.md80s[0].setTargetPosition(-12.48)

    def set_max_velocity(self, max_velocity):
        self.md80s[0].setMaxVelocity(max_velocity)

    def set_target_position(self, position):
        self.md80s[0].setTargetPosition(position)

    def set_target_velocity(self, velocity):
        self.md80s[0].setTargetVelocity(velocity)

    def set_torque(self, torque):
        # self.md80s[0].setTorque(torque)
        self.max_torque = torque

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

    def position_value(self, value):
        # Generate universe variables
        torque = np.arange(0, 0.5, 0.05, dtype=float)
        position = np.arange(-6, 0, 0.5, dtype=float)

        # Generate fuzzy membership functions for input
        i_null = fuzz.trimf(torque, [0.0, 0.0, 0.0])
        i_zero = fuzz.trimf(torque, [0.0, 0.0, 0.05])
        i_small = fuzz.trimf(torque, [0.05, 0.1, 0.15])
        i_medium = fuzz.trimf(torque, [0.15, 0.2, 0.25])
        i_large = fuzz.trimf(torque, [0.25, 0.3, 0.35])
        i_very_large = fuzz.trimf(torque, [0.35, 0.4, 0.45])

        # Output membership function
        # o_null = fuzz.trimf(position, [-6, -6, -5.5])
        o_zero = fuzz.trimf(position, [-5.5, -5.0, -4.5])
        o_small = fuzz.trimf(position, [-4.5, -4.0, -3.5])
        o_medium = fuzz.trimf(position, [-3.5, -3.0, -2.5])
        o_large = fuzz.trimf(position, [-2.5, -2.0, -1.5])
        o_very_large = fuzz.trimf(position, [-1.5, -1.0, -0.5])

        # membership value calculation using the value
        torque_null = fuzz.interp_membership(torque, i_null, value)
        torque_level_zero = fuzz.interp_membership(torque, i_zero, value)
        torque_level_small = fuzz.interp_membership(torque, i_small, value)
        torque_level_medium = fuzz.interp_membership(torque, i_medium, value)
        torque_level_large = fuzz.interp_membership(torque, i_large, value)
        torque_level_very_large = fuzz.interp_membership(torque, i_very_large, value)

        active_rule1 = np.fmax(torque_null, torque_level_zero)
        position_activation_zero = np.fmin(active_rule1, o_zero)
        position_activation_small = np.fmin(torque_level_small, o_small)
        position_activation_medium = np.fmin(torque_level_medium, o_medium)
        position_activation_large = np.fmin(torque_level_large, o_large)
        position_activation_very_large = np.fmin(torque_level_very_large, o_very_large)

        position_dummy = np.zeros_like(position)
        aggregated = np.fmax(position_activation_zero,
                             np.fmax(position_activation_small, np.fmax(position_activation_medium,
                                                                        np.fmax(position_activation_large,
                                                                                position_activation_very_large))))

        # Calculate defuzzified result
        position_out = fuzz.defuzz(position, aggregated, 'centroid')
        print(position_out)
        position_activation = fuzz.interp_membership(position, aggregated, position_out)  # for plot

        # Visualize
        fig, (ax0, ax1, ax2, ax3) = plt.subplots(nrows=4, figsize=(8, 9))

        ax0.plot(torque, i_null, 'b', linewidth=1.5, )
        ax0.plot(torque, i_zero, 'r', linewidth=1.5, )
        ax0.plot(torque, i_small, 'b', linewidth=1.5, )
        ax0.plot(torque, i_medium, 'r', linewidth=1.5, )
        ax0.plot(torque, i_large, 'b', linewidth=1.5, )
        ax0.plot(torque, i_very_large, 'r', linewidth=1.5, )
        ax0.set_title('INPUT membership function (Volt)')
        ax0.legend()

        ax1.plot(position, o_zero, 'r', linewidth=1.5, )
        ax1.plot(position, o_small, 'b', linewidth=1.5, )
        ax1.plot(position, o_medium, 'r', linewidth=1.5, )
        ax1.plot(position, o_large, 'b', linewidth=1.5, )
        ax1.plot(position, o_very_large, 'r', linewidth=1.5, )
        ax1.set_title('OUTPUT membership function (RPM)')
        ax1.legend()

        ax2.fill_between(position, position_dummy, position_activation_zero, facecolor='b', alpha=0.7)
        ax2.plot(position, position_activation_zero, 'b', linewidth=0.5, linestyle='--', )
        ax2.fill_between(position, position_dummy, position_activation_small, facecolor='g', alpha=0.7)
        ax2.plot(position, position_activation_small, 'g', linewidth=0.5, linestyle='--')
        ax2.fill_between(position, position_dummy, position_activation_medium, facecolor='r', alpha=0.7)
        ax2.plot(position, position_activation_medium, 'r', linewidth=0.5, linestyle='--')
        ax2.fill_between(position, position_dummy, position_activation_large, facecolor='r', alpha=0.7)
        ax2.plot(position, position_activation_large, 'r', linewidth=0.5, linestyle='--')
        ax2.fill_between(position, position_dummy, position_activation_very_large, facecolor='r', alpha=0.7)
        ax2.plot(position, position_activation_very_large, 'r', linewidth=0.5, linestyle='--')
        ax2.set_title('Output membership activity')
        ax1.legend()

        ax3.plot(position, o_zero, 'b', linewidth=0.5, linestyle='--', )
        ax3.plot(position, o_small, 'g', linewidth=0.5, linestyle='--')
        ax3.plot(position, o_medium, 'r', linewidth=0.5, linestyle='--')
        ax3.plot(position, o_large, 'y', linewidth=0.5, linestyle='--', )
        ax3.plot(position, o_very_large, 'v', linewidth=0.5, linestyle='--')

        ax3.fill_between(position, position_dummy, aggregated, facecolor='Orange', alpha=0.7)
        ax3.plot([position_out, position_out], [0, position_activation], 'k', linewidth=1.5, alpha=0.9)
        ax3.set_title('Aggregated membership and result (line)')
        ax3.legend()

        # Turn off top/right axes
        for ax in (ax0, ax1, ax2, ax3):
            ax.spines['top'].set_visible(False)
            ax.spines['right'].set_visible(False)
            ax.get_xaxis().tick_bottom()
            ax.get_yaxis().tick_left()

        plt.tight_layout()

        # plt.savefig('output/out.png')
        # cv2.imwrite('output/output.png',(cv2.resize(cv2.imread('output/out.png')),(300,400)))

        return position_out
        # plt.show()