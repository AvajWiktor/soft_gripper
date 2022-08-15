import threading

import mab.pyCandle as pyCandle
import time
import sys
import math
from threading import Thread, currentThread
import numpy as np
import skfuzzy as fuzz
import serial


class MainModel(pyCandle.Candle):
    # working PID values 14.0 / 0.01 / 0.3
    # working PID for position 10.0 / 100/ 0.1
    def __init__(self):
        super().__init__(pyCandle.CAN_BAUD_1M, True)
        self.motor_id = 101
        self.kp = 1.0
        self.temp_kp = self.kp
        self.ki = 100
        self.kd = 0.001
        self.windup = 0.0
        self.main_thread = Thread(name="MainLoopThread", target=self.main_loop)
        self.output_multiplier = 1.0
        self.open_position = -12.48
        self.control_mode = pyCandle.POSITION_PID
        # self.control_mode = pyCandle.IMPEDANCE
        self.desired_torque = 0.0
        self.grip_flag = False
        self.prepare_fuzzy_rules()
        self.program_status = self.startup_procedure()
        if not self.program_status:
            self.end()
        self.main_thread.start()
        self.main_thread.join()

    def main_loop(self):
        ser = serial.Serial(
            port='/dev/ttyUSB0',
            baudrate=9600,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=1
        )
        while self.handle_serial_input(ser.readline().decode()):
            try:
                time.sleep(0.1)
            except KeyboardInterrupt:
                break

    def set_encoders_to_zero(self):
        return self.controlMd80SetEncoderZero(self.md80s[0])

    def handle_serial_input(self, input):
        print(f"torq: {self.get_torque()}")
        if len(input)>0:
            try:
                if input[0] == 'x':
                    return False
                elif input[:2] == "st":
                    self.set_torque(float(input[2:]))
                elif input[:2] == "cl":
                    self.grip_flag = True
                    time.sleep(0.1)
                    self.grip_flag = False
                    t = Thread(name="Closer", target=self.close_gripper)
                    t.start()
                elif input[:2] == "op":
                    self.open_gripper()
                elif input[:2] == "so":
                    if self.isFloat(input[2:]):
                        self.open_position = float(input[2:])
                elif input[:2] == "ss":
                    if self.isFloat(input[2:]):
                        self.output_multiplier = float(input[2:])
            except Exception as e:
                print(f"Exception occurred: {e}")
        return True

    def set_grip_flag(self):
        if self.grip_flag:
            self.grip_flag = False
        else:
            self.grip_flag = True

    def startup_procedure(self):
        # self.addMd80(self.motor_id, True)
        if self.ping():
            self.addMd80(self.motor_id)
            self.controlMd80Mode(self.motor_id, self.control_mode)
            self.controlMd80Enable(self.motor_id, True)
            if self.control_mode == pyCandle.IMPEDANCE:
                self.md80s[0].setImpedanceControllerParams(self.kp, self.kd)
            elif self.control_mode == pyCandle.POSITION_PID:
                self.md80s[0].setPositionControllerParams(self.kp, self.ki, self.kd, self.windup)
                self.md80s[0].setMaxTorque(0.45)
                self.set_max_velocity(12.48)

                # self.set_encoders_to_zero()

            self.begin()
            return True
        else:
            return False

    def close_gripper(self):
        # self.md80s[0].setTargetPosition(0.0)
        tmp_flag = False
        while True:
            if (-0.1 < self.get_position() < 0.1) or tmp_flag:
                break
            # self.set_target_position(self.position_value(self.get_torque()))
            self.set_target_position(
                self.get_position() + self.calculate_position(self.desired_torque - self.get_torque()))
            time.sleep(0.001)
            # if (self.desired_torque + 0.03) >= self.get_torque() >= self.desired_torque - 0.03:
            if self.grip_flag:
                self.set_target_position(self.get_position())
                self.grip_flag = False
                break

    def open_gripper(self):
        self.grip_flag = True
        time.sleep(0.1)
        self.md80s[0].setTargetPosition(self.open_position)
        time.sleep(0.1)
        self.grip_flag = False

    def set_max_velocity(self, max_velocity):
        if self.isFloat(max_velocity):
            self.md80s[0].setMaxVelocity(max_velocity)

    def set_target_position(self, position):
        if self.isFloat(position):
            self.md80s[0].setTargetPosition(position)

    def set_torque(self, torque):
        if self.isFloat(torque):
            if 0.5 > torque >= 0.0:
                self.desired_torque = torque
                return True
            else:
                return False

    def get_torque(self):
        return self.md80s[0].getTorque()

    def get_position(self):
        return self.md80s[0].getPosition()

    def change_gains(self, kp, ki, kd):
        self.md80s[0].setPositionControllerParams(kp, ki, kd, self.windup)
        print(f"new PID gains, KP:{self.kp}, KI:{self.ki}, KD:{self.kd}")

    def set_output_multiplier(self, value):
        if self.isFloat(value):
            self.output_multiplier = value

    def prepare_fuzzy_rules(self):
        self.torque_error = np.arange(0.0, 0.6, 0.02, dtype=float)
        self.position = np.arange(0, 3.0, 0.1, dtype=float)

        self.te_zero = fuzz.trapmf(self.torque_error, [0.0, 0.0, 0.0, 0.05])
        self.te_small = fuzz.trapmf(self.torque_error, [0.05, 0.1, 0.1, 0.15])
        self.te_medium = fuzz.trapmf(self.torque_error, [0.15, 0.25, 0.25, 0.3])
        self.te_large = fuzz.trapmf(self.torque_error, [0.25, 0.3, 0.35, 0.4])
        self.te_very_large = fuzz.trapmf(self.torque_error, [0.35, 0.40, 0.45, 0.5])
        self.te_edge = fuzz.trapmf(self.torque_error, [0.45, 0.5, 0.5, 0.55])

        self.p_edge = fuzz.trapmf(self.position, [2.0, 2.5, 2.5, 3.0])
        self.p_very_large = fuzz.trapmf(self.position, [1.6, 1.8, 1.8, 2.0])
        self.p_large = fuzz.trapmf(self.position, [1.2, 1.4, 1.4, 1.6])
        self.p_medium = fuzz.trapmf(self.position, [0.8, 1.0, 1.0, 1.2])
        self.p_small = fuzz.trapmf(self.position, [0.4, 0.6, 0.6, 0.8])
        self.p_zero = fuzz.trapmf(self.position, [0.0, 0.0, 0.2, 0.4])

    def calculate_position(self, error_input):

        error = abs(error_input)
        # print(f"Error: {error}")
        sign = -1 if error_input < 0 else 1

        R1 = fuzz.interp_membership(self.torque_error, self.te_edge, error)
        R2 = fuzz.interp_membership(self.torque_error, self.te_very_large, error)
        R3 = fuzz.interp_membership(self.torque_error, self.te_large, error)
        R4 = fuzz.interp_membership(self.torque_error, self.te_medium, error)
        R5 = fuzz.interp_membership(self.torque_error, self.te_small, error)
        R6 = fuzz.interp_membership(self.torque_error, self.te_zero, error)

        PA1 = np.fmin(R1, self.p_edge)
        PA2 = np.fmin(R2, self.p_very_large)
        PA3 = np.fmin(R3, self.p_large)
        PA4 = np.fmin(R4, self.p_medium)
        PA5 = np.fmin(R5, self.p_small)
        PA6 = np.fmin(R6, self.p_zero)

        R_combined = np.fmax(PA1, np.fmax(PA2, np.fmax(PA3, np.fmax(PA4, np.fmax(PA5, PA6)))))

        # predicted_position = np.zeros_like(torque_error)
        # for i in range(len(predicted_position)):
        #    print(i)
        #    predicted_position[i] = fuzz.defuzz(position, R_combined[i, :], 'centroid')
        # print(predicted_position)

        # print(R_combined)
        position_out = fuzz.defuzz(self.position, R_combined, 'centroid')

        # print(sign * position_out / 10.0)
        return sign * position_out * self.output_multiplier

    def position_value(self, value):
        """Legacy, do not use"""
        print(f"Torque error: {value}")
        # Generate universe variables
        torque = np.arange(-0.55, 0.55, 0.05, dtype=float)
        position = np.arange(-1.1, 1.1, 0.1, dtype=float)
        # Generate fuzzy membership functions for input
        i_neg_very_large = fuzz.trimf(torque, [-0.5, -0.45, -0.4])
        i_neg_large = fuzz.trimf(torque, [-0.4, -0.35, -0.35])
        i_negative_medium = fuzz.trimf(torque, [-0.35, -0.3, -0.25])
        i_negative_small = fuzz.trimf(torque, [-0.25, -0.20, -0.15])
        i_negative_very_small = fuzz.trimf(torque, [-0.15, -0.10, -0.05])
        i_zero = fuzz.trimf(torque, [-0.05, 0.0, 0.05])
        i_positive_very_small = fuzz.trimf(torque, [0.05, 0.1, 0.15])
        i_positive_small = fuzz.trimf(torque, [0.15, 0.20, 0.25])
        i_positive_medium = fuzz.trimf(torque, [0.25, 0.3, 0.35])
        i_positive_large = fuzz.trimf(torque, [0.35, 0.35, 0.40])
        i_positive_very_large = fuzz.trimf(torque, [0.40, 0.45, 0.50])

        # Output membership function

        o_negative_very_large = fuzz.trimf(position, [-1.0, -0.9, -0.8])
        o_negative_large = fuzz.trimf(position, [-0.8, -0.7, -0.7])
        o_negative_medium = fuzz.trimf(position, [-0.7, -0.6, -0.5])
        o_negative_small = fuzz.trimf(position, [-0.5, -0.4, -0.3])
        o_negative_very_small = fuzz.trimf(position, [-0.3, -0.2, -0.1])
        o_zero = fuzz.trimf(position, [-0.1, 0.0, 0.1])
        o_positive_very_small = fuzz.trimf(position, [0.1, 0.2, 0.3])
        o_positive_small = fuzz.trimf(position, [0.3, 0.4, 0.5])
        o_positive_medium = fuzz.trimf(position, [0.5, 0.6, 0.7])
        o_positive_large = fuzz.trimf(position, [0.7, 0.7, 0.8])
        o_positive_very_large = fuzz.trimf(position, [0.8, 0.9, 1.0])

        # membership value calculation using the value
        torque_level_negative_very_large = fuzz.interp_membership(torque, i_neg_very_large, value)
        torque_level_negative_large = fuzz.interp_membership(torque, i_neg_large, value)
        torque_level_negative_medium = fuzz.interp_membership(torque, i_negative_medium, value)
        torque_level_negative_small = fuzz.interp_membership(torque, i_negative_small, value)
        torque_level_negative_very_small = fuzz.interp_membership(torque, i_negative_very_small, value)
        torque_level_zero = fuzz.interp_membership(torque, i_zero, value)
        torque_level_positive_very_small = fuzz.interp_membership(torque, i_positive_very_small, value)
        torque_level_positive_small = fuzz.interp_membership(torque, i_positive_small, value)
        torque_level_positive_medium = fuzz.interp_membership(torque, i_positive_medium, value)
        torque_level_positive_large = fuzz.interp_membership(torque, i_positive_large, value)
        torque_level_positive_very_large = fuzz.interp_membership(torque, i_positive_very_large, value)

        position_activation_negative_very_large = np.fmin(torque_level_negative_very_large, o_negative_very_large)
        position_activation_negative_large = np.fmin(torque_level_negative_large, o_negative_large)
        position_activation_negative_medium = np.fmin(torque_level_negative_medium, o_negative_medium)
        position_activation_negative_small = np.fmin(torque_level_negative_small, o_negative_small)
        position_activation_negative_very_small = np.fmin(torque_level_negative_very_small, o_negative_very_small)
        position_activation_zero = np.fmin(torque_level_zero, o_zero)
        position_activation_positive_very_small = np.fmin(torque_level_positive_very_small, o_positive_very_small)
        position_activation_positive_small = np.fmin(torque_level_positive_small, o_positive_small)
        position_activation_positive_medium = np.fmin(torque_level_positive_medium, o_positive_medium)
        position_activation_positive_large = np.fmin(torque_level_positive_large, o_positive_large)
        position_activation_positive_very_large = np.fmin(torque_level_positive_very_large, o_positive_very_large)

        # position_dummy = np.zeros_like(position)
        aggregated = np.fmax(position_activation_negative_very_large,
                             np.fmax(position_activation_negative_large, np.fmax(position_activation_negative_medium,
                                                                                 np.fmax(
                                                                                     position_activation_negative_small,
                                                                                     np.fmax(
                                                                                         position_activation_negative_very_small,
                                                                                         np.fmax(
                                                                                             position_activation_zero,
                                                                                             np.fmax(
                                                                                                 position_activation_positive_very_small,
                                                                                                 np.fmax(
                                                                                                     position_activation_positive_small,
                                                                                                     np.fmax(
                                                                                                         position_activation_positive_medium,
                                                                                                         np.fmax(
                                                                                                             position_activation_positive_large,
                                                                                                             position_activation_positive_very_large))))))))))

        # Calculate defuzzified result
        position_out = fuzz.defuzz(position, aggregated, 'centroid')
        # print(f"Position error: {position_out}")

        # position_activation = fuzz.interp_membership(position, aggregated, position_out)  # for plot

        # Visualize
        # fig, (ax0, ax1, ax2, ax3) = plt.subplots(nrows=4, figsize=(8, 9))
        #
        # ax0.plot(torque, i_null, 'b', linewidth=1.5, )
        # ax0.plot(torque, i_zero, 'r', linewidth=1.5, )
        # ax0.plot(torque, i_small, 'b', linewidth=1.5, )
        # ax0.plot(torque, i_medium, 'r', linewidth=1.5, )
        # ax0.plot(torque, i_large, 'b', linewidth=1.5, )
        # ax0.plot(torque, i_very_large, 'r', linewidth=1.5, )
        # ax0.set_title('INPUT membership function (Volt)')
        # ax0.legend()
        #
        # ax1.plot(position, o_zero, 'r', linewidth=1.5, )
        # ax1.plot(position, o_small, 'b', linewidth=1.5, )
        # ax1.plot(position, o_medium, 'r', linewidth=1.5, )
        # ax1.plot(position, o_large, 'b', linewidth=1.5, )
        # ax1.plot(position, o_very_large, 'r', linewidth=1.5, )
        # ax1.set_title('OUTPUT membership function (RPM)')
        # ax1.legend()
        #
        # ax2.fill_between(position, position_dummy, position_activation_zero, facecolor='b', alpha=0.7)
        # ax2.plot(position, position_activation_zero, 'b', linewidth=0.5, linestyle='--', )
        # ax2.fill_between(position, position_dummy, position_activation_small, facecolor='g', alpha=0.7)
        # ax2.plot(position, position_activation_small, 'g', linewidth=0.5, linestyle='--')
        # ax2.fill_between(position, position_dummy, position_activation_medium, facecolor='r', alpha=0.7)
        # ax2.plot(position, position_activation_medium, 'r', linewidth=0.5, linestyle='--')
        # ax2.fill_between(position, position_dummy, position_activation_large, facecolor='r', alpha=0.7)
        # ax2.plot(position, position_activation_large, 'r', linewidth=0.5, linestyle='--')
        # ax2.fill_between(position, position_dummy, position_activation_very_large, facecolor='r', alpha=0.7)
        # ax2.plot(position, position_activation_very_large, 'r', linewidth=0.5, linestyle='--')
        # ax2.set_title('Output membership activity')
        # ax1.legend()
        #
        # ax3.plot(position, o_zero, 'b', linewidth=0.5, linestyle='--', )
        # ax3.plot(position, o_small, 'g', linewidth=0.5, linestyle='--')
        # ax3.plot(position, o_medium, 'r', linewidth=0.5, linestyle='--')
        # ax3.plot(position, o_large, 'y', linewidth=0.5, linestyle='--', )
        # ax3.plot(position, o_very_large, 'v', linewidth=0.5, linestyle='--')
        #
        # ax3.fill_between(position, position_dummy, aggregated, facecolor='Orange', alpha=0.7)
        # ax3.plot([position_out, position_out], [0, position_activation], 'k', linewidth=1.5, alpha=0.9)
        # ax3.set_title('Aggregated membership and result (line)')
        # ax3.legend()
        #
        # # Turn off top/right axes
        # for ax in (ax0, ax1, ax2, ax3):
        #     ax.spines['top'].set_visible(False)
        #     ax.spines['right'].set_visible(False)
        #     ax.get_xaxis().tick_bottom()
        #     ax.get_yaxis().tick_left()

        # plt.tight_layout()

        # plt.savefig('output/out.png')
        # cv2.imwrite('output/output.png',(cv2.resize(cv2.imread('output/out.png')),(300,400)))

        return position_out / 5.0
        # plt.show()

    def isFloat(self, num):
        try:
            float(num)
            return True
        except ValueError:
            return False