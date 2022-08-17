import threading

import mab.pyCandle as pyCandle
import time
import sys
import math
from threading import Thread, currentThread
import numpy as np
import skfuzzy as fuzz
from simple_pid import PID
import matplotlib.pyplot as plt


class MainModel(pyCandle.Candle):
    # working PID values 14.0 / 0.01 / 0.3
    # working PID for position 10.0 / 100/ 0.1
    def __init__(self, root):
        super().__init__(pyCandle.CAN_BAUD_1M, True)
        self.root = root
        self.motor_id = 101
        self.kp = 1
        self.temp_kp = self.kp
        self.ki = 100
        self.kd = 0.001
        self.windup = 0.0
        self.output_multiplier = 1.0
        self.open_position = -12.48
        self.controller_type = 'PID'
        self.control_mode = pyCandle.POSITION_PID
        # self.control_mode = pyCandle.IMPEDANCE
        self.desired_torque = 0.0
        self.grip_flag = False
        self.pid = PID(10, 1, 0.001, setpoint=0.0, output_limits=(-2.0, 2.0))
        self.prepare_fuzzy_rules()
        self.program_status = self.startup_procedure()
        if not self.program_status:
            self.end()

    def __del__(self):
        self.grip_flag = False
        self.controlMd80Enable(self.motor_id, False)
        self.end()

    def set_controller_type(self, controller):
        """
        Set controller type
        :param controller: Fuzzy or PID
        :return: None
        """
        if controller == 'Fuzzy' or 'PID':
            self.controller_type = controller

    def set_encoders_to_zero(self):
        return self.controlMd80SetEncoderZero(self.md80s[0])

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
                self.set_max_velocity(12.48)
                self.md80s[0].setPositionControllerParams(self.kp, self.ki, self.kd, self.windup)
                # self.md80s[0].setMaxTorque(0.20)

                #self.set_encoders_to_zero()

            self.begin()
            return True
        else:
            return False

    def close_gripper(self):
        # self.md80s[0].setTargetPosition(0.0)
        tmp_flag = False
        while True:
            if self.controller_type == 'Fuzzy':
                self.set_target_position(
                    self.get_position() + self.get_fuzzy_prediction(self.desired_torque - self.get_torque()))
            elif self.controller_type == 'PID':
                self.set_target_position(
                    self.get_position() + self.get_pid_prediction(self.get_torque()))
            if self.grip_flag:
                self.set_target_position(self.get_position())
                self.grip_flag = False
                break
            time.sleep(0.001)

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

    def set_target_velocity(self, velocity):
        self.md80s[0].setTargetVelocity(velocity)

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
        #self.md80s[0].setPositionControllerParams(kp, ki, kd, self.windup)
        self.pid.Kp = kp
        self.pid.Ki = ki
        self.pid.Kd = kd
        print(f"new PID gains, KP:{self.kp}, KI:{self.ki}, KD:{self.kd}")

    def set_output_multiplier(self, value):
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

    def get_fuzzy_prediction(self, error_input):

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

        position_out = fuzz.defuzz(self.position, R_combined, 'centroid')

        return sign * position_out * self.output_multiplier

    def get_pid_prediction(self, error_input):
        self.pid.setpoint = self.desired_torque
        output = self.pid(error_input)
        print(output)
        return output*1.0

    def isFloat(self, num):
        try:
            float(num)
            return True
        except ValueError:
            return False
