import threading
import tkinter as tk
import matplotlib
import datetime as dt
import json
import time
import rospy

matplotlib.use('TkAgg')

from matplotlib.figure import Figure
import matplotlib.animation as animation
from matplotlib.backends.backend_tkagg import (
    FigureCanvasTkAgg,
    NavigationToolbar2Tk
)

import ttkbootstrap as ttk
from ttkbootstrap.constants import *
from threading import *
from PIL import ImageTk, Image
from model.main_model import MainModel


class MainWindowView:
    def __init__(self):
        self.root = ttk.Window(themename='cyborg', title='Gripper Gui')
        self.root.minsize(width=1200, height=800)
        self.model = MainModel(self.root)
        self.position = ttk.StringVar(value='0.0')
        self.torque = ttk.StringVar(value='0.0')

        self.torque_figure = Figure(figsize=(6, 4), dpi=100)
        self.position_figure = Figure(figsize=(6, 4), dpi=100)
        self.torque_axes = self.torque_figure.add_subplot(1, 1, 1)
        self.position_axes = self.position_figure.add_subplot(1, 1, 1)

        """
        *Code here*
        """
        self.kp = tk.StringVar(value=self.model.kp)
        self.ki = tk.StringVar(value=self.model.ki)
        self.kd = tk.StringVar(value=self.model.kd)

        self.create_layout()
        self.add_components()
        self.create_plot()
        self.torque_data = []
        self.torque_set_point_data = []
        self.date_time = []
        self.position_data = []
        self.position_set_point_data = []
        self.torque_animation = animation.FuncAnimation(self.torque_figure, self.animate,
                                                        fargs=(self.date_time, self.torque_data, self.torque_axes, 1),
                                                        interval=100)
        self.position_animation = animation.FuncAnimation(self.position_figure, self.animate, fargs=(
            self.date_time, self.position_data, self.position_axes, 2),
                                                          interval=100)

        """
        """
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        self.root.mainloop()

    def on_closing(self):
        import time
        self.save_data_to_file()
        self.model.end()
        print("Koniec")
        time.sleep(2)
        self.root.destroy()

    def save_data_to_file(self):
        data = {
            'torque': self.torque_data,
            'time': self.date_time
        }
        json_string = json.dumps(data)
        with open('torque_data.json', 'w') as outfile:
            json.dump(json_string, outfile)

    def animate(self, i, date_time, data, axes, data_type):
        if len(date_time) == len(data):
            date_time.append(rospy.get_time())

        if data_type == 1:
            data.append(self.model.get_torque())
            try:
                self.torque_set_point_data.append(float(self.torque.get()))
            except:
                self.torque_set_point_data.append(self.torque_set_point_data[-1])
            self.torque_set_point_data = self.torque_set_point_data[-20:]
            self.torque_label.configure(text=round(data[-1], 3))
        if data_type == 2:
            data.append(self.model.get_position())
            try:
                self.position_set_point_data.append(float(self.position.get()))
            except:
                self.position_set_point_data.append(self.position_set_point_data[-1])
            self.position_set_point_data = self.position_set_point_data[-20:]
            self.position_label.configure(text=round(data[-1], 3))

        date_time = date_time[-20:]
        data = data[-20:]
        axes.clear()

        if data_type == 1:
            axes.plot(date_time, data)
            axes.plot(date_time, self.torque_set_point_data)
        elif data_type == 2:
            axes.plot(date_time, data)
            axes.plot(date_time, self.position_set_point_data)

        if data_type == 2:
            axes.set_ylim([-1.5, 1.5])
            axes.set_ylabel("Position [Rad]")
        elif data_type == 1:
            axes.set_ylim([-0.6, 0.6])
            axes.set_ylabel("Torque [Nm]")

        axes.set_xlabel("Time [s]")

    def create_plot(self):
        # create FigureCanvasTkAgg object
        torque_canvas = FigureCanvasTkAgg(self.torque_figure, self.image_label_frame)
        # create the toolbar
        # NavigationToolbar2Tk(torque_canvas, self.image_label_frame)
        torque_canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=1)

        position_canvas = FigureCanvasTkAgg(self.position_figure, self.image_label_frame)
        # create the toolbar
        # NavigationToolbar2Tk(position_canvas, self.image_label_frame)
        position_canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=1)

    def set_position(self):
        self.model.set_target_position(float(self.position.get()))

    def set_torque(self):
        self.model.set_torque(float(self.torque.get()))

    def executor(self):
        t = Thread(name="Closer", target=self.model.close_gripper)
        t.start()

    def create_layout(self):
        """Packs basic frames to root frame"""
        self.left_frame = ttk.Frame(self.root, padding=(5, 5, 5, 5))
        self.center_frame = ttk.Frame(self.root, padding=(5, 5, 5, 5))
        self.right_frame = ttk.Frame(self.root, padding=(5, 5, 5, 5))

        self.left_frame.pack(side=LEFT, fill='both', expand=False)
        self.center_frame.pack(side=LEFT, fill='both', expand=True)
        self.right_frame.pack(side=RIGHT, fill='both', expand=False)

        self.menu_label_frame = ttk.LabelFrame(self.left_frame, text='Menu', padding=50)
        self.menu_label_frame.pack(fill='both')

        self.image_label_frame = ttk.LabelFrame(self.center_frame, text='Torque plots', padding=50)
        self.image_label_frame.pack(fill='both', expand=True)

    def change_pid_gains(self):
        self.model.change_gains(float(self.kp.get()),float(self.ki.get()),float(self.kd.get()))

    def add_components(self):
        ttk.Button(self.menu_label_frame,bootstyle='warning', text='Open', width=10, command=self.model.open_gripper).pack(pady=5)
        ttk.Button(self.menu_label_frame,bootstyle='success', text='Close', width=10, command=self.executor).pack(pady=5)
        ttk.Button(self.menu_label_frame,bootstyle='success', text='SetFlag', width=10, command=self.model.set_grip_flag).pack(pady=5)





        ttk.Label(self.menu_label_frame, text='Position: ').pack()
        ttk.Entry(self.menu_label_frame, textvariable=self.position, width=10).pack()
        ttk.Button(self.menu_label_frame, text='Move', width=10, command=self.set_position).pack(pady=5)

        ttk.Label(self.menu_label_frame, text='Torque: ').pack()
        ttk.Entry(self.menu_label_frame, textvariable=self.torque, width=10).pack()
        ttk.Button(self.menu_label_frame, text='Set Torque', width=10, command=self.set_torque).pack(pady=5)

        ttk.Button(self.menu_label_frame, text='Start record', width=10, command=self.set_position).pack(pady=5)
        ttk.Button(self.menu_label_frame, text='Stop record', width=10, command=self.set_position).pack(pady=5)

        ttk.Label(self.menu_label_frame, text='Current Position').pack()
        self.position_label = ttk.Label(self.menu_label_frame, text=0.0)
        self.position_label.pack()
        ttk.Label(self.menu_label_frame, text='Current Torque').pack()
        self.torque_label = ttk.Label(self.menu_label_frame, text=0.0)
        self.torque_label.pack()

        ttk.Button(self.menu_label_frame, text="test_pid", command=self.model.change_gains).pack(pady=5)

        ttk.Label(self.menu_label_frame, text='Kp: ').pack()
        ttk.Entry(self.menu_label_frame, textvariable=self.kp, width=10).pack()
        ttk.Label(self.menu_label_frame, text='Ki: ').pack()
        ttk.Entry(self.menu_label_frame, textvariable=self.ki, width=10).pack()
        ttk.Label(self.menu_label_frame, text='Kd: ').pack()
        ttk.Entry(self.menu_label_frame, textvariable=self.kd, width=10).pack()
        ttk.Button(self.menu_label_frame, text="Change gains", command=self.change_pid_gains).pack(pady=5)


