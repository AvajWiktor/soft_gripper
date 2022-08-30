import matplotlib.pyplot as plt
import json
filename = 'torque_data'
#filename = 'miekka'

with open(filename+'.json', 'r') as d:
	dictionary = json.loads(json.load(d))
	print(dictionary)
time = dictionary["time"]
torque = dictionary["torque"]
desired_torque = dictionary['desired_torque']
position = dictionary['position']
if len(position) != len(time):
	position.append(position[-1])
controller_type = dictionary['controller_type']
kp = dictionary['kp']
ki = dictionary['ki']
kd = dictionary['kd']

# plt.grid(True)
# plt.title("PID H H H")
# ## LINE GRAPH ##
# plt.plot(time, torque, color='maroon', label='Torque')
# plt.plot(time, desired_torque, label='Desired Torque')
# plt.plot(time, position, label="Position")
#
# plt.xlabel('Time [s]')
# plt.ylabel('Torque [Nm]')
#
# plt.legend()
# plt.show()

fig, (ax1, ax2) = plt.subplots(nrows=2, figsize=(15, 9))

plt.rcParams["figure.figsize"] = (14,5.5)
title = controller_type
if controller_type == 'PID':
	title += f' Kp:{kp} Ki:{ki} Kd:{kd}'
ax1.set_title('Torque - ' + title)
ax1.plot(time, torque, label='Torque')
ax1.plot(time, desired_torque, label='Desired Torque')
ax1.set_xlabel('Time [s]')
ax1.set_ylabel('Torque [Nm]')
ax1.legend()

ax2.set_title('Position - ' + title)
ax2.plot(time, position, label='Position')
ax2.set_xlabel('Time [s]')
ax2.set_ylabel('Position [rad]')

ax2.legend()
plt.show()

