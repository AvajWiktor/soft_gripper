import matplotlib.pyplot as plt
import


with open('torque_data.json') as json_file:
	data = (json.load(json_file)

x_data = data[0]
y_data = data[1]

print(x_data)
    
plt.plot(x_data, y_data)
plt.ylabel('some numbers')
plt.show()
