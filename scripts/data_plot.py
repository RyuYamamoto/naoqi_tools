import matplotlib.pyplot as plt

f = open("plot.csv", "r")
marker_data = f.readlines()

x_list = []
y_list = []
theta_list = []
for i in marker_data:
    _marker_data = i.split(",")
    x_list.append(float(_marker_data[0]))
    y_list.append(float(_marker_data[1]))
    theta_list.append(float(_marker_data[2]))
time = [i for i in range(len(marker_data))]

fig = plt.figure()

ax1 = fig.add_subplot(3,1,1)
ax1.plot(time, x_list, label="observe")
ax1.set_xlabel("x[m]")
ax1.set_ylabel("step")
ax1.legend()

ax2 = fig.add_subplot(3,1,2)
ax2.plot(time, y_list, label="observe")
ax2.set_xlabel("y[m]")
ax2.set_ylabel("step")
ax2.legend()

ax3 = fig.add_subplot(3,1,3)
ax3.plot(time, theta_list, label="observe")
ax3.set_xlabel("theta[rad]")
ax3.set_ylabel("step")
ax3.legend()

plt.tight_layout()
plt.show()