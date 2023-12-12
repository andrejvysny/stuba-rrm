import matplotlib.pyplot as plt
import csv

t_values = []
y_values = []
z_values = []
rz_values = []

yd_values = []
zd_values = []
rzd_values = []

ydd_values = []
zdd_values = []
rzdd_values = []

csv_file = '/home/andrejvysny/catkin_ws/zadanie3_output2.csv'

with open(csv_file, mode='r') as file:
    reader = csv.DictReader(file)
    for row in reader:
        t_values.append(float(row['t']))
        y_values.append(float(row['y']))
        z_values.append(float(row['z']))
        rz_values.append(float(row['rz']))


        yd_values.append(float(row['yd']))
        zd_values.append(float(row['zd']))
        rzd_values.append(float(row['rzd']))

        ydd_values.append(float(row['ydd']))
        zdd_values.append(float(row['zdd']))
        rzdd_values.append(float(row['rzdd']))




def position():
    plt.plot(t_values, y_values, label="Y Position [m]")
    plt.plot(t_values, z_values, label="Z Position [m]")
    plt.plot(t_values, rz_values, label="Z Rotation [rad]")
    plt.legend()
    plt.title(label="Priebeh polohy Y,Z a Rotacie osi Z")

    plt.xlabel("Time [s]")
    plt.show() 

def velocity():
    plt.plot(t_values, yd_values, label="Y Position [m/s]")
    plt.plot(t_values, zd_values, label="Z Position [m/s]")
    plt.plot(t_values, rzd_values, label="Z Rotation [rad/s]")
    plt.legend()
    plt.title(label="Priebeh rýchlosti Y,Z a Rotacie osi Z")

    plt.xlabel("Time [s]")
    plt.show() 


def acceleration():
    plt.plot(t_values, ydd_values, label="Y Position [m/s^2]")
    plt.plot(t_values, zdd_values, label="Z Position [m/s^2]")
    plt.plot(t_values, rzdd_values, label="Z Rotation [rad/s^2]")
    plt.legend()
    plt.title(label="Priebeh zrýchlenia Y,Z a Rotacie osi Z")

    plt.xlabel("Time [s]")
    plt.show() 


position()
velocity()
acceleration()