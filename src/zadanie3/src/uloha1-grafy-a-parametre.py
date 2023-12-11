import numpy as np
import math
import matplotlib.pyplot as plt

def rad(deg):
      return math.radians(deg)

def M6(t0,tf):
    return np.array([
        [1, t0, t0**2, t0**3, t0**4, t0**5],
        [0, 1, 2*t0, 3*t0**2, 4*t0**3, 5*t0**4],
        [0,0,2,6*t0,12*t0**2,20*t0**3],
        [1, tf, tf**2, tf**3, tf**4, tf**5],
        [0, 1, 2*tf, 3*tf**2, 4*tf**3, 5*tf**4],
        [0,0,2,6*tf,12*tf**2,20*tf**3],
    ])


def Q6(Qt0,Qt0d,Qt0dd,Qtf,Qtfd,Qtfdd):
        return np.array([
        [Qt0],
        [Qt0d],
        [Qt0dd],
        [Qtf],
        [Qtfd],
        [Qtfdd]
    ])


def getPosition(t, a):
      return a[0] + a[1]*t + a[2]*t**2 + a[3]*t**3 + + a[4]*t**4 + a[5]*t**5

def getSpeed(t,a):
      return 0 + a[1] + 2*a[2]*t + 3*a[3]*t**2 + + 4*a[4]*t**3 + 5*a[5]*t**4

def getAcceleration(t,a):
      return 0 + 0 + 2*a[2] + 6*a[3]*t + 12*a[4]*t**2 + 20*a[5]*t**3  


def simulate(t0,tf,a):
    position = []
    speed = []
    acceleration = []
    time = []
    for t in np.arange(t0, tf, 0.05):
        position.append(
               getPosition(t,a)
        )
        speed.append(
                getSpeed(t,a)
        )
        acceleration.append(
                getAcceleration(t,a)
        )
        time.append(t)
    return [position,speed,acceleration,time]


def Q1SimulationPlot(axis):
    M = M6(0,4)
    Q = Q6(0,0,0,rad(90),0,0)
    Minv = np.linalg.inv(M)
    a_Q1_from_0_to_4 = np.dot(Minv,Q)

    [position,speed,acceleration,time] = simulate(0,4,a_Q1_from_0_to_4)

    axis.set_title(label="Q1")
    axis.plot(time, position, label="Position [rad]")
    axis.plot(time, speed, label= "Speed [m/s]")
    axis.plot(time, acceleration, label= "Acceleration [m/s^2]")
    axis.legend()
    


def Q3SimulationPlot(axis):
    M = M6(0,1)
    Q = Q6(0,0,0,rad(30),0,0)
    Minv = np.linalg.inv(M)
    a_Q3_from_0_to_1 = np.dot(Minv,Q)

    M = M6(1,4)
    Q = Q6(rad(30),0,0,0,0,0)
    Minv = np.linalg.inv(M)
    a_Q3_from_1_to_4 = np.dot(Minv,Q)

    [position1,speed1,acceleration1,time1] = simulate(0,1,a_Q3_from_0_to_1)
    [position2,speed2,acceleration2,time2] = simulate(1,4,a_Q3_from_1_to_4)
    
    axis.set_title(label="Q3")
    axis.plot(time1+time2, position1+position2, label="Position [rad]")
    axis.plot(time1+time2, speed1+speed2, label= "Speed [m/s]")
    axis.plot(time1+time2, acceleration1+acceleration2, label= "Acceleration [m/s^2]")
    axis.legend()
    
  
figure, axis = plt.subplots(1, 2) 

Q1SimulationPlot(axis[0])
Q3SimulationPlot(axis[1])
  
plt.show() 