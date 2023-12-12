import numpy as np
import math
import matplotlib.pyplot as plt


def rad(deg):
    return math.radians(deg)


def M6(t0, tf):
    return np.array([
        [1, t0, t0 ** 2, t0 ** 3, t0 ** 4, t0 ** 5],
        [0, 1, 2 * t0, 3 * t0 ** 2, 4 * t0 ** 3, 5 * t0 ** 4],
        [0, 0, 2, 6 * t0, 12 * t0 ** 2, 20 * t0 ** 3],
        [1, tf, tf ** 2, tf ** 3, tf ** 4, tf ** 5],
        [0, 1, 2 * tf, 3 * tf ** 2, 4 * tf ** 3, 5 * tf ** 4],
        [0, 0, 2, 6 * tf, 12 * tf ** 2, 20 * tf ** 3],
    ])


def Q6(Qt0, Qt0d, Qt0dd, Qtf, Qtfd, Qtfdd):
    return np.array([
        [Qt0],
        [Qt0d],
        [Qt0dd],
        [Qtf],
        [Qtfd],
        [Qtfdd]
    ])


def getPosition(t, a):
    return a[0] + a[1] * t + a[2] * t ** 2 + a[3] * t ** 3 + + a[4] * t ** 4 + a[5] * t ** 5


def getSpeed(t, a):
    return 0 + a[1] + 2 * a[2] * t + 3 * a[3] * t ** 2 + + 4 * a[4] * t ** 3 + 5 * a[5] * t ** 4


def getAcceleration(t, a):
    return 0 + 0 + 2 * a[2] + 6 * a[3] * t + 12 * a[4] * t ** 2 + 20 * a[5] * t ** 3


def getJerk(t, a):
    return 6 * a[3] + 24 * a[4] * t + 60 * a[5] * t ** 2


def calcParam(t0, tf, param0, paramf):
    M = M6(t0, tf)
    Q = Q6(param0, 0, 0, paramf, 0, 0)
    Minv = np.linalg.inv(M)
    a_param_t0totf = np.dot(Minv, Q)
    print(f"{t0}->{tf}: {a_param_t0totf}")

    return a_param_t0totf


a_x_0_to_1 = calcParam(0, 1, 1.6, 1)
