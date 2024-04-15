import numpy as np
from numpy import cos, sin, pi

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import sympy as sp

DOF = 2

q1, q2 = sp.symbols('q1 q2')

# Value of pi from the symbolic library for convenience
spi = sp.pi

# Define DH table

DH_params = []

DH_params.append([19.7452, q1, 0, -spi/2])
DH_params.append([0, q2, 35.796, 0])

DH_params

print(DH_params)