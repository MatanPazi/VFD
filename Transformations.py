#! /usr/bin/python3
# This file is a library for transformation functions

import numpy as np

# Clarke Transform: ABC to Alpha-Beta-0
def abc_to_alphaBeta0(a, b, c):
    alpha = (2/3)*(a - b/2 - c/2)
    beta  = (2/3)*(np.sqrt(3)*(b-c)/2)
    return alpha, beta

# Inverse Clarke Transform: alphaBeta0 to abc
def alphaBeta0_to_abc(alpha, beta):
    a = alpha
    b = -alpha/2 + beta*np.sqrt(3)/2
    c = -alpha/2 - beta*np.sqrt(3)/2
    return a, b, c


# Inverse Park Transform
# def dq0_to_abc(d, q, wt):
#     a = d * np.cos(wt) - q * np.sin(wt)
#     b = d * np.cos(wt - 2*np.pi/3) - q * np.sin(wt - 2*np.pi/3)
#     c = d * np.cos(wt + 2*np.pi/3) - q * np.sin(wt + 2*np.pi/3)
#     return a, b, c

# Park Transform:
# d-axis aligned with the α-axis.
def alphaBeta0_to_dq0(alpha, beta, wt):
    d = alpha*np.cos(wt) + beta*np.sin(wt)
    q = -alpha*np.sin(wt) + beta*np.cos(wt)
    return q, d

# # q-axis aligned with the α-axis.
# def alphaBeta0_to_dq0(alpha, beta, wt):
#     d = alpha*np.sin(wt) - beta*np.cos(wt)
#     q = alpha*np.cos(wt) + beta*np.sin(wt)
#     return q, d    




# Inverse Park Transformation:
# d-axis aligned with the α-axis.
def InverseParkTransformation(q, d, wt, delta):
    alpha = d * np.cos(wt + delta) - q * np.sin(wt + delta)
    beta  = d * np.sin(wt + delta) + q * np.cos(wt + delta)
    return alpha, beta

# # q-axis aligned with the α-axis.
# def InverseParkTransformation(q, d, wt, delta):
#     alpha = d * np.sin(wt + delta) + q * np.cos(wt + delta)
#     beta  = -d * np.cos(wt + delta) + q * np.sin(wt + delta)
#     return alpha, beta    












# Inverse Park Transform
def dq0_to_abc(d, q, wt, delta):
    a = d*np.sin(wt+delta) + q*np.cos(wt+delta)
    b = d*np.sin(wt-(2*np.pi/3)+delta) + q*np.cos(wt-(2*np.pi/3)+delta)
    c = d*np.sin(wt+(2*np.pi/3)+delta) + q*np.cos(wt+(2*np.pi/3)+delta)
    return a, b, c    

# # Park Transform: abc to dq0
# def abc_to_dq0(a, b, c, wt, delta):
#   d = (2/3)*(a*np.sin(wt+delta) + b*np.sin(wt+delta-(2*np.pi/3)) + c*np.sin(wt+delta+(2*np.pi/3)))
#   q = (2/3)*(a*np.cos(wt+delta) + b*np.cos(wt+delta-(2*np.pi/3)) + c*np.cos(wt+delta+(2*np.pi/3)))
#   return d, q

#   # Park Transform: abc to dq0
# def abc_to_dq0(a, b, c, wt):
#     d = (2/3) * (a * np.cos(wt) + b * np.cos(wt - 2*np.pi/3) + c * np.cos(wt + 2*np.pi/3))
#     q = (2/3) * (-a * np.sin(wt) - b * np.sin(wt - 2*np.pi/3) - c * np.sin(wt + 2*np.pi/3))
#     return d, q