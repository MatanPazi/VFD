#! /usr/bin/python3

import numpy as np
import matplotlib.pyplot as plt
from scipy import signal
import Transformations as trs

omega = 2*np.pi*300
t = np.linspace(0,0.01,1000)
t2 = t[1:-1]
R = 0.005
L = 0.00001

# BEMF
Ke = -0.0006
KeA = Ke * np.sin(omega*t)              # Sinusoidal BEMF
KeB = Ke * np.sin(omega*t - np.deg2rad(120))              # Sinusoidal BEMF
KeC = Ke * np.sin(omega*t - np.deg2rad(240))              # Sinusoidal BEMF
BEMFA = KeA * omega                      # BEMF
BEMFB = KeB * omega                      # BEMF
BEMFC = KeC * omega                      # BEMF

# Desired current vector
I = 100
dqAng = np.deg2rad(90)      # dqAng = 0[deg] -> Only Id. dqAng = 90[deg]  ->  Only Iq
Iq = I * np.sin(dqAng)
Id = -I * np.cos(dqAng)

# Find ABC current
dAlphaAng = np.deg2rad(0)            # Angle in rad betwen Id and alpha, see: https://www.mathworks.com/help/mcb/ref/inverseparktransform.html
Ialpha, Ibeta = trs.InverseParkTransformation(Iq, Id, omega*t, dAlphaAng)
Ia, Ib, Ic = trs.alphaBeta0_to_abc(Ialpha, Ibeta)

# Find correlating phase voltages (V - BEMF) using I to V transfer function. Non-causal. Add far away pole (At FarPoleFreq) to make it causal.
FarPoleFreq = 10000*R/L
IToV_TF_num = [FarPoleFreq*L, FarPoleFreq*R]
IToV_TF_den = [1, FarPoleFreq]
IToV_TF = signal.TransferFunction(IToV_TF_num, IToV_TF_den)

# # Uncomment to plot Bode.
# w, mag, phase = signal.bode(IToV_TF, w=None, n=100)
# plt.figure()
# plt.semilogx(w, mag)    # Bode magnitude plot
# plt.figure()
# plt.semilogx(w, phase)  # Bode phase plot
# plt.show()

tout,DeltaVa,xout = signal.lsim(IToV_TF, Ia, t)
tout,DeltaVb,xout = signal.lsim(IToV_TF, Ib, t)
tout,DeltaVc,xout = signal.lsim(IToV_TF, Ic, t)

Va = DeltaVa + BEMFA
Vb = DeltaVb + BEMFB
Vc = DeltaVc + BEMFC


# It can be seen very nicely, that with only Iq, the voltage (V) amplitude required is higher than with an added Id element.
plt.figure()
plt.xlabel('Time [Sec]')
plt.ylabel('[A] OR [V*100]')
plt.plot(t[1:-1],100*DeltaVa[1:-1], label='V-BEMF*100')
plt.plot(t[1:-1],100*BEMFA[1:-1], label='BEMFA*100')
plt.plot(t,Ia, label='Ia')
plt.plot(t[1:-1],100*(Va[1:-1]), label='V')
plt.legend()
plt.show()
