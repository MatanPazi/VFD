#! /usr/bin/python3

import numpy as np
import matplotlib.pyplot as plt
from scipy import signal
import Transformations as trs
import matplotlib.animation as animation

# parameter_to_change = 'none'
# parameter_to_change = 'speed'
parameter_to_change = 'idiq'

OMEGA = 2*np.pi*50      # Speed constant [rad/sec]
N = 500                 # Number of sampled
T = 1                   # Plot time [s]
t = np.linspace(0,T,N)  # Time array
omega = OMEGA*np.ones(N)# Speed array

# Index at which to change the selected parameter
chg_index = N//2

if (parameter_to_change == 'speed'):
    omega[chg_index : -1] *= 2

R = 0.1     # [Ohm]
L = 0.002   # [H]

# BEMF
Ke = -0.5   # [V/rad/sec]
KeA = Ke * np.sin(omega*t)                  
KeB = Ke * np.sin(omega*t - np.deg2rad(120))
KeC = Ke * np.sin(omega*t - np.deg2rad(240))
BEMFA = KeA * omega                         
BEMFB = KeB * omega                         
BEMFC = KeC * omega                         

# Desired stator current amplitude
Is = 100    # [A]
dqAng = np.deg2rad(90)*np.ones(N)        # dqAng = 0[deg] -> Only Id. dqAng = 90[deg]  ->  Only Iq
if (parameter_to_change == 'idiq'):
    dqAng[chg_index : -1] = np.deg2rad(45)

Iq = Is * np.sin(dqAng)
Id = -Is * np.cos(dqAng)

# Find ABC current
dAlphaAng = np.deg2rad(0)            # Angle in rad betwen Id and alpha, see: https://www.mathworks.com/help/mcb/ref/inverseparktransform.html
Ialpha, Ibeta = trs.InverseParkTransformation(Iq, Id, omega*t, dAlphaAng)
Ia, Ib, Ic = trs.alphaBeta0_to_abc(Ialpha, Ibeta)

# Find correlating phase voltages (V - BEMF) using I to V transfer function. Non-causal. Add far away pole (At FarPoleFreq) to make it causal.
FarPoleFreq = 10000*R/L
IToV_TF_num = [FarPoleFreq*L, FarPoleFreq*R]
IToV_TF_den = [1, FarPoleFreq]
IToV_TF = signal.TransferFunction(IToV_TF_num, IToV_TF_den)

tout,Va,xout = signal.lsim(IToV_TF, Ia, t)
tout,Vb,xout = signal.lsim(IToV_TF, Ib, t)
tout,Vc,xout = signal.lsim(IToV_TF, Ic, t)

DrivingVa = Va + BEMFA
DrivingVb = Vb + BEMFB
DrivingVc = Vc + BEMFC

Valpha, Vbeta = trs.abc_to_alphaBeta0(DrivingVa, DrivingVb, DrivingVc)
Vq, Vd = trs.alphaBeta0_to_dq0(Valpha, Vbeta, omega*t)
Vamp = np.sqrt(Vq*Vq + Vd*Vd)

# Animating the graphs
fig, (ax, ax_chg) = plt.subplots(2,1)
# Find min max values of represented data
max_y = int(max(max(Ia), max(BEMFA), max(Va), max(DrivingVa))) + 10
min_y = int(min(min(Ia), min(BEMFA), min(Va), min(DrivingVa))) - 10

line1 = ax.plot(t[1], Ia[1], label='Ia [A]')[0]
line2 = ax.plot(t[1], BEMFA[1], label='BEMFA [V]')[0]
line3 = ax.plot(t[1], Va[1], label='Va [V]')[0]
line4 = ax.plot(t[1], DrivingVa[1], label='DrivingVa [V]')[0]
line5 = ax.plot(t[1], Vamp[1], label='Vamp [V]')[0]

ax.set(xlim=[0, T], ylim=[min_y, max_y], xlabel='Time [s]', ylabel='Parameters')
ax.legend()

if (parameter_to_change == 'idiq'):
    line6 = ax_chg.plot(t[1], Iq[1], label='Iq [A]')[0]
    line7 = ax_chg.plot(t[1], Id[1], label='Id [A]')[0]
    ax_chg.set(xlim=[0, T], ylim=[min(min(Iq - 10), min(Id - 10)), max(max(Iq), max(Id)) + 10], xlabel='Time [s]', ylabel='Current [A]')
    ax_chg.legend()
else:
    line6 = ax_chg.plot(t[1], omega[1], label='Omega[Rad/sec]')[0]
    ax_chg.set(xlim=[0, T], ylim=[-10, max(omega)+10], xlabel='Time [s]', ylabel='Omega [rad/sec]')
    ax_chg.legend()


def update(frame):
    # update the line plot:
    line1.set_xdata(t[:frame])
    line1.set_ydata(Ia[:frame])
    line2.set_xdata(t[:frame])
    line2.set_ydata(BEMFA[:frame])
    line3.set_xdata(t[:frame])
    line3.set_ydata(Va[:frame])
    line4.set_xdata(t[:frame])
    line4.set_ydata(DrivingVa[:frame])
    line5.set_xdata(t[:frame])
    line5.set_ydata(Vamp[:frame])
    line6.set_xdata(t[:frame])
    if (parameter_to_change == 'idiq'):
        line6.set_ydata(Iq[:frame])
        line7.set_xdata(t[:frame])
        line7.set_ydata(Id[:frame])
        return (line1, line2, line3, line4, line5, line6, line7)
    else:
        line6.set_ydata(omega[:frame])
    return (line1, line2, line3, line4, line5, line6)

ani = animation.FuncAnimation(fig=fig, func=update, interval=1, frames=N, repeat=False)
plt.show()
