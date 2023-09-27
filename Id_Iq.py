#! /usr/bin/python3

import numpy as np
import matplotlib.pyplot as plt
from scipy import signal
import Transformations as trs
import matplotlib.animation as animation

# parameter_to_change = 'none'
# parameter_to_change = 'speed'
parameter_to_change = 'idiq'

OMEGA = 2*np.pi*50
N = 1000
T = 1
t = np.linspace(0,T,N)
omega = OMEGA*np.ones(N)
chg_index = N//2

if (parameter_to_change == 'speed'):
    omega[chg_index : -1] *= 2

R = 0.5
L = 0.001

# BEMF
Ke = -0.6
KeA = Ke * np.sin(omega*t)              # Sinusoidal BEMF
KeB = Ke * np.sin(omega*t - np.deg2rad(120))              # Sinusoidal BEMF
KeC = Ke * np.sin(omega*t - np.deg2rad(240))              # Sinusoidal BEMF
BEMFA = KeA * omega                      # BEMF
BEMFB = KeB * omega                      # BEMF
BEMFC = KeC * omega                      # BEMF

# Desired current vector
I = 100
dqAng = np.deg2rad(90)*np.ones(N)        # dqAng = 0[deg] -> Only Id. dqAng = 90[deg]  ->  Only Iq
if (parameter_to_change == 'idiq'):
    dqAng[chg_index : -1] *= 0.3

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


# # It can be seen very nicely, that with only Iq, the voltage (V) amplitude required is higher than with an added Id element.
# plt.figure()
# plt.xlabel('Time [Sec]')
# plt.ylabel('[A] OR [V*100]')
# plt.plot(t[1:-1],100*DeltaVa[1:-1], label='V-BEMF*100')
# plt.plot(t[1:-1],100*BEMFA[1:-1], label='BEMFA*100')
# plt.plot(t,Ia, label='Ia')
# plt.plot(t[1:-1],100*(Va[1:-1]), label='V')
# plt.legend()
# plt.show()


# Animating the graphs
fig, (ax, ax_chg) = plt.subplots(2,1)
# Find min max values of represented data
max_y = int(max(max(Ia), max(BEMFA), max(DeltaVa), max(Va))) + 10
min_y = int(min(min(Ia), min(BEMFA), min(DeltaVa), min(Va))) - 10

line1 = ax.plot(t[1], Ia[1], label='Ia[A]')[0]
line2 = ax.plot(t[1], BEMFA[1], label='BEMFA[V]')[0]
line3 = ax.plot(t[1], DeltaVa[1], label='DrivingV - BEMF [V]')[0]
line4 = ax.plot(t[1], Va[1], label='DrivingV [V]')[0]
ax.set(xlim=[0, T], ylim=[min_y, max_y], xlabel='Time [s]', ylabel='Parameters')
ax.legend()

if (parameter_to_change == 'idiq'):
    line5 = ax_chg.plot(t[1], Iq[1], label='Iq [A]')[0]
    line6 = ax_chg.plot(t[1], Id[1], label='Id [A]')[0]
    ax_chg.set(xlim=[0, T], ylim=[min(-10, min(Id - 10)), max(max(Iq), max(Id)) + 10], xlabel='Time [s]', ylabel='Current [A]')
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
    line3.set_ydata(DeltaVa[:frame])
    line4.set_xdata(t[:frame])
    line4.set_ydata(Va[:frame])
    line5.set_xdata(t[:frame])
    if (parameter_to_change == 'idiq'):
        line5.set_ydata(Iq[:frame])
        line6.set_xdata(t[:frame])
        line6.set_ydata(Id[:frame])
        return (line1, line2, line3, line4, line5, line6)
    else:
        line5.set_ydata(omega[:frame])
    return (line1, line2, line3, line4, line5)

ani = animation.FuncAnimation(fig=fig, func=update, interval=1, frames=N)
plt.show()
