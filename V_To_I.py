import numpy as np
import matplotlib.pyplot as plt
from scipy import signal

USE_PWM = 0

omega = 2*np.pi*5               # Sinewave freqeucny [rad/sec]
t = np.linspace(0,1,10000)      # Time array, 1 second, 1000 sample points.
R = 1                           # [Ohm]
L = 0.01                        # [Henry]

V = 1 + np.sin(omega*t)         # Desired Current Waveform
PWM_Freq = omega * 20
PWM = 1 + signal.square(PWM_Freq * t, duty=(V)/2)

# Find voltage by using I to V transfer function.
V_To_I_TF_num = [1]
V_To_I_TF_den = [L, R]
V_To_I_TF = signal.TransferFunction(V_To_I_TF_num, V_To_I_TF_den)

if (not USE_PWM):
    tout,I,xout = signal.lsim(V_To_I_TF, V, t, X0 = 0.01)
else:
    tout,I,xout = signal.lsim(V_To_I_TF, PWM, t, X0 = 0.01)

plt.xlabel('Time [Sec]')
plt.ylabel('[A] OR [V]')
if (not USE_PWM):
    plt.plot(t,V, label='V')
else:
    plt.plot(t,PWM, label='V')
plt.plot(t,I, label='I')
plt.legend()
plt.show()