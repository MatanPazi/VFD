# MIT License

# Copyright (c) 2023 Matan Pazi

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

## .py script to show voltage to current transfer function and the use of PWM signals to generate current waveforms.
import numpy as np
import matplotlib.pyplot as plt
from scipy import signal

USE_PWM = 0

omega = 2*np.pi*5               # Sinewave freqeucny [rad/sec]
t = np.linspace(0,1,10000)      # Time array, 1 second, 1000 sample points.
R = 1                           # [Ohm]
L = 0.01                        # [Henry]

V = 1 + np.sin(omega*t)         # Desired Voltage Waveform
PWM_Freq = omega * 20           # PWM frequency
PWM = 1 + signal.square(PWM_Freq * t, duty=(V)/2)   # PWM signal correlating to the voltage sinusoidal waveform

# Find voltage by using V to I transfer function.
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
