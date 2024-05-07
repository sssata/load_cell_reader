import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import freqz, lfilter
import scipy.signal


fs = 860  # max freq of ads1115

nyquist = fs / 2


class DigitalFilter:
    def __init__(self, name, b, a=1):
        self.name = name
        self.b = np.array(b, dtype=np.float64)
        self.a = (
            np.array(a, dtype=np.float64)
            if a is not None
            else np.array([], dtype=np.float64)
        )

    def freq_response(self):
        w, h = freqz(self.b, self.a)
        return w, 20 * np.log10(abs(h))

    def step_response(self, n=100):
        return np.arange(n), lfilter(self.b, self.a, np.ones(n))


# FIR Filter coefficients
fir_coeffs = np.array([0.25, 0.5, 0.25])

# IIR Filter coefficients (single-pole low-pass)
iir_coeffs = np.array([0.5])

# Exponential Moving Average (EMA) filter
alpha_ema = 0.9

# Moving Average (MA) filter
window_size_ma = 50

# Create filters
filters = [
    # DigitalFilter("", fir_coeffs),
    DigitalFilter(
        "Bessel 1st order",
        *scipy.signal.iirfilter(
            N=1,
            Wn=nyquist / 100,
            btype="lowpass",
            analog=False,
            ftype="bessel",
            fs=fs,
            output="ba",
        ),
    ),
    DigitalFilter(
        "Bessel 2nd order",
        *scipy.signal.iirfilter(
            N=2,
            Wn=nyquist / 100,
            btype="lowpass",
            analog=False,
            ftype="bessel",
            fs=fs,
            output="ba",
        ),
    ),
    DigitalFilter("EMA", [1 - alpha_ema], [1, -alpha_ema]),
    DigitalFilter("Moving Avg", np.ones(window_size_ma) / window_size_ma),
]

# Plot Bode plot
print("plotting bode plot")
plt.figure(figsize=(10, 6))

for i, filt in enumerate(filters):
    w, h = freqz(filt.b, filt.a, fs=fs)
    plt.semilogx(w, 20 * np.log10(abs(h)), label=filt.name)

plt.title("Bode Plot")
plt.ylabel("Magnitude (dB)")
plt.xlabel("Frequency (Hz)")
plt.ylim(-80, 10)  # For magnitude range from -100 dB to 10 dB
plt.grid(True)
plt.legend()

# Plot Step Response
step_response_duration_s = 1.0
samples = int(fs * step_response_duration_s)
print("plotting step response")
plt.figure(figsize=(10, 6))

for i, filt in enumerate(filters):
    y = lfilter(filt.b, filt.a, np.ones(samples))
    t = np.linspace(0, step_response_duration_s, samples)
    plt.plot(t, y, label=filt.name)

plt.title("Step Response")
plt.xlabel("Time [sec]")
plt.ylabel("Output")
plt.grid(True)
plt.legend()

plt.tight_layout()
plt.show()
