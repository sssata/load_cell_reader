import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import freqz, lfilter
import scipy.signal
import pathlib
from typing import List
import pandas as pd

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


# Plot original and filtered signals
def plot_signals(time: list, data: list, filters: List[DigitalFilter]):
    plt.figure(figsize=(10, 6))
    
    # Plot original signal
    plt.plot(time, data, label='Original Signal', color='blue')

    print(len(data))

    # Plot filtered signals
    for i, filter in enumerate(filters):
        filtered_data = lfilter(filter.b, filter.a, data)
        plt.plot(time, filtered_data, label=filter.name)
        print(f"{filter.name}: std dev: {np.std(filtered_data[-1000:])}")
        
    plt.xlabel('Time')
    plt.ylabel('Amplitude')
    plt.legend()


# Load sample data from CSV file
def load_data(file_path):
    data = pd.read_csv(file_path, sep=' +')
    return data


# Exponential Moving Average (EMA) filter
alpha_ema = 0.7

# Moving Average (MA) filter
window_size_ma = 10

# Create filters
filters = [
    # DigitalFilter("", fir_coeffs),
    DigitalFilter(
        "Bessel 1st order",
        *scipy.signal.iirfilter(
            N=1,
            Wn=60,
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
            Wn=75,
            btype="lowpass",
            analog=False,
            ftype="bessel",
            fs=fs,
            output="ba",
        ),
    ),
    DigitalFilter(
        "Bessel 3nd order",
        *scipy.signal.iirfilter(
            N=3,
            Wn=90,
            btype="lowpass",
            analog=False,
            ftype="bessel",
            fs=fs,
            output="ba",
        ),
    ),
    DigitalFilter(
        "FIR Filter 20",
        scipy.signal.firwin(
            numtaps = 20,
            cutoff = 50,
            pass_zero = "lowpass",
            fs=fs
        )
    ),
    DigitalFilter("EMA", [1 - alpha_ema], [1, -alpha_ema]),
    DigitalFilter("Moving Avg", np.ones(window_size_ma) / window_size_ma),
]

# Plot Bode plot
print("plotting bode plot")
plt.figure(figsize=(10, 6))

for i, filt in enumerate(filters):
    w, h = freqz(filt.b, filt.a, worN=1024, fs=fs)
    plt.semilogx(w, 20 * np.log10(abs(h)), label=filt.name)

plt.title("Bode Plot")
plt.ylabel("Magnitude (dB)")
plt.xlabel("Frequency (Hz)")
plt.ylim(-80, 10)  # For magnitude range from -100 dB to 10 dB
plt.grid(True)
plt.legend()

# Plot Step Response
step_response_duration_s = 0.2
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


data_path = pathlib.Path(__file__).parent / "raw_data.csv"

data = load_data(data_path)

print(data)
print(data.columns.tolist())

plot_signals(list(data["time_us"]), list(data["reading"]), filters)

plt.show()


inputs = [0, 1, 1, 1, 1, 1]

print(f"Inputs: {inputs}")

# Calc exact values for filters
for filter in filters:
    print(f"{filter.name}")
    print(f"B:{filter.b}")
    print(f"A:{filter.a}")
    outputs = lfilter(filter.b, filter.a, inputs)
    print(f"Output: {outputs}")
