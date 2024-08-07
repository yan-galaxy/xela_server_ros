import numpy as np
import scipy.fftpack
import matplotlib.pyplot as plt

def load_data(filename):
    with open(filename, 'r') as file:
        data = [float(line.strip()) for line in file]
    return np.array(data)

def perform_fft(data, sampling_rate):
    N = len(data)
    T = 1.0 / sampling_rate
    x = np.linspace(0.0, N*T, N, endpoint=False)
    yf = scipy.fftpack.fft(data)
    xf = np.fft.fftfreq(N, T)[:N//2]

    # Plot original waveform
    plt.figure()
    plt.subplot(2, 1, 1)
    plt.plot(x, data)
    plt.grid()
    plt.xlabel('Time (s)')
    plt.ylabel('Amplitude')
    plt.title('Original Waveform')

    # Plot frequency spectrum
    plt.subplot(2, 1, 2)
    plt.plot(xf, 2.0/N * np.abs(yf[:N//2]))
    plt.grid()
    plt.xlabel('Frequency (Hz)')
    plt.ylabel('Magnitude')
    plt.title('Frequency Spectrum')

    plt.tight_layout()
    plt.show()

    return xf, yf

if __name__ == "__main__":
    filename = '/home/galaxy/Desktop/Xela_ws/src/threed_viz/data/xela_z.txt'
    sampling_rate = 100  # Hz

    data = load_data(filename)
    if data.size == 0:
        print("Error: No data loaded.")
    else:
        frequencies, fft_values = perform_fft(data, sampling_rate)
        
        for freq, magnitude in zip(frequencies, np.abs(fft_values[:len(frequencies)])):
            print(f"Frequency: {freq:.2f} Hz, Magnitude: {magnitude:.2f}")
