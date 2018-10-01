
import numpy as np
import matplotlib.pyplot as plt


x = np.load('cm.npy')


def clean_data(x):
    N=x.size
    fft = np.fft.rfft(x)

    fft_sparse = fft.copy()
    fft_sparse[0] = 0
    fft_sparse[1] = 0
    fft_sparse[2] = 0
    fft_sparse[3] = 0
    fft_sparse[4] = 0
    fft_sparse[6] = 0
#
    fft_sparse[90:] = 0
    interp_data = np.fft.irfft(fft_sparse), fft_sparse
    return interp_data


c, s = clean_data(x)



# e^ix = cos(x) + i sin(x)
N=4096
mag = 2*np.abs(s)/N
phase = np.arctan2(np.imag(s), np.real(s))

recon = np.zeros((N))
for i in range(int(N/2)):
    recon += np.cos(2*np.pi*i*np.linspace(0,1,N) + phase[i]) * mag[i]
    
    
plt.plot(recon, label='recon')
plt.plot(x, label='orig')
plt.plot(c, label='ifft')
plt.legend()
plt.show()


