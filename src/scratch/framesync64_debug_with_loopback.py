#!/usr/bin/env python3
'''plot frame sync debug files'''
import argparse, os, sys
import numpy as np, matplotlib.pyplot as plt
import scipy.signal as sig

def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('sources', nargs='+', help='input files')
    args = p.parse_args()

    fname1  = args.sources[0];
    fname2  = args.sources[1];

    framesync64_plot(fname1,fname2)

def framesync64_plot(filename1, filename2,export=None):
    # open file and read values
    fid         = open(filename1,'rb')
    buf         = np.fromfile(fid, count=1440, dtype=np.csingle)
    tau_hat     = np.fromfile(fid, count=   1, dtype=np.single)
    dphi_hat    = np.fromfile(fid, count=   1, dtype=np.single)
    phi_hat     = np.fromfile(fid, count=   1, dtype=np.single)
    gamma_hat   = np.fromfile(fid, count=   1, dtype=np.single)
    evm         = np.fromfile(fid, count=   1, dtype=np.single)
    payload_rx  = np.fromfile(fid, count= 630, dtype=np.csingle)
    payload_sym = np.fromfile(fid, count= 600, dtype=np.csingle)
    payload_dec = np.fromfile(fid, count=  72, dtype=np.int8)

    # compute filter response in dB
    nfft = 2400
    f = np.arange(nfft)/nfft-0.5
    psd = np.abs(np.fft.fftshift(np.fft.fft(buf, nfft)))**2
    m   = int(0.01*nfft)
    w   = np.hamming(2*m+1)
    h   = np.concatenate((w[m:], np.zeros(nfft-2*m-1), w[:m])) / (sum(w) * nfft)
    H   = np.fft.fft(h)
    psd = 10*np.log10( np.real(np.fft.ifft(H * np.fft.fft(psd))) )
    #psd = 20*np.log10(np.abs(np.fft.fftshift(np.fft.fft(buf, nfft))))


    # open file and read values
    fid2         = open(filename2,'rb')
    buf2         = np.fromfile(fid2, count=1440, dtype=np.csingle)
    tau2_hat     = np.fromfile(fid2, count=   1, dtype=np.single)
    dphi2_hat    = np.fromfile(fid2, count=   1, dtype=np.single)
    phi2_hat     = np.fromfile(fid2, count=   1, dtype=np.single)
    gamma2_hat   = np.fromfile(fid2, count=   1, dtype=np.single)
    evm2         = np.fromfile(fid2, count=   1, dtype=np.single)
    payload2_rx  = np.fromfile(fid2, count= 630, dtype=np.csingle)
    payload2_sym = np.fromfile(fid2, count= 600, dtype=np.csingle)
    payload2_dec = np.fromfile(fid2, count=  72, dtype=np.int8)

    # compute filter response in dB
    psd2 = np.abs(np.fft.fftshift(np.fft.fft(buf2, nfft)))**2
    psd2 = 10*np.log10( np.real(np.fft.ifft(H * np.fft.fft(psd2))) )
    #psd2 = 20*np.log10(np.abs(np.fft.fftshift(np.fft.fft(buf2, nfft))))


    fig, ax = plt.subplots(figsize=(1,1), constrained_layout=False)

    # plot impulse and spectral responses
    t = np.arange(len(buf))

    ax.plot(f,psd)
    ax.plot(f,psd2)
    ax.set(xlim=(-0.5,0.5))
    ax.set_title('Power Spectral Density')


    fig.suptitle('First: tau:%.6f, dphi:%.6f, phi:%.6f, rssi:%.3f dB, evm:%.3f\nSecond: tau:%.6f, dphi:%.6f, phi:%.6f, rssi:%.3f dB, evm:%.3f' % \
        (tau_hat, dphi_hat, phi_hat, 20*np.log10(gamma_hat), evm, tau2_hat, dphi2_hat, phi2_hat, 20*np.log10(gamma2_hat), evm2))

    plt.show()
    plt.close()

if __name__ == '__main__':
    sys.exit(main())

