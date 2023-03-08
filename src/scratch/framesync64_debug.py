#!/usr/bin/env python3
'''plot frame sync debug files'''
import argparse, os, sys
import numpy as np, matplotlib.pyplot as plt
import scipy.signal as sig

def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('sources', nargs='+', help='input files')
    p.add_argument('-export', default=None, action='store_true', help='export figure')
    p.add_argument('-nodisplay', action='store_true', help='disable display')
    args = p.parse_args()

    for fname in args.sources:
        filename = framesync64_plot(fname,args.export)

def framesync64_plot(filename,export=None):
    # open file and read values
    fid         = open(filename,'rb')
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

    # plot impulse and spectral responses
    fig, _ax = plt.subplots(2,2,figsize=(12,12))
    ax = _ax.flatten()
    t = np.arange(len(buf))
    qpsk = np.exp(0.5j*np.pi*(np.arange(4)+0.5)) # payload constellation
    ax[0].plot(t,np.real(buf), t,np.imag(buf))
    ax[0].set_title('Raw I/Q Samples')
    ax[1].plot(f,psd)
    ax[1].set(xlim=(-0.5,0.5))
    ax[1].set_title('Power Spectral Density')
    ax[2].plot(np.real(payload_rx),  np.imag(payload_rx),  '.')
    ax[2].set_title('RX Payload Syms')
    ax[3].plot(np.real(payload_sym), np.imag(payload_sym), '.')
    ax[3].set_title('Synchronized Payload Syms')
    for _ax in ax[2:]:
        _ax.set(xlim=(-1.3,1.3), ylim=(-1.3,1.3))
        _ax.plot(np.real(qpsk),np.imag(qpsk),'rx')
        _ax.set_xlabel('Real')
        _ax.set_ylabel('Imag')
    for _ax in ax:
        _ax.grid(True)
    fig.suptitle('frame64, tau:%.6f, dphi:%.6f, phi:%.6f, rssi:%.3f dB, evm:%.3f' % \
        (tau_hat, dphi_hat, phi_hat, 20*np.log10(gamma_hat), evm))

    # eye diagrams
    num_symbols = 2      # number of symbols to display in eye diagram
    windows = 200        # a window is one path across display shown 
    sym_length = 7

    # resample data to at least 64x per symbol to emulate continuous waveform
    oversamp = 1;
    resamp = int(np.ceil(64/oversamp))
    rx_resamp = sig.resample(payload_rx, len(payload_rx)*resamp)
    samp_per_win = oversamp * resamp * num_symbols

    # divide by number of samples per win and then 
    # pad zeros to next higher multiple using tx_eye = np.array(tx_shaped), 
    # tx_eye.resize(N)
    N = len(rx_resamp)//samp_per_win

    rx_eye = np.array(rx_resamp)
    rx_eye.resize(N * samp_per_win)

    grouped = np.reshape(rx_resamp, [N, samp_per_win])    

    transient = sym_length // 2
    eye = np.real(grouped.T)

    # create an xaxis in samples np.shape(eye) gives the
    # 2 dimensional size of the eye data and the first element
    # is the interpolated number of samples along the x axis
    nsamps = np.shape(eye)[0]
    xaxis = np.arange(nsamps)/resamp

    plt.figure()

    # plot showing continuous trajectory of 
    plt.plot(xaxis, eye[:,transient:transient+windows])

    # actual sample locations
    plt.plot(xaxis[::resamp], eye[:,transient:transient+windows][::resamp],'b.')
    plt.title("Eye Diagram (Received Payload Symbols")
    plt.xlabel('Samples')
    plt.grid()

     # resample data to at least 64x per symbol to emulate continuous waveform
    sym_resamp = sig.resample(payload_sym, len(payload_sym)*resamp)

    # divide by number of samples per win and then 
    # pad zeros to next higher multiple using tx_eye = np.array(tx_shaped), 
    # tx_eye.resize(N)
    N = len(sym_resamp)//samp_per_win

    rx_eye = np.array(sym_resamp)
    rx_eye.resize(N * samp_per_win)

    grouped = np.reshape(sym_resamp, [N, samp_per_win])    

    transient = sym_length // 2
    eye = np.real(grouped.T)

    # create an xaxis in samples np.shape(eye) gives the
    # 2 dimensional size of the eye data and the first element
    # is the interpolated number of samples along the x axis
    nsamps = np.shape(eye)[0]
    xaxis = np.arange(nsamps)/resamp

    plt.figure()

    # plot showing continuous trajectory of 
    plt.plot(xaxis, eye[:,transient:transient+windows])

    # actual sample locations
    plt.plot(xaxis[::resamp], eye[:,transient:transient+windows][::resamp],'b.')
    plt.title("Eye Diagram (Syncronized Payload Symbols")
    plt.xlabel('Samples')
    plt.grid()


    if export==None:
        plt.show()
    else:
        fig.savefig(os.path.splitext(filename)[0]+'.png',bbox_inches='tight')

    plt.close()

if __name__ == '__main__':
    sys.exit(main())

