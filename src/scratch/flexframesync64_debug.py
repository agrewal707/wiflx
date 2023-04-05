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
        filename = flexframesync_plot(fname,args.export,args.nodisplay)

def flexframesync_plot(filename,export=None,nodisplay=True):
    # open file and read values
    fid         = open(filename,'rb')
    buf_len     = np.fromfile(fid, count=   1, dtype=np.uint32)
    print(f'buf_len: {buf_len[0]}')
    buf         = np.fromfile(fid, count=   buf_len[0], dtype=np.csingle)
    tau_hat     = np.fromfile(fid, count=   1, dtype=np.single)
    dphi_hat    = np.fromfile(fid, count=   1, dtype=np.single)
    phi_hat     = np.fromfile(fid, count=   1, dtype=np.single)
    gamma_hat   = np.fromfile(fid, count=   1, dtype=np.single)
    evm         = np.fromfile(fid, count=   1, dtype=np.single)
    payload_sym_len = np.fromfile(fid, count= 1, dtype=np.uint32)
    print(f'payload_sym_len: {payload_sym_len[0]}')
    payload_sym     = np.fromfile(fid, count= payload_sym_len[0], dtype=np.csingle)
    payload_mod_len = np.fromfile(fid, count= 1, dtype=np.uint32)
    print(f'payload_mod_len: {payload_mod_len[0]}')
    payload_mod     = np.fromfile(fid, count= payload_mod_len[0], dtype=np.csingle)
    payload_dec_len = np.fromfile(fid, count= 1, dtype=np.uint32)
    print(f'payload_dec_len: {payload_dec_len[0]}')
    payload_dec     = np.fromfile(fid, count= payload_dec_len[0], dtype=np.int8)

    # compute smooth spectral response in dB
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
    ax[2].plot(np.real(payload_sym),  np.imag(payload_sym),  '.')
    ax[2].set_title('RX Payload Syms')
    ax[3].plot(np.real(payload_mod), np.imag(payload_mod), '.')
    ax[3].set_title('Synchronized Payload Syms')
    for _ax in ax[2:]:
        _ax.set(xlim=(-1.3,1.3), ylim=(-1.3,1.3))
        _ax.plot(np.real(qpsk),np.imag(qpsk),'rx')
        _ax.set_xlabel('Real')
        _ax.set_ylabel('Imag')
    for _ax in ax:
        _ax.grid(True)
    title = '%s, tau:%9.6f, dphi:%9.6f, phi:%9.6f, rssi:%6.3f dB, evm:%6.3f' % \
        (filename, tau_hat, dphi_hat, phi_hat, 20*np.log10(gamma_hat), evm)
    print(title)
    fig.suptitle(title)
    
    # eye diagrams
    num_symbols = 2      # number of symbols to display in eye diagram
    windows = 200        # a window is one path across display shown 
    sym_length = 7

    # PAYLOAD SYMBOLS BEFORE PILOTS ARE PROCESSED

    # resample data to at least 64x per symbol to emulate continuous waveform
    oversamp = 1;
    resamp = int(np.ceil(64/oversamp))
    payload_sym_resamp = sig.resample(payload_sym, len(payload_sym)*resamp)
    samp_per_win = oversamp * resamp * num_symbols

    # divide by number of samples per win and then 
    # pad zeros to next higher multiple using 
    # eye = np.array(resamp), eye.resize(N)
    N = len(payload_sym_resamp)//samp_per_win
    payload_sym_eye = np.array(payload_sym_resamp)
    payload_sym_eye.resize(N * samp_per_win)
    grouped = np.reshape(payload_sym_eye, [N, samp_per_win])

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

    # PAYLOAD SYMBOLS AFTER PILOTS ARE PROCESSED

    # resample data to at least 64x per symbol to emulate continuous waveform
    payload_mod_resamp = sig.resample(payload_mod, len(payload_mod)*resamp)

    # divide by number of samples per win and then 
    # pad zeros to next higher multiple using 
    # eye = np.array(resamp), eye.resize(N)
    N = len(payload_mod_resamp)//samp_per_win
    payload_mod_eye = np.array(payload_mod_resamp)
    payload_mod_eye.resize(N * samp_per_win)
    grouped = np.reshape(payload_mod_eye, [N, samp_per_win])    

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
    plt.title("Eye Diagram (Syncronized Payload Symbols)")
    plt.xlabel('Samples')
    plt.grid()

    if not nodisplay:
        plt.show()
    if export is not None:
        fig.savefig(os.path.splitext(filename)[0]+'.png',bbox_inches='tight')
    plt.close()

if __name__ == '__main__':
    sys.exit(main())

