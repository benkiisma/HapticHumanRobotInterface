# Haptic Human Robot Interface

## Introduction

Electromyography (EMG) is a technique for the evaluation and recording of the electrical activity produced by the muscles. It is based on the simple fact that whenever a muscle contracts, an electric activity is generated which propagates through adjacent tissue and can be recorded from neighboring skin areas. As EMG signals reflect the activities of muscles, myoelectric control can thus be used as a control variable.

The amplitude of that EMG signal can range from 0 to 10 mV (peak-to-peak) or 0 to 1.5 (rms) and it can be represented by a Gaussian distribution function. More than that, the usable signals are in a certain frequency range (0-500Hz) because these are those with energy above the electrical noise level. It is important to note that the dominant energy is just in the 50-150 Hz range.

There are three common applications of EMG signals:
  • Determination of the activation timing of the muscle meaning that we look for the duration of the muscle’s excitation.
  • Estimation of the force produced by the muscle.
  • Determination of the rate at which a muscle fatigues through the analysis of the frequency spectrum of the signal.
  
These applications lead to very interesting solutions for real-life issues such as helping people during rehabilitation or control of prosthetics.

## Signal Processing

When working with EMG signals, the are some issues of concern that influence the fidelity of the signal. One of these is the signal-to-noise ratio (SNR) which is the ratio of energy in the EMG signal to the energy in the noise signal. I want the SNR to be as high as possible, so I chose to use a biceps as its ratio is one of the highest.
Another issue is the distortion of the signal, meaning that the relative contribution of any frequency component in the EMG signal should be altered. More than that, noise has a huge impact on the signal. Indeed, noise can emanate from various sources including inherent noise in the electronics components, ambient noise, motion artifacts and inherent instability of the signal. Therefore, preprocessing is important as it allows us to obtain an EMG signal that contains the maximum amount of information and the minimum contamination from noise.

So, the main idea of processing the signal is to maximize the signal-to-noise ratio with minimal distortion to the EMG. To do that, I implemented the following :
  1. An IIR high-pass filter with a cutoff frequency of 20Hz. This removes the offset in the data and drift from movement
  2. An IIR low-pass filter with a cutoff frequency of 400Hz. This removes most of the noize, while still allowing rapid changes in the signal
  3. A rectification (mean value) of the signal. This gives the envelope of the signal, which is much more representative of the contraction.
  4. A second low-pass filter, implemented with a rolling average of the last 500 values of the envelope (500ms). This further smoothes the signal and makes it less noisy after the rectification

This results in a signal relatively stable and noise-free. In order to convert it into a muscle contraction, I take its min and max value over the experiment and scale it between these two values which gives a value between 0 and 1 (hereafter called ”contraction”) representing the muscle contraction, which can be used for control.



