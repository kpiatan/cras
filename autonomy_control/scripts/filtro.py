#!/usr/bin/env python
from numpy import cos, sin, pi, absolute, arange
from scipy.signal import kaiserord, lfilter, firwin, freqz
from pylab import figure, clf, plot, xlabel, ylabel, xlim, ylim, title, grid, axes, show

import math
from sensor_msgs.msg import LaserScan, PointCloud
from geometry_msgs.msg import Point

taps = 0
N = 0

def calcularFiltro():
    global taps
    global N
    #------------------------------------------------
    # Create a FIR filter
    #------------------------------------------------

    # The Nyquist rate of the signal.
    #nyq_rate = sample_rate / 2.0

    # The desired width of the transition from pass to stop,
    # relative to the Nyquist rate.  We'll design the filter
    # with a 5 Hz transition width.
    width = 0.05
    # The desired attenuation in the stop band, in dB.
    ripple_db = 80.0

    # Compute the order and Kaiser parameter for the FIR filter.
    N, beta = kaiserord(ripple_db, width)

    # The cutoff frequency of the filter.
    cutoff = 0.2

    # Use firwin with a Kaiser window to create a lowpass FIR filter.
    taps = firwin(N, cutoff, window=('kaiser', beta))
    return

def criarGraficos():
    #------------------------------------------------
    # Plot the FIR filter coefficients.
    #------------------------------------------------

    #t = arange(1045)

    # Use lfilter to filter x with the FIR filter.


    #------------------------------------------------
    # Plot the original and filtered signals.
    #------------------------------------------------

    # The phase delay of the filtered signal.
    delay = 0.5 * (N-1)

    figure(1)
    plot(taps, 'bo-', linewidth=2)
    title('Filter Coefficients (%d taps)' % N)
    grid(True)

    #------------------------------------------------
    # Plot the magnitude response of the filter.
    #------------------------------------------------

    figure(2)
    clf()
    w, h = freqz(taps, worN=8000)
    plot((w/pi), absolute(h), linewidth=2)
    xlabel('Frequency (Normalized)')
    ylabel('Gain')
    title('Frequency Response')
    ylim(-0.05, 1.05)
    grid(True)

#    ax1 = axes([0.42, 0.6, .45, .25])
#    plot((w/pi), absolute(h), linewidth=2)
#    xlim(0,8.0)
#    ylim(0.9985, 1.001)
#    grid(True)

    # Lower inset plot
    #ax2 = axes([0.42, 0.25, .45, .25])
    #plot((w/pi), absolute(h), linewidth=2)
    #xlim(12.0, 20.0)
    #ylim(0.0, 0.0025)
    #grid(True)
    #if not rospy.is_shutdown():
    #    pub.publish()
    #figure(3)
    #clf()
    # Plot the original signal.
    #plot(t, x)
    # Plot the filtered signal, shifted to compensate for the phase delay.
    #plot(t-delay, filtered_x, 'r-')
    # Plot just the "good" part of the filtered signal.  The first N-1
    # samples are "corrupted" by the initial conditions.
    #plot(t[N-1:]-delay, filtered_x[N-1:], 'g', linewidth=4)

    #print len(filtered_x)

    #xlabel('t')
    #grid(True)

    show()

    return

def aplicarFiltro(data):
    global taps
    global N
    scan = PointCloud()

    scan.header = data.header
    for i in range(0,len(data.points)):
        scan.points.append(Point(data.points[i].x,data.points[i].y,data.points[i].z))

    signal = [0.0]*2*len(data.points)
    #x = [0.0]*(kFin-kIni+1)
#    j = 0
    for i in range(0,len(data.points)):
        signal[i] = data.points[i].z;
        signal[i+len(data.points)] = data.points[len(data.points)-1].z;
#        x[j+2*(kFin-kIni+1)] = data.ranges[i];
#        j = j + 1

    filtered_signal = lfilter(taps, 1.0, signal)
    delay = int(0.5 * (N-1))

    for i in range(len(data.points)):
        scan.points[i].z = filtered_signal[delay + i]

    return scan

def calcularIndices(data):
    kIni=0
    while data.ranges[kIni] == 0:
        kIni = kIni+1

    kFin=len(data.ranges)-1
    while data.ranges[kFin] == 0:
        kFin = kFin - 1
    return (kIni,kFin)
