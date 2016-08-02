import sys
import skrf
import matplotlib.pyplot as plt
import numpy as np
skrf.stylely()

def tline_input(zl, z0, t, f):
    c = 299792458
    w = c/f
    b = np.pi*2/w
    l = t*c
    return z0*(zl+1j*z0*np.tan(b*l))/(z0+1j*zl*np.tan(b*l))

def gamma(zl, z0):
    return (zl-z0)/(zl+z0)

def make_standards():
    open_c = [20e-15, -1140e-27, 2176e-36, -213e-45]
    open_offset_t = 35.7e-12
    open_offset_z0 = 50.0
    short_offset_t = 31.6e-12
    short_offset_z0 = 51.9

    load_offset_t = 76.6e-12
    load_offset_z0 = 50.9

    open_reactance = []
    for f in freqs:
        for i in xrange(len(open_c)):
            c = open_c[i]*f**i
        xc = (-1.0j/(2*np.pi*f*c))
        open_reactance.append(tline_input(xc, open_offset_z0, open_offset_t, f))
    open_reactance = np.array(open_reactance)
    open_sparam = gamma(open_reactance, 50)

    o_i = skrf.Network(s=open_sparam, f=freqs, f_unit='Hz')

    short_reactance = tline_input(np.zeros(len(freqs), dtype=np.complex), short_offset_z0, short_offset_t, freqs)
    short_sparam = gamma(short_reactance, 50)

    s_i = skrf.Network(s=short_sparam, f=freqs, f_unit='Hz')
    l_sparam = [gamma(tline_input(50, load_offset_z0, load_offset_t, f), 50) for f in freqs]
    l_i = skrf.Network(s=l_sparam, f=freqs, f_unit='Hz')

    return s_i, o_i, l_i

o = skrf.Network('open.s1p')
s = skrf.Network('short.s1p')
l = skrf.Network('load.s1p')
dut = skrf.Network(sys.argv[1])

freqs = o.f

s_i, o_i, l_i = make_standards()

cal = skrf.OnePort(\
        measured = [o, s, l],
        ideals =[o_i, s_i, l_i]
        )

plt.figure()
dut_cal = cal.apply_cal(dut)
dut_cal.plot_s_db()
plt.show(block=True)

