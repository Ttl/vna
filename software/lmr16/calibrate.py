import sys
import skrf
import matplotlib.pyplot as plt
import numpy as np
skrf.stylely()

os = skrf.Network('open_short.s2p')
so = skrf.Network('short_open.s2p')

ls = skrf.Network('load_short.s2p')
sl = skrf.Network('short_load.s2p')
ss = skrf.Network('short_short.s2p')

ll = skrf.Network('load_load.s2p')
through = skrf.Network('through.s2p')
dut = skrf.Network(sys.argv[1])

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

freqs = through.f

s_i, o_i, l_i = make_standards()

ll_i = skrf.two_port_reflect(l_i, l_i)
os_i = skrf.two_port_reflect(o_i, s_i)
so_i = skrf.two_port_reflect(s_i, o_i)
ss_i = skrf.two_port_reflect(s_i, s_i)
sl_i = skrf.two_port_reflect(s_i, l_i)
ls_i = skrf.two_port_reflect(l_i, s_i)

through_delay = 41.1e-12
d = 2*np.pi*through_delay
through_s = [[[0,np.exp(-1j*d*f)],[np.exp(-1j*d*f),0]] for f in freqs]
through_i = skrf.Network(s=through_s, f=freqs, f_unit='Hz')

cal = None

if 1:
    cal = skrf.LMR16(\
            measured = [through, ll, ss, sl, ls],
            ideal_is_reflect=True,
            ideals = s_i,
            )

    cal.run()

    os = cal.apply_cal(os)
    so = cal.apply_cal(so)
    ll = cal.apply_cal(ll)
    ss = cal.apply_cal(ss)
    ls = cal.apply_cal(ls)
    sl = cal.apply_cal(sl)
    through = cal.apply_cal(through)

    #plt.figure()
    #cal.solved_through.plot_s_deg(m=1, n=0)
    #cal.solved_reflect.plot_s_smith()

if 1:
    if cal != None:
        dut = cal.apply_cal(dut)
    cal = skrf.TwelveTerm(\
            measured = [os, so, ll, through],
            ideals =[os_i, so_i, ll_i, through_i],
            n_thrus = 1,
            )
    cal.run()

if 0:
    cal = skrf.SixteenTerm(\
            measured = [ss, os, so, ll, through],
            ideals = [ss_i, os_i, so_i, ll_i, through_i],
            )

    cal.run()
    coefs = cal.coefs

if 0:
    cal = skrf.EightTerm(\
            measured = [os, so, ll, through],
            ideals = [os_i, so_i, ll_i, through_i]
            )

    cal.run()

plt.figure()
dut = cal.apply_cal(dut)
dut.plot_s_db()
axes = plt.gca()
axes.set_ylim([-60,0])

plt.show(block=True)

