from __future__ import division
from fractions import Fraction
import usb
import time
import lmh2110_cal
import numpy as np
import skrf
from scipy.signal import butter, sosfilt, sosfilt_zi
import pickle
import matplotlib.pyplot as plt
from scipy import stats

class DummyDevice():
    def ctrl_transfer(self, bmRequestType, bRequest, wValue=0, wIndex=0,
            data_or_wLength = None, timeout = None):
        print 'CTRL: rtype: {}, req: {}, value: {}, index: {}'.format(bmRequestType,bRequest, wValue, wIndex)

class MAX2871():
    def __init__(self, usb_command=2, device=None):
        #Register definitions:
        # (Register: name: (First bit, Length, [value]))
        self.register_def = {
                0:{'int':(31, 1), 'n':(15, 16), 'frac':(3, 12)},
                1:{'reserved0':(31, 1), 'cpl':(29, 2), 'cpt': (27, 2), 'p':(15,12), 'm':(3,12)},
                2:{'lds':(31,1), 'sdn':(29,2), 'mux':(26,3), 'dbr':(25,1), 'rdiv2':(24,1), 'r':(14,10),
                    'reg4db':(13,1), 'cp':(9,4), 'ldf':(8,1), 'ldp':(7,1), 'pdp':(6,1), 'shdn':(5,1),
                    'tri':(4,1), 'rst':(3,1)},
                3:{'vco':(26,6), 'vas_shdn':(25,1), 'vas_temp':(24,1), 'reserved1':(19,5), 'csm':(18,1),
                    'mutedel':(17,1), 'cdm':(15,2), 'cdiv':(3,12)},
                4:{'reserved2':(29,3), 'sdldo':(28,1), 'sddiv':(27,1), 'sdref':(26,1), 'bs_msb':(24,2),
                    'fb':(23,1), 'diva':(20,3), 'bs_lsb':(12,8), 'sdvco':(11,1), 'mtld':(10,1),
                    'bdiv':(9,1), 'rfb_en':(8,1), 'bpwr':(6,2), 'rfa_en':(5,1), 'apwr':(3,2)},
                5:{'reserved3':(31,1), 'vas_dly':(29,2), 'reserved4':(26,3), 'sdlo_pll':(25,1),
                    'f01':(24,1), 'ld':(22,2), 'reserved5':(19,3), 'mux3':(18,1), 'reserved6':(7,11),
                    'adcs':(6,1), 'adcm':(3,3)},
                6:{'die_id':(28,4), 'reserved7':(24,4), 'por':(23,1), 'adc':(16,7), 'adcv':(15,1),
                    'reserved8':(10,5), 'vasa':(9,1), 'v':(3,6)}
        }
        self.registers = [0]*8
        self.modified = [False]*8
        self.usb_command = usb_command
        self.device = device

        #Check unique names
        keys = []
        for key in self.register_def.itervalues():
            for r in key:
                if r in keys:
                    raise Exception("Duplicate register {}".format(r))
                keys.append(r)

    def freq_to_regs(self, fout, fpd, m=4095, fb=1, apwr=0, rfa_en=1):
        """Output register values for fout output frequency given
        the phase comparator frequency fpd.
        fb = Counter feedback position. 0 = Divider, 1 = VCO.
        M = Fractional modulus. f = (N + F/M)*fpd """
        #Reg4:reserved2 must be 3
        self.write_value(reserved2=3)
        self.write_value(pdp=1) #Positive phase-detector polarity
        self.write_value(cpl=2) #Charge pump linearity 20%
        self.write_value(cp=12) #Charge pump current. Icp = (1.63/Rset)*(1+cp)
        self.write_value(ld=1) #Digital lock detect pin function
        self.write_value(ldf=0) #Fractional-N lock detect
        self.write_value(r=1) #Reference divide by 1
        self.write_value(p=1) #Phase, recommended 1
        self.write_value(sdn=2) #Low-spur mode 1
        self.write_value(apwr=apwr)
        self.write_value(rfa_en=rfa_en)

        #self.write_value(dbr=1) #R doubler

        if fpd < 32e6:
            self.write_value(lds=0)
        else:
            self.write_value(lds=1)

        self.write_value(mux3=0)
        self.write_value(mux=0) # Tri-state

        if 3e9 <= fout:
            self.write_value(diva=0)
            div = 1
        elif 1.5e9 <= fout:
            self.write_value(diva=1)
            div = 2
        elif 750e6 <= fout:
            self.write_value(diva=2)
            div = 4
        elif 375e6 <= fout:
            self.write_value(diva=3)
            div = 8
        elif 187.5e6 <= fout:
            self.write_value(diva=4)
            div = 16
        elif 93.75e6 <= fout:
            self.write_value(diva=5)
            div = 32
        elif 46.875e6 <= fout:
            self.write_value(diva=6)
            div = 64
        else:
            self.write_value(diva=7)
            div = 128

        fvco = fout*div
        for i in xrange(2):
            d = 1 if fb else min(div, 16)
            n = int((fvco/fpd)/d)
            if 19 <= n <= 4091:
                break
            else:
                fb = int(not fb)

        self.write_value(fb=fb)
        self.write_value(n=n)

        self.write_value(cdiv=int(round(fpd/100e3)))

        bs = int(round(fpd/50e3))
        if bs > 1023:
            bs = 1023
        self.write_value(bs_msb=bs >> 8)
        self.write_value(bs_lsb=bs & 0xFF)

        if 1:
            #Choose best f/m to minimize frequency error
            x = Fraction((fvco-d*fpd*n)/(d*fpd)).limit_denominator(4095)
            f = x.numerator
            m = x.denominator
            if f == 1 and m == 1:
                f = 4094
                m = 4095
        else:
            #Fixed modulus
            f = int(round(m*(fvco-d*fpd*n)/(d*fpd)))

        if m < 2 or m > 4095:
            if f == 0:
                #Integer mode
                #TODO: Set integer mode
                m = 2
            else:
                raise ValueError("Invalid M value {}. 2 <= M <= 4095. f = {}".format(m, f))
        self.write_value(m=m)
        self.write_value(frac=f)

        #print ((n+f/m)*fpd*d)/div,
        #print 'f {} m {} n {} d {} div {} fb {}'.format(f,m,n,d,div,fb)
        #print 'fvco {} fout {}'.format(fvco, fvco/div)
        assert 3e9 <= fvco# <= 6e9
        assert f < m
        assert 19 <= n <= 4091
        fvco = d*fpd*(n+f/m)

        return ((n+f/m)*fpd*d)/div

    def find_reg(self, reg):
        """Finds register by name"""
        for key, val in self.register_def.iteritems():
            if reg in val.keys():
                return key, val[reg]
        return None, None

    def write_value(self, **kw):
        """Write value to register, doesn't update the device"""
        for reg, val in kw.iteritems():
            #print "{} = {}".format(reg, val)
            reg_n, reg_def = self.find_reg(reg)
            if reg_n == None:
                raise ValueError("Register {} not found".format(reg))
            reg_start = reg_def[0]
            reg_len = reg_def[1]
            if val > 2**reg_len or val < 0:
                raise ValueError("Invalid value, got: {}, maximum {}".format(val, reg_len))
            #Clear previous value
            self.registers[reg_n] &= (~((((2**reg_len-1))&0xFFFFFFFF) << reg_start) & 0xFFFFFFFF)
            self.registers[reg_n] |= (val) << reg_start
            self.modified[reg_n] = True
        return

    def to_device(self, device=None):
        if device == None:
            device = self.device
        for i in xrange(5,-1,-1):
            self.write_device(i, self.registers[i], device)

        self.modified = [False]*8

    def write_device(self, control, word, device=None):
        if device == None:
            device = self.device
        if control > 7 or control < 0:
            raise ValueError("Invalid control")
        #print "Write",control,hex(word),(word >> 16) & 0x0000FFFF,(word & 0x0000FFF8) | control
        if device != None:
            return device.ctrl_transfer(0x40, self.usb_command, (word >> 16) & 0x0000FFFF, (word & 0x0000FFF8) | control)
        else:
            print 'No device'

    #FIXME: Add firmware support
    #def read_device(self, device):
    #    """Read from MUXOUT pin of the device, MUXOUT must have been configured
    #    previously"""
    #    word = device.ctrl_transfer(0xC0, ?, 0, 0, 4)
    #    return word


class VNA():
    def __init__(self, lo2_freq=2e6, output_power=0, averages=1, averages_at_low_f=None, dummy=False):

        if dummy:
            self.device = DummyDevice()
        else:
            self.connect()

        self.lo_pll = MAX2871(2, self.device)
        self.source_pll = MAX2871(3, self.device)
        self.source_filter = None

        self.sources_set = False

        #Threshold for extra averaging at low frequencies
        self.low_f = 400e6
        if averages_at_low_f == None:
            self.averages_at_low_f = averages
        else:
            self.averages_at_low_f = averages_at_low_f
        #PLL reference frequency
        self.ref_freq = 19.2e6
        #Microcontroller clock frequency
        self.mcu_clock = 120e6
        #Time to switch the receiver SP4T switch
        self.sp4t_sw_t = 15e-6
        #Mixer output frequency and digital IQ mixing frequency
        self.lo2_freq = lo2_freq
        #ADC sampling frequency
        self.fsample = self.ref_freq/2

        self.output_power = output_power
        self.averages = averages

        self.lo_apwr = 0
        self.source_apwr = 0

        self.power_correction_f = None
        self.power_correction = None
        self.output_power_ref = None

        try:
            with open('vna_p_curve.p', 'r') as f:
                self.power_correction_f, self.power_correction, self.output_power_ref, source_p = pickle.load(f)
        except:
            print "Output power uncalibrated"

        self.set_gpio(signal=True, pa=True, mixer=True)

    def program_sources(self):
        self.lo_pll.to_device()
        self.source_pll.to_device()

    def set_gpio(self, signal=False, pa=False, mixer=False, led=False):
        self.device.ctrl_transfer(0x40, 4, 0, (signal << 0) | (pa << 2) | (mixer << 3) | (led << 4) )

    def clear_gpio(self, signal=False, pa=False, mixer=False, led=False):
        self.device.ctrl_transfer(0x40, 5, 0, (signal << 0) | (pa << 2) | (mixer << 3) | (led << 4) )

    def connect(self):
        self.device = usb.core.find(idVendor=0x1d50, idProduct=0x6099)

        if self.device == None:
            raise Exception("Device not found")
        self.device.set_configuration()

    def write_att(self, att):
        def reverse_bits(x, n=8):
            result = 0
            for i in xrange(n):
                if (x >> i) & 1:
                    result |= 1 << (n - 1 - i)
            return result

        if att > 31.75:
            raise ValueError("Too high attenuation. Max 31.75 dB, got {}".format(att))
        #Bits must be shifted LSB first, but MCU shifts MSB first
        w = reverse_bits(int(round(att/0.25)))
        assert 0 <= w <= 255
        assert w & 0x01 == 0x00
        self.device.ctrl_transfer(0x40, 9, 0, w)

    def select_port(self, port):
        if port == 1:
            self.device.ctrl_transfer(0x40, 8, 0, 0)
        elif port == 2:
            self.device.ctrl_transfer(0x40, 8, 1, 0)
        else:
            raise ValueError("Invalid port number: {}".format(port))

    def read_mcp3021(self):
        res = self.device.ctrl_transfer(0xC0, 19, 0, 0, 2)
        return ((res[0]<<8)|res[1]) >> 2

    def source_power(self, source_freq):
        v = 3.3*self.read_mcp3021()/2**10
        return lmh2110_cal.v_to_dbm(v, source_freq) + 9.54

    def select_filter(self, freq):
        #1: 1.1 - 2.1
        #2: 2.1 - 4.2
        #3: 4.2 - 6.0
        #4: 0 - 1.1
        if freq < 1.1e9:
            f = 4
        elif freq < 2.1e9:
            f = 1
        elif freq < 4.2e9:
            f = 2
        else:
            f = 3
        if f != self.source_filter:
            self.source_filter = f
            self.device.ctrl_transfer(0x40, 6, f, 0)

    def select_mixer_input(self, p):
        choices = {'rx1': 0, 'a': 2, 'rx2': 1, 'b': 3}
        try:
            p = p.lower()
            i = choices[p]
        except (AttributeError, KeyError):
            raise ValueError("Invalid port name {}. Valid names: {}".format(p, choices.keys()))
        self.device.ctrl_transfer(0x40, 7, i, 0)

    def i_to_ch(self, i):
        """Receiver SP4T switch control signal to channel number"""
        return ['rx1','rx2','a','b'][i]

    def assemble_samples(self, x):
        y = []
        for i in xrange(len(x)//2):
            y.append((x[2*i+1]<<8)+x[2*i])
        return y

    def mag_ang(self, x):
        """Complex number to dB and angle in degrees"""
        return 20*np.log10(np.abs(x)),np.arctan2(x.imag, x.real)*180/np.pi

    def butter_bandpass(self, lowcut, highcut, order=5):
        nyq = 0.5 * self.fsample
        low = lowcut / nyq
        high = highcut / nyq
        sos = butter(order, [low, high], btype='bandpass', output='sos')
        return sos

    def butter_bandpass_filter(self, data, lowcut, highcut, order=5):
        sos = self.butter_bandpass(lowcut, highcut, order=order)
        y = sosfilt(sos, data)
        return y

    def butter_lowpass(self, highcut, order=5):
        nyq = 0.5 * self.fsample
        high = highcut / nyq
        sos = butter(order, high, btype='lowpass', output='sos')
        return sos

    def butter_lowpass_filter(self, data, highcut, order=5, init=0):
        sos = self.butter_lowpass(highcut, order=order)
        zi = sosfilt_zi(sos)
        y = sosfilt(sos, data, zi=init*zi)[0]
        return y

    def set_output_power(self, freq, power):
        freq_i = np.searchsorted(self.power_correction_f, freq)
        freq_i = min(len(self.output_power_ref)-1, freq_i)
        ref_power = self.output_power_ref[freq_i]
        att = ref_power - power
        if att < 0:
            att = 0
        self.write_att(att)

    def sample(self, ports, source_port, all_channels=None):

        def ch_to_i(x):
            return {'rx1': 0, 'a': 2, 'rx2': 1, 'b': 3}[x]

        samples = 16384

        if all_channels == False and ports == [1, 2]:
            if source_port == 1:
                channels = map(ch_to_i, ['rx1', 'a', 'b'])
            else:
                channels = map(ch_to_i, ['rx2', 'a', 'b'])
        else:
            if ports == [1]:
                channels = map(ch_to_i, ['rx1', 'a'])
            if ports == [2]:
                channels = map(ch_to_i, ['rx2', 'b'])
            if ports == [1, 2]:
                if source_port == 1:
                    channels = map(ch_to_i, ['rx1', 'a', 'b', 'rx2'])
                else:
                    channels = map(ch_to_i, ['rx1', 'a', 'b', 'rx2'])

        #Equal delay = MCU clock*(samples/f_fsample)/channels)
        delays = [int(self.mcu_clock*(samples/self.fsample)/len(channels))]*len(channels)

        switch = [int(d*self.fsample/self.mcu_clock) for d in delays]
        switch = np.cumsum(switch)

        samples_delays1 = [0]
        samples_delays2 = []
        for s in switch[:-1]:
            samples_delays2.append(s)
            samples_delays1.append(s + int(self.sp4t_sw_t*self.fsample))
        samples_delays2.append(samples)

        sample_delays = zip(samples_delays1, samples_delays2)

        delays8 = []
        for d in delays:
            delays8.append((d >> 0*8) & 0xFF)
            delays8.append((d >> 1*8) & 0xFF)
            delays8.append((d >> 2*8) & 0xFF)
            delays8.append((d >> 3*8) & 0xFF)

        channel_word = 0x0F

        for ch in channels[::-1]:
            channel_word = (channel_word << 4) | ch
        channel_word = channel_word & (2**32 -1)

        self.device.ctrl_transfer(0x40, 20, channel_word >> 16, channel_word & 0xFFFF, ''.join(map(chr, delays8)))

        return map(self.i_to_ch, channels), sample_delays

    def measure_iq(self, freqs, ports=[1,2], all_channels=True):
        iqs = []

        for port in ports:
            self.select_port(port)
            for e,freq in enumerate(freqs):
                self.set_output_power(freq, self.output_power)
                iqs.append({})
                source_freq = freq
                lo_freq = source_freq-self.lo2_freq
                self.select_filter(source_freq)
                real_lo_f = self.lo_pll.freq_to_regs(lo_freq, self.ref_freq, apwr=self.lo_apwr)
                real_source_f = self.source_pll.freq_to_regs(source_freq, self.ref_freq, apwr=self.source_apwr)
                lo2_f = real_source_f - real_lo_f

                self.program_sources()
                if not self.sources_set:
                    #Program twice the first time
                    time.sleep(20e-3)
                    self.program_sources()
                    self.sources_set = True

                if freq < self.low_f:
                    averages = self.averages_at_low_f
                else:
                    averages = self.averages

                #Wait for PLLs to lock
                #Hardware lock output doesn't seem to be accurate enough
                time.sleep(0.5e-3)

                for a in xrange(averages):

                    channels, delays = self.sample(ports, port, all_channels)

                    data = self.device.read(0x81, 32768)
                    y = np.array(self.assemble_samples(data))
                    t = np.linspace(0,(len(y)-1)/self.fsample, len(y))

                    #f = [self.fsample*i/(len(y)) for i in xrange(len(y)//2+1)]
                    #w = np.hanning(len(y))
                    #plt.plot(f, 20*np.log10(1.0/2**10*1.0/len(y)*np.abs(np.fft.rfft(w*y))))
                    #plt.figure()
                    #plt.plot(y)
                    #plt.show()

                    lo_i = np.cos(-2*np.pi*lo2_f*t)
                    lo_q = np.sin(-2*np.pi*lo2_f*t)

                    #Digital IQ mixing
                    for i in xrange(len(delays)):
                        s_start, s_end = delays[i]
                        x = y[s_start:s_end]
                        x = x-np.mean(x) #Subtract DC

                        #x = self.butter_bandpass_filter(x, 0.8*self.lo2_freq, 1.1*self.lo2_freq, order=5)
                        #f = [self.fsample*j/(len(x)) for j in xrange(len(x)//2+1)]
                        #w = np.hanning(len(x))
                        #plt.plot(f, 20*np.log10(1.0/2**10*1.0/len(y)*np.abs(np.fft.rfft(w*x))))
                        #plt.plot(x)
                        #plt.show()

                        iqr = (lo_i[s_start:s_end])*x
                        iqi = (lo_q[s_start:s_end])*x

                        iqr = self.butter_lowpass_filter(iqr, 0.5e6, order=2, init=np.mean(iqr))
                        iqi = self.butter_lowpass_filter(iqi, 0.5e6, order=2, init=np.mean(iqi))

                        iq = iqr+1j*iqi

                        #plt.figure()
                        #plt.plot(np.real(iq))
                        #plt.plot(np.imag(iq))
                        #plt.show()

                        #plt.figure()
                        #plt.plot(np.cumsum(np.real(iq))/range(len(iq)))
                        #plt.plot(np.cumsum(np.imag(iq))/range(len(iq)))
                        #plt.show()

                        iq = np.mean(iq)

                        if a == 0:
                            iqs[e][(channels[i],port)] = [iq]
                        else:
                            iqs[e][(channels[i],port)].append(iq)

                if port == 1:
                    norm = np.angle(iqs[e][('rx1',1)])
                else:
                    norm = np.angle(iqs[e][('rx2',2)])

                chs = [k for k in iqs[e].keys() if k[1] == port]
                for ch in chs:
                    z = 0
                    for j, a in enumerate(iqs[e][ch]):
                        z += a*np.exp(-1j*norm[j])
                    iqs[e][ch] = z/len(iqs[e][ch])

                meas = [iqs[e][k] for k in chs]
                chs, meas = zip(*sorted(zip(chs, meas)))
                for i in xrange(len(chs)):
                    print '{} {}: {:5.1f} {:6.1f}'.format(port, chs[i][0], 20*np.log10(np.abs(meas[i])), np.angle(meas[i])*180/np.pi) ,
                print

        return iqs

    def iq_to_sparam(self, iqs, freqs):
        sparams = []
        if len(iqs[0]) == 2:
            ports = [iqs[0].keys()[0][1]]
        else:
            ports = [1,2]


        if len(ports) == 1:
            for f in xrange(len(freqs)):
                if ports[0] == 1:
                    s = iqs[f][('a',1)]/iqs[f][('rx1',1)]
                else:
                    s = iqs[f][('b',2)]/iqs[f][('rx2',2)]
                sparams.append(s)
        elif len(ports) == 2:
            for f in xrange(len(freqs)):
                s11 = []
                s12 = []
                s21 = []
                s22 = []
                #Switch correction
                D = 1.0 - (iqs[f][('rx2',1)]/iqs[f][('rx1',1)])*(iqs[f][('rx1',2)]/iqs[f][('rx2',2)])
                sm11 = (1.0/D)*( iqs[f][('a',1)]/iqs[f][('rx1',1)] - (iqs[f][('a',2)]/iqs[f][('rx2',2)])*(iqs[f][('rx2',1)]/iqs[f][('rx1',1)]) )
                sm12 = (1.0/D)*( iqs[f][('a',2)]/iqs[f][('rx2',2)] - (iqs[f][('a',1)]/iqs[f][('rx1',1)])*(iqs[f][('rx1',2)]/iqs[f][('rx2',2)]) )
                sm21 = (1.0/D)*( iqs[f][('b',1)]/iqs[f][('rx1',1)] - (iqs[f][('b',2)]/iqs[f][('rx2',2)])*(iqs[f][('rx2',1)]/iqs[f][('rx1',1)]) )
                sm22 = (1.0/D)*( iqs[f][('b',2)]/iqs[f][('rx2',2)] - (iqs[f][('b',1)]/iqs[f][('rx1',1)])*(iqs[f][('rx1',2)]/iqs[f][('rx2',2)]) )
                s11.append(sm11)
                s12.append(sm12)
                s21.append(sm21)
                s22.append(sm22)
                sparams.append( [[np.mean(s11), np.mean(s12)], [np.mean(s21), np.mean(s22)]] )

        return skrf.Network(s=sparams, f=freqs, f_unit='Hz')

    def measure(self, freqs, ports=[1,2]):
        return self.iq_to_sparam(self.measure_iq(freqs, ports), freqs)


if __name__ == "__main__":

    vna = VNA(lo2_freq=2e6, output_power=-5, averages=1, averages_at_low_f=1)
    freqs = np.linspace(26e6, 6.2e9, 400)
    ports = [1, 2]

    iqs = vna.measure_iq(freqs, ports, True)
    pickle.dump((freqs,iqs), open('iqs', 'w'))
    net = vna.iq_to_sparam(iqs, freqs)

    if len(ports) == 2:
        net.write_touchstone('response.s2p')
    else:
        net.write_touchstone('response.s1p')
    net.plot_s_db()
    plt.show()
