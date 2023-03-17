
# coding: utf-8

# In[20]:

from qick import *
import pandas as pd
import numpy as np

soc = QickSoc()
soccfg = QickConfig(soc.get_cfg())
print(soccfg)
def array_splitter(sequence):
    '''
    input: [ (0, 0, 100), (0, 175, 200) , (1, 50, 150), (1, 175, 250), (2, 75, 125) ] #start and end times for channels 0, 1, 2, ...
    output: [ (1, 0, 50), (3, 50, 75), (7, 75, 100), (6, 100, 125), (2,125, 150), (2, 160, 175), (3, 175, 200), (2,200, 250) ]
    '''
    times = np.array([])
    start_times = []
    end_times = []
    for i in range(0, len(sequence)):
        times = np.append(times, sequence[i][1])
        times = np.append(times, sequence[i][2])

        start_times.append( (sequence[i][0], sequence[i][1])  )
        end_times.append( (sequence[i][0], sequence[i][2] ))
    
    
    times = np.sort(times)
    times = np.unique(times)
    
    sequence_list = []
    out = 0
    for i in range(0, len(times)-1):
        #print(times[i])
        for j in range(0, len(start_times)):
            if times[i] == start_times[j][1]:
                out += 2**(start_times[j][0])
        for k in range(0, len(end_times)):
            if times[i] == end_times[k][1]:
                out -= 2**(start_times[k][0])
                
        sequence_list.append( (out, times[i], times[i+1]))

        #print(i, sequence_list)
    sequence_list.append((0, times[-1], times[-1] + 100))
    return sequence_list
    
    
class LoopbackProgram(AveragerProgram):
        
    def initialize(self):
        cfg=self.cfg   
    
    def body(self):
        
        cfg=self.cfg         
        pulse_list = cfg["pulses"]
        sequence_list = cfg["sequences"]

        
        if len(pulse_list) > 0: 
            res_ch = pulse_list[0][0]
            # set the nyquist zone
            self.declare_gen(ch=res_ch, nqz=1)
            freq = self.freq2reg(pulse_list[0][5] ,gen_ch=res_ch, ro_ch=cfg["ro_chs"][0])
            phase = self.deg2reg(pulse_list[0][6], gen_ch=res_ch)
            gain = pulse_list[0][4]
            style = pulse_list[0][1]
            length= int(round(pulse_list[0][3]*0.384))
            start_time_ns = pulse_list[0][2]
            start_time_clock = int(np.floor(pulse_list[0][2]*0.384))
            for ch in cfg["ro_chs"]:
                self.declare_readout(ch=ch, length=self.cfg["readout_length"],
                                     freq=freq, gen_ch=res_ch)
            if style == "arb":
                x = np.arange(0, 16*length)
                function = pulse_list[0][9]
                self.default_pulse_registers(ch=res_ch, freq=freq, phase=phase, gain=gain)
                I_data = (1/function(16*length))*function(x)
                self.add_pulse(ch = res_ch, name ="wave" + str(0) , idata = gain*I_data) 
                self.set_pulse_registers(ch=res_ch, style=style,waveform = "wave" + str(0),mode = pulse_list[0][7], outsel = pulse_list[0][8])
            else:
                diff = start_time_ns - 2.6*int(np.floor(pulse_list[0][2]*0.384))
                zero_buffer_start = np.zeros(int(round(diff*6.144)))
                ones_buffer = np.ones(int(round(pulse_list[0][3]*6.144)))
                idata = np.concatenate(zero_buffer_start, ones_buffer)
                if len(idata) % 16:
                    idata = np.concatenate(idata, np.zeros(16 - (len(idata) % 16)) )

                self.default_pulse_registers(ch=res_ch, freq=freq, phase=phase, gain=gain)
                self.add_pulse(ch = res_ch, name ="wave" + str(0) , idata = gain*I_data) 
                self.set_pulse_registers(ch=res_ch, style=style,waveform = "wave" + str(0),mode = pulse_list[0][7], outsel = pulse_list[0][8])                
                #self.set_pulse_registers(ch=res_ch, style=style, freq=freq, phase=phase, gain=gain,length=int(round(pulse_list[0][3]*0.384)), mode = pulse_list[0][7])

            self.synci(200)  # give processor some time to configure pulses
            self.pulse(ch=res_ch, t = start_time_clock)
        
        if len(pulse_list) > 0:
            for i in range(1, len(pulse_list)):
                res_ch = pulse_list[i][0]
                self.declare_gen(ch=res_ch, nqz=1)

                freq = self.freq2reg(pulse_list[i][5] ,gen_ch=res_ch, ro_ch=cfg["ro_chs"][0])
                phase = self.deg2reg(pulse_list[i][6], gen_ch=res_ch)
                gain = pulse_list[i][4]
                style = pulse_list[i][1]
                start_time_ns = pulse_list[i][2]
                start_time_clock = int(np.floor(pulse_list[i][2]*0.384))
                length= int(round(pulse_list[i][3]*0.384))
                for ch in cfg["ro_chs"]:
                    self.declare_readout(ch=ch, length=self.cfg["readout_length"],
                                     freq=freq, gen_ch=res_ch)


                if style == "arb":
                    x = np.arange(0, 16*length)
                    function = pulse_list[i][9]
                    
                    #first_half = (1/function(16*length))*function(x)
                    #second_half = np.flip(first_half)
                    I_data = (1/function(16*length))*function(x)
                    self.add_pulse(ch = res_ch, name ="wave" + str(i) , idata = gain*I_data) 
                    self.set_pulse_registers(ch=res_ch, style=style, freq=freq, phase=phase, gain=gain, waveform = "wave" + str(i), mode = pulse_list[i][7], outsel = pulse_list[i][8])
                else:
                    diff = start_time_ns - 2.6*int(np.floor(pulse_list[i][2]*0.384))
                    zero_buffer_start = np.zeros(int(round(diff*6.144)))
                    ones_buffer = np.ones(int(round(pulse_list[i][3]*6.144)))
                    idata = np.concatenate(zero_buffer_start, ones_buffer)
                    if len(idata) % 16:
                        idata = np.concatenate(idata, np.zeros(16 - (len(idata) % 16)) )

                    self.default_pulse_registers(ch=res_ch, freq=freq, phase=phase, gain=gain)
                    self.add_pulse(ch = res_ch, name ="wave" + str(i) , idata = gain*I_data) 
                    self.set_pulse_registers(ch=res_ch, style=style,waveform = "wave" + str(0),mode = pulse_list[i][7], outsel = pulse_list[i][8])    
                self.pulse(ch=res_ch, t = start_time_clock)     

            trig_output = self.soccfg['tprocs'][0]['trig_output'] #usually just 0
        
        for sequence in sequence_list:
            out = sequence[0]
            t_start = int(round(sequence[1]*0.384))
            if out > 0:
                self.regwi(0, 31, out, f'out = 0b{out:>016b}')
                self.seti(trig_output, 0, 31, t_start, f'ch =0 out = ${31} @t = {t_start}')
            else:
                self.seti(trig_output, 0, 0, t_start, f'ch =0 out = 0 @t = {t_start}')
        

        self.wait_all()
        self.sync_all(self.us2cycles(self.cfg["relax_delay"]))