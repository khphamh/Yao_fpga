#####################################################################
#                                                                   #
# /ZCU4.py                                                  #
#                                                                   #
# Copyright 2013, Monash University                                 #
#                                                                   #
# This file is part of the module labscript_devices, in the         #
# labscript suite (see http://labscriptsuite.org), and is           #
# licensed under the Simplified BSD License. See the license.txt    #
# file in the root of the project for the full license.             #
#                                                                   #
#####################################################################
from labscript_devices import runviewer_parser, labscript_device, BLACS_tab, BLACS_worker

from labscript import Device, IntermediateDevice, Pseudoclock, ClockLine, PseudoclockDevice, config, LabscriptError, StaticAnalogQuantity, AnalogOut, DigitalOut, set_passed_properties, WaitMonitor, compiler, DDS, DDSQuantity
import copy

import numpy as np
import labscript_utils.h5_lock, h5py
from ctypes import *
import struct
import serial
import time
max_deviations = [
    {'low': 0.1,        'high': 249.999,    'dev': 10.0},
    {'low': 249.999,    'high': 500.0,      'dev': 5.0},
    {'low': 500.0,      'high': 1000.0,     'dev': 10.0},
    {'low': 1000.0,     'high': 2000.0,     'dev': 20.0},
    {'low': 2000.0,     'high': 4000.0,     'dev': 40.0}
]


class ZCU4DDS(DDSQuantity):
    description = 'ZCU4DDS'
    def __init__(self, *args, **kwargs):
        if 'call_parents_add_device' in kwargs:
            call_parents_add_device = kwargs['call_parents_add_device']
        else:
            call_parents_add_device = True

        kwargs['call_parents_add_device'] = False
        DDSQuantity.__init__(self, *args, **kwargs)

        self.gate = DigitalQuantity(self.name + '_gate', self, 'gate')
        self.phase_reset = DigitalQuantity(self.name + '_phase_reset', self, 'phase_reset')

        if call_parents_add_device:
            self.parent_device.add_device(self)

    def hold_phase(self, t):
        self.phase_reset.go_high(t)

    def release_phase(self, t):
        self.phase_reset.go_low(t)


profiles = {}
def profile(funct):
    func = funct.__name__
    if func not in profiles:
        profiles[func] = {'total_time':0, 'min':None, 'max':0, 'num_calls':0, 'average_time_per_call':0}
    
    def new_func(*args,**kwargs):
        start_time = time.time()
        ret = funct(*args,**kwargs)
        runtime = time.time()-start_time
        profiles[func]['total_time'] += runtime
        profiles[func]['num_calls'] += 1
        profiles[func]['min'] = profiles[func]['min'] if profiles[func]['min'] is not None and profiles[func]['min'] < runtime else runtime
        profiles[func]['max'] = profiles[func]['max'] if profiles[func]['max'] > runtime else runtime
        profiles[func]['average_time_per_call'] = profiles[func]['total_time']/profiles[func]['num_calls']
        
        return ret
    # return new_func
    return funct
    
def start_profile(name):
    if name not in profiles:
        profiles[name] = {'total_time':0, 'min':None, 'max':0, 'num_calls':0, 'average_time_per_call':0}
        
    if 'start_time' in profiles[name]:
        raise Exception('You cannot call start_profile for %s without first calling stop_profile'%name)
        
    profiles[name]['start_time'] = time.time()
    
def stop_profile(name):
    if name not in profiles or 'start_time' not in profiles[name]:
        raise Exception('You must first call start_profile for %s before calling stop_profile')
        
    runtime = time.time()-profiles[name]['start_time']
    del profiles[name]['start_time']
    profiles[name]['total_time'] += runtime
    profiles[name]['num_calls'] += 1
    profiles[name]['min'] = profiles[name]['min'] if profiles[name]['min'] is not None and profiles[name]['min'] < runtime else runtime
    profiles[name]['max'] = profiles[name]['max'] if profiles[name]['max'] > runtime else runtime
    profiles[name]['average_time_per_call'] = profiles[name]['total_time']/profiles[name]['num_calls']


class ZCU4(PseudoclockDevice):
    pb_instructions = {'CONTINUE':   0,
                       'STOP':       1, 
                       'LOOP':       2, 
                       'END_LOOP':   3,
                       'BRANCH':     6,
                       'LONG_DELAY': 7,
                       'WAIT':       8}

                       
    description = 'PB-DDSII-300'
    clock_limit = 8.3e6 # Slight underestimate I think.
    clock_resolution = 26.6666666666666666e-9
    # TODO: Add n_dds and generalise code
    n_flags = 8
    
    core_clock_freq = 75 # MHz
    # This value is coupled to a value in the ZCU4 worker process of BLACS
    # This number was found experimentally but is determined theoretically by the
    # instruction lengths in BLACS, and a finite delay in the ZCU4
    #
    # IF YOU CHANGE ONE, YOU MUST CHANGE THE OTHER!
    trigger_delay = 250e-9 
    wait_delay = 100e-9
    trigger_edge_type = 'falling'
    
    # This device can only have Pseudoclock children (digital outs and DDS outputs should be connected to a child device)
    allowed_children = [Pseudoclock, DigitalOut]
    
    @set_passed_properties(
        property_names = {"connection_table_properties": ["com_port", ]}
        )
    def __init__(self, name, trigger_device=None, trigger_connection=None, com_port="COM7", **kwargs):
        PseudoclockDevice.__init__(self, name, trigger_device, trigger_connection, **kwargs)
        self.BLACS_connection = com_port
        self.min_delay = 0.5/self.clock_limit
        self.long_delay = 2**32 / (self.core_clock_freq * 1e6) - self.min_delay
        # Create the internal pseudoclock
        self._pseudoclock = Pseudoclock('%s_pseudoclock'%name, self, 'clock') # possibly a better connection name than 'clock'?
        # Create the internal direct output clock_line
        self._direct_output_clock_line = ClockLine('%s_direct_output_clock_line'%name, self.pseudoclock, 'internal', ramping_allowed = False)
        # Create the internal intermediate device connected to the above clock line
        # This will have the direct DigitalOuts of DDSs of the ZCU4 connected to it
        self._direct_output_device = ZCU4DirectOutputs('%s_direct_output_device'%name, self._direct_output_clock_line)
    
    @property
    def pseudoclock(self):
        return self._pseudoclock
        
    @property
    def direct_outputs(self):
        return self._direct_output_device
    
    def add_device(self, device):
        if not self.child_devices and isinstance(device, Pseudoclock):
            PseudoclockDevice.add_device(self, device)
            
        elif isinstance(device, Pseudoclock):
            raise LabscriptError('The %s %s automatically creates a Pseudoclock because it only supports one. '%(self.description, self.name) +
                                 'Instead of instantiating your own Pseudoclock object, please use the internal' +
                                 ' one stored in %s.pseudoclock'%self.name)
        elif isinstance(device, DDS) or isinstance(device, ZCU4DDS) or isinstance(device, DigitalOut):
            #TODO: Defensive programming: device.name may not exist!
            raise LabscriptError('You have connected %s directly to %s, which is not allowed. You should instead specify the parent_device of %s as %s.direct_outputs'%(device.name, self.name, device.name, self.name))
        else:
            raise LabscriptError('You have connected %s (class %s) to %s, but %s does not support children with that class.'%(device.name, device.__class__, self.name, self.name))
                
    def flag_valid(self, flag):
        if -1 < flag < self.n_flags:
            return True
        return False     
        
    def flag_is_clock(self, flag):
        for clock_line in self.pseudoclock.child_devices:
            if clock_line.connection == 'internal': #ignore internal clockline
                continue
            if flag == self.get_flag_number(clock_line.connection):
                return True
        return False
            
    def get_flag_number(self, connection):
        # TODO: Error checking
        prefix, connection = connection.split()
        return int(connection)


    def get_direct_outputs(self):
        """Finds out which outputs are directly attached to the ZCU4"""
        dig_outputs = []
        dds_outputs = []
        for output in self.direct_outputs.get_all_outputs():
            # If we are a child of a DDS
            if isinstance(output.parent_device, DDS) or isinstance(output.parent_device, ZCU4DDS):
                # and that DDS has not been processed yet
                if output.parent_device not in dds_outputs:
                    # process the DDS instead of the child
                    output = output.parent_device
                else:
                    # ignore the child
                    continue
            
            # only check DDS and DigitalOuts (so ignore the children of the DDS)
            if isinstance(output,DDS) or isinstance(output,ZCU4DDS) or isinstance(output, DigitalOut):
                # get connection number and prefix
                try:
                    prefix, connection = output.connection.split()
                    assert prefix == 'flag' or prefix == 'dds'
                    connection = int(connection)
                except:
                    raise LabscriptError('%s %s has invalid connection string: \'%s\'. '%(output.description,output.name,str(output.connection)) + 
                                         'Format must be \'flag n\' with n an integer less than %d, or \'dds n\' with n less than 2.'%self.n_flags)
                # run checks on the connection string to make sure it is valid
                # TODO: Most of this should be done in add_device() No?
                if prefix == 'flag' and not self.flag_valid(connection):
                    raise LabscriptError('%s is set as connected to flag %d of %s. '%(output.name, connection, self.name) +
                                         'Output flag number must be a integer from 0 to %d.'%(self.n_flags-1))
                if prefix == 'flag' and self.flag_is_clock(connection):
                    raise LabscriptError('%s is set as connected to flag %d of %s.'%(output.name, connection, self.name) +
                                         ' This flag is already in use as one of the ZCU4\'s clock flags.')                         
                if prefix == 'dds' and not connection < 2:
                    raise LabscriptError('%s is set as connected to output connection %d of %s. '%(output.name, connection, self.name) +
                                         'DDS output connection number must be a integer less than 2.')
                
                # Check that the connection string doesn't conflict with another output
                for other_output in dig_outputs + dds_outputs:
                    if output.connection == other_output.connection:
                        raise LabscriptError('%s and %s are both set as connected to %s of %s.'%(output.name, other_output.name, output.connection, self.name))
                
                # store a reference to the output
                if isinstance(output, DigitalOut):
                    dig_outputs.append(output)
                elif isinstance(output, DDS) or isinstance(output, ZCU4DDS):
                    dds_outputs.append(output)
                
        return dig_outputs, dds_outputs
        
    def convert_to_pb_inst(self, dig_outputs, dds_outputs):
        pb_inst = []
        
        # index to keep track of where in output.raw_output the
        # ZCU4 flags are coming from
        # starts at -1 because the internal flag should always tick on the first instruction and be 
        # incremented (to 0) before it is used to index any arrays
        i = -1 
        # index to record what line number of the ZCU4 hardware
        # instructions we're up to:
        j = 0
        # We've delegated the initial two instructions off to BLACS, which
        # can ensure continuity with the state of the front panel. Thus
        # these two instructions don't actually do anything:
        flags = [0]*self.n_flags
        dds_enables = [0]*2
        
        pb_inst.append({'enables':dds_enables,
                        'flags': ''.join([str(flag) for flag in flags]), 'instruction':'STOP',
                        'data': 0, 'delay': 10.0/self.clock_limit*1e9})
        pb_inst.append({'enables':dds_enables,
                        'flags': ''.join([str(flag) for flag in flags]),'instruction':'STOP',
                        'data': 0, 'delay': 10.0/self.clock_limit*1e9})
        j += 2
        
        flagstring = '0'*self.n_flags # So that this variable is still defined if the for loop has no iterations
        wanted_instruction_list = []
        for k, instruction in enumerate(self.pseudoclock.clock):
            wanted_instruction_list.append(instruction)
            if instruction == 'WAIT':
                # This is a wait instruction. Repeat the last instruction but with a 100ns delay and a WAIT op code:
                wait_instruction = pb_inst[-1].copy()
                wait_instruction['delay'] = 100
                wait_instruction['instruction'] = 'WAIT'
                wait_instruction['data'] = 0
                pb_inst.append(wait_instruction)
                j += 1
                continue
                
            flags = [0]*self.n_flags
            # The registers below are ones, not zeros, so that we don't
            # use the BLACS-inserted initial instructions. Instead
            # unused DDSs have a 'zero' in register one for freq, amp
            # and phase.
            dds_enables = [0]*2
            
            # This flag indicates whether we need a full clock tick, or are just updating an internal output
            only_internal = True
            # find out which clock flags are ticking during this instruction
            for clock_line in instruction['enabled_clocks']:
                if clock_line == self._direct_output_clock_line: 
                    # advance i (the index keeping track of internal clockline output)
                    i += 1
                else:
                    flag_index = int(clock_line.connection.split()[1])
                    flags[flag_index] = 1
                    # We are not just using the internal clock line
                    only_internal = False
            
            for output in dig_outputs:
                flagindex = int(output.connection.split()[1])
                flags[flagindex] = int(output.raw_output[i])
            for output in dds_outputs:
                ddsnumber = int(output.connection.split()[1])
                dds_enables[ddsnumber] = output.gate.raw_output[i]
                
            flagstring = ''.join([str(flag) for flag in flags])
            
            if not only_internal:
                high_time = instruction['step']/2
                high_time = min(high_time, self.long_delay)

                # Low time is whatever is left:
                low_time = instruction['step'] - high_time

                # Do we need to insert a LONG_DELAY instruction to create a delay this
                # long?
                n_long_delays, remaining_low_time =  divmod(low_time, self.long_delay)

                # If the remainder is too short to be output, add self.long_delay to it.
                # self.long_delay was constructed such that adding self.min_delay to it
                # is still not too long for a single instruction:
                if n_long_delays and remaining_low_time < self.min_delay:
                    n_long_delays -= 1
                    remaining_low_time += self.long_delay

                # The start loop instruction, Clock edges are high:
                pb_inst.append({'enables':dds_enables,
                                'flags': flagstring, 'instruction': 'LOOP',
                                'data': instruction['reps'], 'delay': high_time*1e9})
                
                for clock_line in instruction['enabled_clocks']:
                    if clock_line != self._direct_output_clock_line:
                        flag_index = int(clock_line.connection.split()[1])
                        flags[flag_index] = 0
                        
                flagstring = ''.join([str(flag) for flag in flags])
            
                # The long delay instruction, if any. Clock edges are low: 
                if n_long_delays:
                    pb_inst.append({'enables':dds_enables,
                                'flags': flagstring, 'instruction': 'LONG_DELAY',
                                'data': int(n_long_delays), 'delay': self.long_delay*1e9})
                                
                # Remaining low time. Clock edges are low:
                pb_inst.append({'enables':dds_enables, 
                                'flags': flagstring, 'instruction': 'END_LOOP',
                                'data': j, 'delay': remaining_low_time*1e9})
                                
                # Two instructions were used in the case of there being no LONG_DELAY, 
                # otherwise three. This increment is done here so that the j referred
                # to in the previous line still refers to the LOOP instruction.
                j += 3 if n_long_delays else 2
            else:
                # We only need to update a direct output, so no need to tick the clocks.

                # Do we need to insert a LONG_DELAY instruction to create a delay this
                # long?
                n_long_delays, remaining_delay =  divmod(instruction['step'], self.long_delay)
                # If the remainder is too short to be output, add self.long_delay to it.
                # self.long_delay was constructed such that adding self.min_delay to it
                # is still not too long for a single instruction:
                if n_long_delays and remaining_delay < self.min_delay:
                    n_long_delays -= 1
                    remaining_delay += self.long_delay
                
                if n_long_delays:
                    pb_inst.append({'enables':dds_enables,
                                'flags': flagstring, 'instruction': 'LONG_DELAY',
                                'data': int(n_long_delays), 'delay': self.long_delay*1e9})

                pb_inst.append({'enables':dds_enables,
                                'flags': flagstring, 'instruction': 'CONTINUE',
                                'data': 0, 'delay': remaining_delay*1e9})
                
                j += 2 if n_long_delays else 1
                
            
        raise LabscriptError(str(wanted_instruction_list))
        return pb_inst
        
    def write_pb_inst_to_h5(self, pb_inst, hdf5_file):
        # OK now we squeeze the instructions into a numpy array ready for writing to hdf5:
        pb_dtype = [('dds_en0', np.int32),('dds_en1', np.int32), 
                    ('flags', np.int32), ('inst', np.int32),
                    ('inst_data', np.int32), ('length', np.float64)]
        pb_inst_table = np.empty(len(pb_inst),dtype = pb_dtype)
        for i,inst in enumerate(pb_inst):
            flagint = int(inst['flags'][::-1],2)
            instructionint = self.pb_instructions[inst['instruction']]
            dataint = inst['data']
            delaydouble = inst['delay']
            en0 = inst['enables'][0]
            en1 = inst['enables'][1]
            
            pb_inst_table[i] = (en0,en1, flagint, 
                                instructionint, dataint, delaydouble)     
                                
        # Okay now write it to the file: 
        group = hdf5_file['/devices/'+self.name]  
        group.create_dataset('PULSE_PROGRAM', compression=config.compression,data = pb_inst_table)   
        self.set_property('stop_time', self.stop_time, location='device_properties')


    def _check_wait_monitor_ok(self):
        if (
            compiler.master_pseudoclock is self
            and compiler.wait_table
            and compiler.wait_monitor is None
            and self.programming_scheme != 'pb_stop_programming/STOP'
        ):
            msg = """If using waits without a wait monitor, the ZCU4 used as a
                master pseudoclock must have
                programming_scheme='pb_stop_programming/STOP'. Otherwise there is no way
                for BLACS to distinguish between a wait, and the end of a shot. Either
                use a wait monitor (see labscript.WaitMonitor for details) or set
                programming_scheme='pb_stop_programming/STOP for %s."""
            raise LabscriptError(dedent(msg) % self.name)

    def generate_code(self, hdf5_file):
        # Generate the hardware instructions
        hdf5_file.create_group('/devices/' + self.name)
        PseudoclockDevice.generate_code(self, hdf5_file)
        dig_outputs, dds_outputs = self.get_direct_outputs()
        pb_inst = self.convert_to_pb_inst(dig_outputs, dds_outputs)
        self._check_wait_monitor_ok()
        self.write_pb_inst_to_h5(pb_inst, hdf5_file)


class ZCU4DirectOutputs(IntermediateDevice):
    allowed_children = [DDS, ZCU4DDS, DigitalOut]
    clock_limit = ZCU4.clock_limit
    description = 'PB-DDSII-300 Direct Outputs'
  
    def add_device(self, device):
        IntermediateDevice.add_device(self, device)
        if isinstance(device, DDS):
            # Check that the user has not specified another digital line as the gate for this DDS, that doesn't make sense.
            # Then instantiate a DigitalQuantity to keep track of gating.
            if device.gate is None:
                device.gate = DigitalQuantity(device.name + '_gate', device, 'gate')
            else:
                raise LabscriptError('You cannot specify a digital gate ' +
                                     'for a DDS connected to %s. '% (self.name) + 
                                     'The digital gate is always internal to the ZCU4.')

import time

from blacs.tab_base_classes import Worker, define_state
from blacs.tab_base_classes import MODE_MANUAL, MODE_TRANSITION_TO_BUFFERED, MODE_TRANSITION_TO_MANUAL, MODE_BUFFERED

from blacs.device_base_class import DeviceTab

@BLACS_tab
class ZCU4Tab(DeviceTab):
    def initialise_GUI(self):
        # Capabilities
        self.base_units =    {'freq':'MHz',         'amp':'dBm',   'phase':'Degrees', 'length': "ns"}
        self.base_min =      {'freq':0,           'amp':-136.0,  'phase':0, 'length': 0}
        self.base_max =      {'freq':4000.,         'amp':25.0,    'phase':360, 'length':10000}
        self.base_step =     {'freq':1.0,           'amp':1.0,     'phase':1, 'length':1}
        self.base_decimals = {'freq':4,             'amp':4,       'phase':3, 'length':3} # TODO: find out what the phase precision is!
        self.num_DO = 8

        # Create DDS Output objects
        RF_prop = {}
        for i in range(7):
            RF_prop['channel '+str(i)] = {}
            for subchnl in ['freq', 'amp', 'phase', 'length']:
                RF_prop['channel '+str(i)][subchnl] = {'base_unit':self.base_units[subchnl],
                                                    'min':self.base_min[subchnl],
                                                    'max':self.base_max[subchnl],
                                                    'step':self.base_step[subchnl],
                                                    'decimals':self.base_decimals[subchnl]
                                                    }
        do_prop = {}
        for i in range(self.num_DO): 
            do_prop['flag %d'%i] = {}
        
        # Create the output objects    
        self.create_digital_outputs(do_prop)        
        # Create widgets for output objects
        
        # Define the sort function for the digital outputs
        def sort(channel):
            flag = channel.replace('flag ','')
            flag = int(flag)
            return '%02d'%(flag)

        # Create the output objects
        self.create_dds_outputs(RF_prop)

        # Create widgets for output objects
        dds_widgets,ao_widgets,do_widgets = self.auto_create_widgets()
        # and auto place the widgets in the UI
        self.auto_place_widgets(("RF Output",dds_widgets) ,("Flags",do_widgets,sort))

        # Store the COM port to be used
        self.com_port = str(self.settings['connection_table'].find_by_name(self.device_name).BLACS_connection)

        # Create and set the primary worker
        self.create_worker("main_worker", ZCU4Worker, {'com_port':self.com_port})
        self.primary_worker = "main_worker"

        # Set the capabilities of this device
        self.supports_remote_value_check(False) # !!!
        self.supports_smart_programming(False) # !!!

@BLACS_worker
class ZCU4Worker(Worker):

    def init(self):

        global h5py; import labscript_utils.h5_lock, h5py

        self.COMPort = self.com_port
        self.baudrate = 115200

        self.final_values = {}

        self.smart_cache = {'RF_DATA': None,
                            'SWEEP_DATA': None}

        ZCU4ser = serial.Serial(self.COMPort, baudrate=self.baudrate, timeout=1)

        if(ZCU4ser.isOpen() == False):
            ZCU4ser.open()
        '''
        while True:
            if ZCU4ser.inWaiting() > 0:
                break
        time.sleep(30)
        '''
        ZCU4ser.write(b"cd jupyter_notebooks\r\n")
        time.sleep(1)

        ZCU4ser.write(b"cd qick\r\n")
        time.sleep(1)
        
        ZCU4ser.write(b"cd qick_demos\r\n")
        time.sleep(1)

        ZCU4ser.write(b"sudo python3\r\n")
        time.sleep(1)

        ZCU4ser.write(b"xilinx\r\n")
        time.sleep(1)

        ZCU4ser.write(b"exec(open('initialize.py').read())\r\n")
        time.sleep(1)

        ZCU4ser.close()

    def check_remote_values(self):
        results = {}
        for i in range(7):
            results['channel '+str(i)]=  {}
        self.final_values = {}

        ZCU4ser = serial.Serial(self.COMPort, baudrate=self.baudrate, timeout=1)
        if(ZCU4ser.isOpen() == False):
            ZCU4ser.open()


        for i in range(7):
            results['channel '+str(i)]['freq'] = 0
            results['channel '+str(i)]['amp'] = 0
            results['channel '+str(i)]['phase'] = 0
            results['channel '+str(i)]['length'] = 0


        return results


    def program_manual(self,front_panel_values):

        ZCU4ser = serial.Serial(self.COMPort, baudrate=self.baudrate, timeout=1)
        if(ZCU4ser.isOpen() == False):
            ZCU4ser.open()

        for i in range(7):
            values = front_panel_values['channel ' + str(i)]

        sequence_list = []
        for i in range(8):
            if front_panel_values['flag %d'%i]:
                sequence_list.append((i, 0, 10**4))
                #raise LabscriptError(str(sequence_list) + " attempt to use digital output")

        pulse_list = [[6, 'const', 0, 100, 30000, 100, 0, 'oneshot', 'product', '[]']]
        #pulse_list = []
        pulse_list_string = "pulse_list = " + str(pulse_list) + "\r\n"
        ZCU4ser.write(pulse_list_string.encode())
        time.sleep(1)
        sequence_list_string = "sequence_list = " + str(sequence_list) + "\r\n"
        ZCU4ser.write(sequence_list_string.encode())
        time.sleep(1)

        ZCU4ser.write(b"exec(open('send_pulse.py').read())\r\n")
        time.sleep(1)

        ZCU4ser.close()

        # Now that a manual update has been done, we'd better invalidate the saved RF_DATA:
        self.smart_cache['RF_DATA'] = None

        return self.check_remote_values()
        
    def transition_to_buffered(self,device_name,h5file,initial_values,fresh):
        self.h5file = h5file

        with h5py.File(h5file,'r') as hdf5_file:
            group = hdf5_file['devices/%s'%device_name]
            
            # Is this shot using the fixed-duration workaround instead of checking the ZCU4's status?
            self.time_based_stop_workaround = group.attrs.get('time_based_stop_workaround', False)
            if self.time_based_stop_workaround:
                self.time_based_shot_duration = (group.attrs['stop_time']
                                                 + hdf5_file['waits'][:]['timeout'].sum()
                                                 + group.attrs['time_based_stop_workaround_extra_time'])
                
            # Now for the pulse program:
            pulse_program = group['PULSE_PROGRAM'][2:]
            
            #Let's get the final state of the ZCU4. z's are the args we don't need:
            en0,en1,flags,z,z,z = pulse_program[-1]

            
            # Always call start_programming regardless of whether we are going to do any
            # programming or not. This is so that is the programming_scheme is 'pb_stop_programming/STOP'
            # we are ready to be triggered by a call to pb_stop_programming() even if no programming
            # occurred due to smart programming:
            wanted_string = ''
            for i in pulse_program[-1]:
                wanted_string +=' ' +  str(i)
            raise LabscriptError("Flags" + wanted_string)            
            if fresh or (self.smart_cache['initial_values'] != initial_values) or \
                (len(self.smart_cache['pulse_program']) != len(pulse_program)) or \
                (self.smart_cache['pulse_program'] != pulse_program).any() or \
                not self.smart_cache['ready_to_go']:
            
                self.smart_cache['ready_to_go'] = True
                self.smart_cache['initial_values'] = initial_values

                # create initial flags string
                # NOTE: The spinapi can take a string or integer for flags.
                # If it is a string: 
                #     flag: 0          12
                #          '101100011111'
                #
                # If it is a binary number:
                #     flag:12          0
                #         0b111110001101
                #
                # Be warned!
                initial_flags = ''
                for i in range(12):
                    if initial_values['flag %d'%i]:
                        initial_flags += '1'
                    else:
                        initial_flags += '0'

                if self.programming_scheme == 'pb_start/BRANCH':
                    # Line zero is a wait on the final state of the program in 'pb_start/BRANCH' mode 
                    pb_inst_dds2(freqreg0,phasereg0,ampreg0,en0,0,freqreg1,phasereg1,ampreg1,en1,0,flags,WAIT,0,100)
                else:
                    # Line zero otherwise just contains the initial state 
                    pb_inst_dds2(0,0,0,initial_values['dds 0']['gate'],0,0,0,0,initial_values['dds 1']['gate'],0,initial_flags, CONTINUE, 0, 100)

                # Line one is a continue with the current front panel values:
                pb_inst_dds2(0,0,0,initial_values['dds 0']['gate'],0,0,0,0,initial_values['dds 1']['gate'],0,initial_flags, CONTINUE, 0, 100)
                # Now the rest of the program:
                if fresh or len(self.smart_cache['pulse_program']) != len(pulse_program) or \
                (self.smart_cache['pulse_program'] != pulse_program).any():
                    self.smart_cache['pulse_program'] = pulse_program
                    for args in pulse_program:
                        pb_inst_dds2(*args)
            
            # Are there waits in use in this experiment? The monitor waiting for the end
            # of the experiment will need to know:
            wait_monitor_exists = bool(hdf5_file['waits'].attrs['wait_monitor_acquisition_device'])
            waits_in_use = bool(len(hdf5_file['waits']))
            self.waits_pending = wait_monitor_exists and waits_in_use
            if waits_in_use and not wait_monitor_exists:
                # This should be caught during labscript compilation, but just in case.
                # Having waits but not a wait monitor means we can't tell when the shot
                # is over unless the shot ends in a STOP instruction:
                assert self.programming_scheme == 'pb_stop_programming/STOP'

            # Now we build a dictionary of the final state to send back to the GUI:
            return_values = {'dds 0':{'freq':finalfreq0, 'amp':finalamp0, 'phase':finalphase0, 'gate':en0},
                             'dds 1':{'freq':finalfreq1, 'amp':finalamp1, 'phase':finalphase1, 'gate':en1},
                            }
            # Since we are converting from an integer to a binary string, we need to reverse the string! (see notes above when we create flags variables)
            return_flags = str(bin(flags)[2:]).rjust(12,'0')[::-1]
            for i in range(12):
                return_values['flag %d'%i] = return_flags[i]
                
            return return_values
            


    def abort_transition_to_buffered(self):
        return self.transition_to_manual(True)

    def abort_buffered(self):
        return self.transition_to_manual(True)

    def transition_to_manual(self,abort = False):

        if abort:
            # If we're aborting the run, then we need to reset the Static DDSs to their initial values.
            # We also need to invalidate the smart programming cache.
            self.smart_cache['RF_DATA'] = None
            self.smart_cache['SWEEP_DATA'] = None
        else:
            # If we're not aborting the run, then we need to set the Static DDSs to their final values.
            values = self.final_values['channel 0']

        ZCU4 = serial.Serial(self.COMPort, baudrate=self.baudrate, timeout=1)
        if(ZCU4.isOpen() == False):
            ZCU4.open()
        ZCU4.close()

        return True

    def start_run(self):
        if(ZCU4ser.isOpen() == False):
            ZCU4ser.open()
        ZCU4ser.write(b"exec(open('send_pulse.py').read())\r\n")
        ZCU4ser.close()


    def shutdown(self):
        return


import labscript_utils.h5_lock  # noqa: F401
import h5py
import numpy as np

import labscript_utils.properties as properties


class ZCU4Parser(object):
    """Runviewer parser for the ZCU4 Pseudoclocks."""
    def __init__(self, path, device):
        """
        Args:
            path (str): path to h5 shot file
            device (str): labscript name of ZCU4 device
        """
        self.path = path
        self.name = device.name
        self.device = device

    def get_traces(self, add_trace, clock=None):
        """Reads the shot file and extracts hardware instructions to produce
        runviewer traces.

        Args:
            add_trace (func): function handle that adds traces to runviewer
            clock (tuple, optional): clock times from timing device, if not
                the primary pseudoclock

        Returns:
            dict: Dictionary of clocklines and triggers derived from instructions
        """

        if clock is not None:
            times, clock_value = clock[0], clock[1]
            clock_indices = np.where((clock_value[1:] - clock_value[:-1]) == 1)[0] + 1
            # If initial clock value is 1, then this counts as a rising edge
            # (clock should be 0 before experiment) but this is not picked up
            # by the above code. So we insert it!
            if clock_value[0] == 1:
                clock_indices = np.insert(clock_indices, 0, 0)
            clock_ticks = times[clock_indices]

        # get the pulse program
        pulse_programs = []
        with h5py.File(self.path, "r") as f:
            # Get the device properties
            device_props = properties.get(f, self.name, "device_properties")
            conn_props = properties.get(f, self.name, "connection_table_properties")

            self.clock_resolution = device_props["clock_resolution"]
            self.trigger_delay = device_props["trigger_delay"]
            self.wait_delay = device_props["wait_delay"]

            # Extract the pulse programs
            num_pseudoclocks = conn_props["num_pseudoclocks"]
            for i in range(num_pseudoclocks):
                pulse_programs.append(f[f"devices/{self.name}/PULSE_PROGRAM_{i}"][:])

        # Generate clocklines and triggers
        clocklines_and_triggers = {}
        
        for pseudoclock_name, pseudoclock in self.device.child_list.items():
            # Get pseudoclock index
            connection_parts = pseudoclock.parent_port.split()
            # Skip if not one of the 4 possible pseudoclock outputs (there is one for
            # the wait monitor too potentially)
            if connection_parts[0] != "pseudoclock":
                continue

            # Get the pulse program
            index = int(connection_parts[1])
            pulse_program = pulse_programs[index]

            time = []
            states = []
            trigger_index = 0
            t = 0 if clock is None else clock_ticks[trigger_index] + self.trigger_delay
            trigger_index += 1

            clock_factor = self.clock_resolution / 2.0

            last_instruction_was_wait = False
            for row in pulse_program:
                if row["reps"] == 0 and not last_instruction_was_wait:  # WAIT
                    last_instruction_was_wait = True
                    if clock is not None:
                        t = clock_ticks[trigger_index] + self.trigger_delay
                        trigger_index += 1
                    else:
                        t += self.wait_delay
                elif last_instruction_was_wait:
                    # two waits in a row means an indefinite wait, so we just skip this
                    # instruction.
                    last_instruction_was_wait = False
                    continue
                else:
                    last_instruction_was_wait = False
                    for i in range(row["reps"]):
                        for j in range(1, -1, -1):
                            time.append(t)
                            states.append(j)
                            t += row["half_period"] * clock_factor

            pseudoclock_clock = (np.array(time), np.array(states))

            for clock_line_name, clock_line in pseudoclock.child_list.items():
                # Ignore the dummy internal wait monitor clockline
                if clock_line.parent_port.startswith("GPIO"):
                    clocklines_and_triggers[clock_line_name] = pseudoclock_clock
                    add_trace(
                        clock_line_name, pseudoclock_clock, self.name, clock_line.parent_port
                    )

        return clocklines_and_triggers


import labscript_devices

labscript_device_name = 'ZCU4'
blacs_tab = 'labscript_devices.ZCU4.ZCU4Tab'
parser = 'labscript_devices.ZCU4.ZCU4Parser'

labscript_devices.register_classes(
    labscript_device_name=labscript_device_name,
    BLACS_tab=blacs_tab,
    runviewer_parser=parser,
)