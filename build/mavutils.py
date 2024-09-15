#!/usr/bin/env python

class mavfile(object):
    '''a generic mavlink port'''
    def __init__(self, fd, address, source_system=255, source_component=0, notimestamps=False, input=True, use_native=False):
        global mavfile_global
        if input:
            mavfile_global = self
        self.fd = fd
        self.sysid = 0
        self.param_sysid = (0,0)
        self.address = address
        self.timestamp = 0
        self.last_seq = {}
        self.mav_loss = 0
        self.mav_count = 0
        self.param_fetch_start = 0

        # state for each sysid
        self.sysid_state = {}
        self.sysid_state[self.sysid] = mavfile_state()

        # param state for each sysid/compid tuple
        self.param_state = {}
        self.param_state[self.param_sysid] = param_state()
        
        # status of param fetch, indexed by sysid,compid tuple
        self.source_system = source_system
        self.source_component = source_component
        self.first_byte = True
        self.robust_parsing = True
        self.mav = mavlink.MAVLink(self, srcSystem=self.source_system, srcComponent=self.source_component, use_native=use_native)
        self.mav.robust_parsing = self.robust_parsing
        self.logfile = None
        self.logfile_raw = None
        self.start_time = time.time()
        self.message_hooks = []
        self.idle_hooks = []
        self.uptime = 0.0
        self.notimestamps = notimestamps
        self._timestamp = None
        self.WIRE_PROTOCOL_VERSION = mavlink.WIRE_PROTOCOL_VERSION
        self.stop_on_EOF = False
        self.portdead = False

    @property
    def target_system(self):
        return self.sysid

    @property
    def target_component(self):
        return self.param_sysid[1]
    
    @target_system.setter
    def target_system(self, value):
        self.sysid = value
        if not self.sysid in self.sysid_state:
            self.sysid_state[self.sysid] = mavfile_state()
        if self.sysid != self.param_sysid[0]:
            self.param_sysid = (self.sysid, self.param_sysid[1])
            if not self.param_sysid in self.param_state:
                self.param_state[self.param_sysid] = param_state()

    @target_component.setter
    def target_component(self, value):
        if value != self.param_sysid[1]:
            self.param_sysid = (self.param_sysid[0], value)
            if not self.param_sysid in self.param_state:
                self.param_state[self.param_sysid] = param_state()

    @property
    def params(self):
        if self.param_sysid[1] == 0:
            eff_tuple = (self.sysid, 1)
            if eff_tuple in self.param_state:
                return getattr(self.param_state[eff_tuple],'params')
        return getattr(self.param_state[self.param_sysid],'params')

    @property
    def messages(self):
        return getattr(self.sysid_state[self.sysid],'messages')

    @property
    def flightmode(self):
        return getattr(self.sysid_state[self.sysid],'flightmode')

    @flightmode.setter
    def flightmode(self, value):
        setattr(self.sysid_state[self.sysid],'flightmode',value)

    @property
    def vehicle_type(self):
        return getattr(self.sysid_state[self.sysid],'vehicle_type')

    @vehicle_type.setter
    def vehicle_type(self, value):
        setattr(self.sysid_state[self.sysid],'vehicle_type',value)

    @property
    def mav_type(self):
        return getattr(self.sysid_state[self.sysid],'mav_type')

    @mav_type.setter
    def mav_type(self, value):
        setattr(self.sysid_state[self.sysid],'mav_type',value)
    
    @property
    def base_mode(self):
        return getattr(self.sysid_state[self.sysid],'base_mode')

    @base_mode.setter
    def base_mode(self, value):
        setattr(self.sysid_state[self.sysid],'base_mode',value)
    
    def auto_mavlink_version(self, buf):
        '''auto-switch mavlink protocol version'''
        global mavlink
        if len(buf) == 0:
            return
        try:
            magic = ord(buf[0])
        except:
            magic = buf[0]
        if not magic in [ 85, 254, 253 ]:
            return
        self.first_byte = False
        if self.WIRE_PROTOCOL_VERSION == "0.9" and magic == 254:
            self.WIRE_PROTOCOL_VERSION = "1.0"
            set_dialect(current_dialect)
        elif self.WIRE_PROTOCOL_VERSION == "1.0" and magic == 85:
            self.WIRE_PROTOCOL_VERSION = "0.9"
            os.environ['MAVLINK09'] = '1'
            set_dialect(current_dialect)
        elif self.WIRE_PROTOCOL_VERSION != "2.0" and magic == 253:
            self.WIRE_PROTOCOL_VERSION = "2.0"
            os.environ['MAVLINK20'] = '1'
            set_dialect(current_dialect)
        else:
            return
        # switch protocol 
        (callback, callback_args, callback_kwargs) = (self.mav.callback,
                                                      self.mav.callback_args,
                                                      self.mav.callback_kwargs)
        self.mav = mavlink.MAVLink(self, srcSystem=self.source_system, srcComponent=self.source_component)
        self.mav.robust_parsing = self.robust_parsing
        self.WIRE_PROTOCOL_VERSION = mavlink.WIRE_PROTOCOL_VERSION
        (self.mav.callback, self.mav.callback_args, self.mav.callback_kwargs) = (callback,
                                                                                 callback_args,
                                                                                 callback_kwargs)

    def recv(self, n=None):
        '''default recv method'''
        raise RuntimeError('no recv() method supplied')

    def close(self, n=None):
        '''default close method'''
        raise RuntimeError('no close() method supplied')

    def write(self, buf):
        '''default write method'''
        raise RuntimeError('no write() method supplied')


    def select(self, timeout):
        '''wait for up to timeout seconds for more data'''
        if self.fd is None:
            time.sleep(min(timeout,0.5))
            return True
        try:
            (rin, win, xin) = select.select([self.fd], [], [], timeout)
        except select.error:
            return False
        return len(rin) == 1

    def pre_message(self):
        '''default pre message call'''
        return

    def set_rtscts(self, enable):
        '''enable/disable RTS/CTS if applicable'''
        return

    def probably_vehicle_heartbeat(self, msg):
        if msg.get_srcComponent() == mavlink.MAV_COMP_ID_GIMBAL:
            return False
        if msg.type in (mavlink.MAV_TYPE_GCS,
                        mavlink.MAV_TYPE_GIMBAL,
                        mavlink.MAV_TYPE_ADSB,
                        mavlink.MAV_TYPE_ONBOARD_CONTROLLER):
            return False
        if msg.autopilot in frozenset([
                mavlink.MAV_AUTOPILOT_INVALID
                ]):
            return False
        return True

    def post_message(self, msg):
        '''default post message call'''
        if '_posted' in msg.__dict__:
            return
        msg._posted = True
        msg._timestamp = time.time()
        type = msg.get_type()

        if 'usec' in msg.__dict__:
            self.uptime = msg.usec * 1.0e-6
        if 'time_boot_ms' in msg.__dict__:
            self.uptime = msg.time_boot_ms * 1.0e-3

        if self._timestamp is not None:
            if self.notimestamps:
                msg._timestamp = self.uptime
            else:
                msg._timestamp = self._timestamp

        src_system = msg.get_srcSystem()
        src_component = msg.get_srcComponent()
        src_tuple = (src_system, src_component)

        radio_tuple = (ord('3'), ord('D'))

        if not src_system in self.sysid_state:
            # we've seen a new system
            self.sysid_state[src_system] = mavfile_state()

        add_message(self.sysid_state[src_system].messages, type, msg)

        if src_tuple == radio_tuple:
            # as a special case radio msgs are added for all sysids
            for s in self.sysid_state.keys():
                self.sysid_state[s].messages[type] = msg

        if not (src_tuple == radio_tuple or msg.get_type() == 'BAD_DATA'):
            if not src_tuple in self.last_seq:
                last_seq = -1
            else:
                last_seq = self.last_seq[src_tuple]
            seq = (last_seq+1) % 256
            seq2 = msg.get_seq()
            if seq != seq2 and last_seq != -1:
                diff = (seq2 - seq) % 256
                self.mav_loss += diff
                #print("lost %u seq=%u seq2=%u last_seq=%u src_tupe=%s %s" % (diff, seq, seq2, last_seq, str(src_tuple), msg.get_type()))
            self.last_seq[src_tuple] = seq2
            self.mav_count += 1
        
        self.timestamp = msg._timestamp
        if type == 'HEARTBEAT' and self.probably_vehicle_heartbeat(msg):
            if self.sysid == 0:
                # lock onto id tuple of first vehicle heartbeat
                self.sysid = src_system
            if float(mavlink.WIRE_PROTOCOL_VERSION) >= 1:
                self.flightmode = mode_string_v10(msg)
                self.mav_type = msg.type
                self.base_mode = msg.base_mode
                self.sysid_state[self.sysid].armed = (msg.base_mode & mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                self.sysid_state[self.sysid].mav_type = msg.type
                self.sysid_state[self.sysid].mav_autopilot = msg.autopilot
        elif type == 'HIGH_LATENCY2':
            if self.sysid == 0:
                # lock onto id tuple of first vehicle heartbeat
                self.sysid = src_system
            self.flightmode = mode_string_v10(msg)
            self.mav_type = msg.type
            if msg.autopilot == mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA:
                self.base_mode = msg.custom0
                self.sysid_state[self.sysid].armed = (msg.custom0 & mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            self.sysid_state[self.sysid].mav_type = msg.type
            self.sysid_state[self.sysid].mav_autopilot = msg.autopilot

        elif type == 'PARAM_VALUE':
            if not src_tuple in self.param_state:
                self.param_state[src_tuple] = param_state()
            self.param_state[src_tuple].params[msg.param_id] = msg.param_value
        elif type == 'SYS_STATUS' and mavlink.WIRE_PROTOCOL_VERSION == '0.9':
            self.flightmode = mode_string_v09(msg)
        elif type == 'GPS_RAW':
            if self.sysid_state[src_system].messages['HOME'].fix_type < 2:
                self.sysid_state[src_system].messages['HOME'] = msg
        elif type == 'GPS_RAW_INT':
            if self.sysid_state[src_system].messages['HOME'].fix_type < 3:
                self.sysid_state[src_system].messages['HOME'] = msg
        for hook in self.message_hooks:
            hook(self, msg)

        if (msg.get_signed() and
            self.mav.signing.link_id == 0 and
            msg.get_link_id() != 0 and
            self.target_system == msg.get_srcSystem() and
            self.target_component == msg.get_srcComponent()):
            # change to link_id from incoming packet
            self.mav.signing.link_id = msg.get_link_id()


    def packet_loss(self):
        '''packet loss as a percentage'''
        if self.mav_count == 0:
            return 0
        return (100.0*self.mav_loss)/(self.mav_count+self.mav_loss)


    def recv_msg(self):
        '''message receive routine'''
        self.pre_message()
        while True:
            n = self.mav.bytes_needed()
            s = self.recv(n)
            numnew = len(s)

            if numnew != 0:
                if self.logfile_raw:
                    if is_py3:
                        self.logfile_raw.write(s)
                    else:
                        self.logfile_raw.write(str(s))
                if self.first_byte:
                    self.auto_mavlink_version(s)

            # We always call parse_char even if the new string is empty, because the existing message buf might already have some valid packet
            # we can extract
            msg = self.mav.parse_char(s)
            if msg:
                if self.logfile and  msg.get_type() != 'BAD_DATA' :
                    usec = int(time.time() * 1.0e6) & ~3
                    if is_py3:
                        self.logfile.write(struct.pack('>Q', usec) + msg.get_msgbuf())
                    else:
                        self.logfile.write(str(struct.pack('>Q', usec) + msg.get_msgbuf()))
                self.post_message(msg)
                return msg
            else:
                # if we failed to parse any messages _and_ no new bytes arrived, return immediately so the client has the option to
                # timeout
                if numnew == 0:
                    return None
                
    def recv_match(self, condition=None, type=None, blocking=False, timeout=None):
        '''recv the next MAVLink message that matches the given condition
        type can be a string or a list of strings'''
        if type is not None and not isinstance(type, list) and not isinstance(type, set):
            type = [type]
        start_time = time.time()
        while True:
            if timeout is not None:
                now = time.time()
                if now < start_time:
                    start_time = now # If an external process rolls back system time, we should not spin forever.
                if start_time + timeout < time.time():
                    return None
            m = self.recv_msg()
            if m is None:
                if blocking:
                    for hook in self.idle_hooks:
                        hook(self)
                    if timeout is None:
                        self.select(0.05)
                    else:
                        self.select(timeout/2)
                    continue
                return None
            if type is not None and not m.get_type() in type:
                continue
            if not evaluate_condition(condition, self.messages):
                continue
            return m

    def check_condition(self, condition):
        '''check if a condition is true'''
        return evaluate_condition(condition, self.messages)

    def mavlink10(self):
        '''return True if using MAVLink 1.0 or later'''
        return float(self.WIRE_PROTOCOL_VERSION) >= 1

    def mavlink20(self):
        '''return True if using MAVLink 2.0 or later'''
        return float(self.WIRE_PROTOCOL_VERSION) >= 2

    def setup_logfile(self, logfile, mode='wb'):
        '''start logging to the given logfile, with timestamps'''
        self.logfile = open(logfile, mode=mode)

    def setup_logfile_raw(self, logfile, mode='wb'):
        '''start logging raw bytes to the given logfile, without timestamps'''
        self.logfile_raw = open(logfile, mode=mode)

    def wait_heartbeat(self, blocking=True, timeout=None):
        '''wait for a heartbeat so we know the target system IDs'''
        return self.recv_match(type='HEARTBEAT', blocking=blocking, timeout=timeout)

    def param_fetch_all(self):
        '''initiate fetch of all parameters'''
        if time.time() - self.param_fetch_start < 2.0:
            # don't fetch too often
            return
        self.param_fetch_start = time.time()
        self.mav.param_request_list_send(self.target_system, self.target_component)

    def param_fetch_one(self, name):
        '''initiate fetch of one parameter'''
        try:
            idx = int(name)
            self.mav.param_request_read_send(self.target_system, self.target_component, b"", idx)
        except Exception:
            if sys.version_info.major >= 3 and not isinstance(name, bytes):
                name = bytes(name,'ascii')
            self.mav.param_request_read_send(self.target_system, self.target_component, name, -1)

    def time_since(self, mtype):
        '''return the time since the last message of type mtype was received'''
        if not mtype in self.messages:
            return time.time() - self.start_time
        return time.time() - self.messages[mtype]._timestamp

    def param_set_send(self, parm_name, parm_value, parm_type=None):
        '''wrapper for parameter set'''
        if self.mavlink10():
            if parm_type is None:
                parm_type = mavlink.MAVLINK_TYPE_FLOAT
            self.mav.param_set_send(self.target_system, self.target_component,
                                    parm_name.encode('utf8'), parm_value, parm_type)
        else:
            self.mav.param_set_send(self.target_system, self.target_component,
                                    parm_name.encode('utf8'), parm_value)

    def waypoint_request_list_send(self):
        '''wrapper for waypoint_request_list_send'''
        if self.mavlink10():
            self.mav.mission_request_list_send(self.target_system, self.target_component)
        else:
            self.mav.waypoint_request_list_send(self.target_system, self.target_component)

    def waypoint_clear_all_send(self):
        '''wrapper for waypoint_clear_all_send'''
        if self.mavlink10():
            self.mav.mission_clear_all_send(self.target_system, self.target_component)
        else:
            self.mav.waypoint_clear_all_send(self.target_system, self.target_component)

    def waypoint_request_send(self, seq):
        '''wrapper for waypoint_request_send'''
        if self.mavlink10():
            self.mav.mission_request_send(self.target_system, self.target_component, seq)
        else:
            self.mav.waypoint_request_send(self.target_system, self.target_component, seq)

    def waypoint_set_current_send(self, seq):
        '''wrapper for waypoint_set_current_send'''
        if self.mavlink10():
            self.mav.mission_set_current_send(self.target_system, self.target_component, seq)
        else:
            self.mav.waypoint_set_current_send(self.target_system, self.target_component, seq)

    def waypoint_current(self):
        '''return current waypoint'''
        if self.mavlink10():
            m = self.recv_match(type='MISSION_CURRENT', blocking=True)
        else:
            m = self.recv_match(type='WAYPOINT_CURRENT', blocking=True)
        return m.seq

    def waypoint_count_send(self, seq):
        '''wrapper for waypoint_count_send'''
        if self.mavlink10():
            self.mav.mission_count_send(self.target_system, self.target_component, seq)
        else:
            self.mav.waypoint_count_send(self.target_system, self.target_component, seq)

    def set_mode_flag(self, flag, enable):
        '''
        Enables/ disables MAV_MODE_FLAG
        @param flag The mode flag, 
          see MAV_MODE_FLAG enum
        @param enable Enable the flag, (True/False)
        '''
        if self.mavlink10():
            mode = self.base_mode
            if enable:
                mode = mode | flag
            elif not enable:
                mode = mode & ~flag
            self.mav.command_long_send(self.target_system, self.target_component,
                                           mavlink.MAV_CMD_DO_SET_MODE, 0,
                                           mode,
                                           0, 0, 0, 0, 0, 0)
        else:
            print("Set mode flag not supported")

    def set_mode_auto(self):
        '''enter auto mode'''
        if self.mavlink10():
            self.mav.command_long_send(self.target_system, self.target_component,
                                       mavlink.MAV_CMD_MISSION_START, 0, 0, 0, 0, 0, 0, 0, 0)
        else:
            MAV_ACTION_SET_AUTO = 13
            self.mav.action_send(self.target_system, self.target_component, MAV_ACTION_SET_AUTO)

    def mode_mapping(self):
        '''return dictionary mapping mode names to numbers, or None if unknown'''
        mav_type = self.sysid_state[self.sysid].mav_type
        mav_autopilot = self.sysid_state[self.sysid].mav_autopilot
        if mav_autopilot == mavlink.MAV_AUTOPILOT_PX4:
            return px4_map
        if mav_type is None:
            return None
        return mode_mapping_byname(mav_type)

    def set_mode_apm(self, mode, custom_mode = 0, custom_sub_mode = 0):
        '''enter arbitrary mode'''
        if isinstance(mode, str):
            mode_map = self.mode_mapping()
            if mode_map is None or mode not in mode_map:
                print("Unknown mode '%s'" % mode)
                return
            mode = mode_map[mode]
        # set mode by integer mode number for ArduPilot
        self.mav.command_long_send(self.target_system,
                                   self.target_component,
                                   mavlink.MAV_CMD_DO_SET_MODE,
                                   0,
                                   mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                                   mode,
                                   0,
                                   0,
                                   0,
                                   0,
                                   0)

    def set_mode_px4(self, mode, custom_mode, custom_sub_mode):
        '''enter arbitrary mode'''
        if isinstance(mode, str):
            mode_map = self.mode_mapping()
            if mode_map is None or mode not in mode_map:
                print("Unknown mode '%s'" % mode)
                return
            # PX4 uses two fields to define modes
            mode, custom_mode, custom_sub_mode = px4_map[mode]
        self.mav.command_long_send(self.target_system, self.target_component,
                                   mavlink.MAV_CMD_DO_SET_MODE, 0, mode, custom_mode, custom_sub_mode, 0, 0, 0, 0)

    def set_mode(self, mode, custom_mode = 0, custom_sub_mode = 0):
        '''set arbitrary flight mode'''
        mav_autopilot = self.field('HEARTBEAT', 'autopilot', None)
        if mav_autopilot == mavlink.MAV_AUTOPILOT_PX4:
            self.set_mode_px4(mode, custom_mode, custom_sub_mode)
        else:
            self.set_mode_apm(mode)
        
    def set_mode_rtl(self):
        '''enter RTL mode'''
        if self.mavlink10():
            self.mav.command_long_send(self.target_system, self.target_component,
                                       mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0, 0, 0, 0, 0, 0)
        else:
            MAV_ACTION_RETURN = 3
            self.mav.action_send(self.target_system, self.target_component, MAV_ACTION_RETURN)

    def set_mode_manual(self):
        '''enter MANUAL mode'''
        if self.mavlink10():
            self.mav.command_long_send(self.target_system, self.target_component,
                                       mavlink.MAV_CMD_DO_SET_MODE, 0,
                                       mavlink.MAV_MODE_MANUAL_ARMED,
                                       0, 0, 0, 0, 0, 0)
        else:
            MAV_ACTION_SET_MANUAL = 12
            self.mav.action_send(self.target_system, self.target_component, MAV_ACTION_SET_MANUAL)

    def set_mode_fbwa(self):
        '''enter FBWA mode'''
        if self.mavlink10():
            self.mav.command_long_send(self.target_system, self.target_component,
                                       mavlink.MAV_CMD_DO_SET_MODE, 0,
                                       mavlink.MAV_MODE_STABILIZE_ARMED,
                                       0, 0, 0, 0, 0, 0)
        else:
            print("Forcing FBWA not supported")

    def set_mode_loiter(self):
        '''enter LOITER mode'''
        if self.mavlink10():
            self.mav.command_long_send(self.target_system, self.target_component,
                                       mavlink.MAV_CMD_NAV_LOITER_UNLIM, 0, 0, 0, 0, 0, 0, 0, 0)
        else:
            MAV_ACTION_LOITER = 27
            self.mav.action_send(self.target_system, self.target_component, MAV_ACTION_LOITER)

    def set_servo(self, channel, pwm):
        '''set a servo value'''
        self.mav.command_long_send(self.target_system, self.target_component,
                                   mavlink.MAV_CMD_DO_SET_SERVO, 0,
                                   channel, pwm,
                                   0, 0, 0, 0, 0)


    def set_relay(self, relay_pin=0, state=True):
        '''Set relay_pin to value of state'''
        if self.mavlink10():
            self.mav.command_long_send(
                self.target_system,  # target_system
                self.target_component, # target_component
                mavlink.MAV_CMD_DO_SET_RELAY, # command
                0, # Confirmation
                relay_pin, # Relay Number
                int(state), # state (1 to indicate arm)
                0, # param3 (all other params meaningless)
                0, # param4
                0, # param5
                0, # param6
                0) # param7
        else:
            print("Setting relays not supported.")

    def calibrate_level(self):
        '''calibrate accels (1D version)'''
        self.mav.command_long_send(self.target_system, self.target_component,
                                   mavlink.MAV_CMD_PREFLIGHT_CALIBRATION, 0,
                                   1, 1, 0, 0, 0, 0, 0)

    def calibrate_pressure(self):
        '''calibrate pressure'''
        if self.mavlink10():
            self.mav.command_long_send(self.target_system, self.target_component,
                                       mavlink.MAV_CMD_PREFLIGHT_CALIBRATION, 0,
                                       0, 0, 1, 0, 0, 0, 0)
        else:
            MAV_ACTION_CALIBRATE_PRESSURE = 20
            self.mav.action_send(self.target_system, self.target_component, MAV_ACTION_CALIBRATE_PRESSURE)

    def reboot_autopilot(self, hold_in_bootloader=False, force=False):
        '''reboot the autopilot'''
        if self.mavlink10():
            if hold_in_bootloader:
                param1 = 3
            else:
                param1 = 1
            if force:
                param6 = 20190226
            else:
                param6 = 0
            self.mav.command_long_send(self.target_system, self.target_component,
                                       mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, 0,
                                       param1, 0, 0, 0, 0, param6, 0)

    def wait_gps_fix(self):
        self.recv_match(type='VFR_HUD', blocking=True)
        if self.mavlink10():
            self.recv_match(type='GPS_RAW_INT', blocking=True,
                            condition='GPS_RAW_INT.fix_type>=3 and GPS_RAW_INT.lat != 0')
        else:
            self.recv_match(type='GPS_RAW', blocking=True,
                            condition='GPS_RAW.fix_type>=2 and GPS_RAW.lat != 0')

    def location(self, relative_alt=False):
        '''return current location'''
        self.wait_gps_fix()
        # wait for another VFR_HUD, to ensure we have correct altitude
        self.recv_match(type='VFR_HUD', blocking=True)
        self.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        if relative_alt:
            alt = self.messages['GLOBAL_POSITION_INT'].relative_alt*0.001
        else:
            alt = self.messages['VFR_HUD'].alt
        return location(self.messages['GPS_RAW_INT'].lat*1.0e-7,
                        self.messages['GPS_RAW_INT'].lon*1.0e-7,
                        alt,
                        self.messages['VFR_HUD'].heading)

    def arducopter_arm(self):
        '''arm motors (arducopter only)'''
        if self.mavlink10():
            self.mav.command_long_send(
                self.target_system,  # target_system
                self.target_component,
                mavlink.MAV_CMD_COMPONENT_ARM_DISARM, # command
                0, # confirmation
                1, # param1 (1 to indicate arm)
                0, # param2 (all other params meaningless)
                0, # param3
                0, # param4
                0, # param5
                0, # param6
                0) # param7

    def arducopter_disarm(self):
        '''disarm motors (arducopter only)'''
        if self.mavlink10():
            self.mav.command_long_send(
                self.target_system,  # target_system
                self.target_component,
                mavlink.MAV_CMD_COMPONENT_ARM_DISARM, # command
                0, # confirmation
                0, # param1 (0 to indicate disarm)
                0, # param2 (all other params meaningless)
                0, # param3
                0, # param4
                0, # param5
                0, # param6
                0) # param7

    def motors_armed(self):
        '''return true if motors armed'''
        return self.sysid_state[self.sysid].armed

    def motors_armed_wait(self):
        '''wait for motors to be armed'''
        while True:
            m = self.wait_heartbeat()
            if self.motors_armed():
                return

    def motors_disarmed_wait(self):
        '''wait for motors to be disarmed'''
        while True:
            m = self.wait_heartbeat()
            if not self.motors_armed():
                return


    def field(self, type, field, default=None):
        '''convenient function for returning an arbitrary MAVLink
           field with a default'''
        if not type in self.messages:
            return default
        return getattr(self.messages[type], field, default)

    def param(self, name, default=None):
        '''convenient function for returning an arbitrary MAVLink
           parameter with a default'''
        if not name in self.params:
            return default
        return self.params[name]

    def setup_signing(self, secret_key, sign_outgoing=True, allow_unsigned_callback=None, initial_timestamp=None, link_id=None):
        '''setup for MAVLink2 signing'''
        self.mav.signing.secret_key = secret_key
        self.mav.signing.sign_outgoing = sign_outgoing
        self.mav.signing.allow_unsigned_callback = allow_unsigned_callback
        if link_id is None:
            # auto-increment the link_id for each link
            global global_link_id
            link_id = global_link_id
            global_link_id = min(global_link_id + 1, 255)
        self.mav.signing.link_id = link_id
        if initial_timestamp is None:
            # timestamp is time since 1/1/2015
            epoch_offset = 1420070400
            now = max(time.time(), epoch_offset)
            initial_timestamp = now - epoch_offset
            initial_timestamp = int(initial_timestamp * 100 * 1000)
        # initial_timestamp is in 10usec units
        self.mav.signing.timestamp = initial_timestamp

    def disable_signing(self):
        '''disable MAVLink2 signing'''
        self.mav.signing.secret_key = None
        self.mav.signing.sign_outgoing = False
        self.mav.signing.allow_unsigned_callback = None
        self.mav.signing.link_id = 0
        self.mav.signing.timestamp = 0



class mavudp(mavfile):
    '''a UDP mavlink socket'''
    def __init__(self, device, input=True, broadcast=False, source_system=255, source_component=0, use_native=default_native, timeout=0):
        a = device.split(':')
        if len(a) != 2:
            print("UDP ports must be specified as host:port")
            sys.exit(1)
        self.port = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_server = input
        self.broadcast = False
        if input:
            self.port.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.port.bind((a[0], int(a[1])))
        else:
            self.destination_addr = (a[0], int(a[1]))
            if broadcast:
                self.port.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
                self.broadcast = True
        set_close_on_exec(self.port.fileno())
        self.port.setblocking(0)
        self.last_address = None
        self.timeout = timeout
        self.clients = set()
        self.clients_last_alive = {}
        self.resolved_destination_addr = None
        mavfile.__init__(self, self.port.fileno(), device, source_system=source_system, source_component=source_component, input=input, use_native=use_native)

    def close(self):
        self.port.close()

    def recv(self,n=None):
        try:
            data, new_addr = self.port.recvfrom(UDP_MAX_PACKET_LEN)
        except socket.error as e:
            if e.errno in [ errno.EAGAIN, errno.EWOULDBLOCK, errno.ECONNREFUSED ]:
                return ""
            raise
        if self.udp_server:
            self.clients.add(new_addr)
            self.clients_last_alive[new_addr] = time.time()
        elif self.broadcast:
            self.last_address = new_addr
        return data

    def write(self, buf):
        try:
            if self.udp_server:
                current_time = time.time()
                to_remove = set()
                for address in self.clients:
                    if len(self.clients) == 1 or self.timeout <= 0 or self.clients_last_alive[address] + self.timeout > current_time:
                        self.port.sendto(buf, address)
                    elif len(self.clients) > 1 and len(to_remove) < len(self.clients) - 1:
                        # we keep always at least 1 client, so we don't break old behavior
                        to_remove.add(address)
                        self.clients_last_alive.pop(address)
                self.clients -= to_remove
            else:
                if self.last_address and self.broadcast:
                    self.destination_addr = self.last_address
                    self.broadcast = False
                    self.port.connect(self.destination_addr)
                # turn a (possible) hostname into an IP address to
                # avoid resolving the hostname for every packet sent:
                if self.destination_addr[0] != self.resolved_destination_addr:
                    self.resolved_destination_addr = self.destination_addr[0]
                    self.destination_addr = (socket.gethostbyname(self.destination_addr[0]), self.destination_addr[1])
                self.port.sendto(buf, self.destination_addr)
        except socket.error:
            pass

    def recv_msg(self):
        '''message receive routine for UDP link'''
        self.pre_message()
        s = self.recv()
        if len(s) > 0:
            if self.first_byte:
                self.auto_mavlink_version(s)

        m = self.mav.parse_char(s)
        if m is not None:
            self.post_message(m)

        return m