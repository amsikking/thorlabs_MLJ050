import time
import serial

class Controller:
    '''
    Basic device adaptor for Thorlabs MLJ050 motorized high-load vertical
    translation stage, 50 mm travel. Test code runs and seems robust.
    '''
    def __init__(self,
                 which_port,
                 name='MLJ050',
                 limits_mm=(0, 50), # reduce if necessary e.g. (0, 10)
                 home=True,         # default home if not 'self._homed'
                 verbose=True,
                 very_verbose=False):
        self.name = name
        self.limits_mm = limits_mm
        self.verbose = verbose
        self.very_verbose = very_verbose
        self.timeout = 1
        if self.verbose: print("%s: opening..."%self.name, end='')
        try:
            self.port = serial.Serial(
                port=which_port, baudrate=115200, timeout=self.timeout)
        except serial.serialutil.SerialException:
            raise IOError(
                '%s: no connection on port %s'%(self.name, which_port))
        if self.verbose: print(" done.")
        assert self.get_model_number() == 'MLJ050'
        self._counts_per_mm = 3 * 409600 # the 3x is not documented!
        self._velocity_mmps = 3 # velocity in mm/s for expected move time
        # In the manual it says the 'home parameters' should be set on init.
        # However, for this instance of the MLJ050 they are not set (empty)
        parameters = bytes(14) # -> empty bytes
        self._set_home_parameters(parameters)
        self._get_encoder_counts()
        self._get_homed_status()
        if not self._homed and home: self._home()
        self._moving = False

    def _encoder_counts_to_mm(self, encoder_counts):
        mm = encoder_counts / self._counts_per_mm
        if self.very_verbose:
            print('%s: -> %i encoder counts = %0.2fmm'%(
                self.name, encoder_counts, mm))
        return mm

    def _mm_to_encoder_counts(self, mm):
        encoder_counts = int(round(mm * self._counts_per_mm))
        if self.very_verbose:
            print('%s: -> %0.2fmm = %i encoder counts'%(
                self.name, mm, encoder_counts))
        return encoder_counts

    def _send(self, cmd, response_bytes=None):
        '''
        Making the cmd can be tricky. Here's some pointers:
        - here the 'Chan Ident' = b'\x01'
        - here the destination byte 'd' = b'\x50' (for generic USB device)
        - here the source byte 's' = b'\x01' (for host controller or PC)
        - if we send cmd with a 'data packet' then 'd' becomes 'd|' which
        equates to 'd|0x80' = or b'\xd0' (= bytes([a[0] | b[0]]) for
        a = b'\x50' and b = b'\x80')
        '''
        if self.very_verbose:
            print('%s: sending cmd: %s'%(self.name, cmd))
        self.port.write(cmd)
        if response_bytes is not None:
            response = self.port.read(response_bytes)
        else:
            response = None
        assert self.port.inWaiting() == 0
        if self.very_verbose:
            print('%s: -> response: %s'%(self.name, response))
        return response

    def get_model_number(self):
        if self.verbose:
            print('%s: getting model number'%self.name)
        # MGMSG_HW_REQ_INFO (no 'Chan Ident' here)
        cmd = b'\x05\x00\x00\x00\x50\x01'
        response = self._send(cmd, response_bytes=90)
        self.model_number = response[10:16].decode('ascii')
        if self.verbose:
            print('%s: -> model_number = %s'%(self.name, self.model_number))
        return self.model_number

    def _get_encoder_counts(self):
        if self.very_verbose:
            print('%s: getting encoder counts'%self.name)
        # MGMSG_MOT_REQ_POSCOUNTER
        cmd = b'\x11\x04\x01\x00\x50\x01'
        response = self._send(cmd, response_bytes=12)
        self._encoder_counts = int.from_bytes(
            response[-4:], byteorder='little', signed=True)
        if self.very_verbose:
            print('%s: -> encoder counts = %i'%(
                self.name, self._encoder_counts))
        self.position_mm = self._encoder_counts_to_mm(self._encoder_counts)
        return self._encoder_counts

    def _get_homed_status(self):
        if self.very_verbose:
            print('%s: getting homed status...'%self.name)
        # MGMSG_MOT_REQ_STATUSBITS
        cmd = b'\x29\x04\x00\x00\x50\x01'       
        status_bits = self._send(cmd, response_bytes=12)[8:]
        self._homed = bool(status_bits[1] & 4) # bit mask 0x00000400 = homed
        if self.very_verbose:
            print('%s: -> homed = %s'%(self.name, self._homed))
        return self._homed

    def _get_home_parameters(self):
        if self.very_verbose:
            print('%s: getting home parameters...'%self.name)
        # MGMSG_MOT_GET_HOMEPARAMS
        cmd = b'\x41\x04\x00\x00\x50\x01'       
        self._home_parameters = self._send(cmd, response_bytes=20)[6:]
        if self.very_verbose:
            print('%s: -> home parameters = %s'%(
                self.name, self._home_parameters))
        return self._home_parameters

    def _set_home_parameters(self, parameters):
        if self.very_verbose:
            print('%s: setting home parameters = %s'%(self.name, parameters))
        cmd = b'\x40\x04\x0E\x00\xd0\x01' + parameters
        self._send(cmd)
        assert self._get_home_parameters() == parameters
        if self.very_verbose:
            print('%s: done setting home parameters'%self.name)
        return None

    def _home(self, block=True):
        if self.very_verbose:
            print('%s: homing...'%self.name)
        # MGMSG_MOT_MOVE_HOME
        cmd = b'\x43\x04\x01\x00\x50\x01'
        self._send(cmd)
        if block:
            self._finish_home()
        return None

    def _finish_home(self):
        self.port.timeout = 60 # ~max time to home if starting at +50mm
        self.port.read(6) # MGMSG_MOT_MOVE_HOMED
        assert self.port.inWaiting() == 0
        self.port.timeout = self.timeout
        assert self._get_homed_status()
        if self.very_verbose:
            print('%s: -> done homing'%self.name)
        return None

    def _move_to_encoder_count(self, encoder_counts, block=True):
        if self._moving:
            self._finish_move()
        if self.very_verbose:
            print('%s: moving to encoder counts = %i'%(
                self.name, encoder_counts))
        self._target_encoder_counts = encoder_counts
        encoder_bytes = encoder_counts.to_bytes(4, 'little', signed=True)
        # MGMSG_MOT_MOVE_ABSOLUTE
        cmd = b'\x53\x04\x06\x00\xd0\x01\x01\x00' + encoder_bytes
        self._send(cmd)
        self._moving = True
        if block:
            self._finish_move()
        return None

    def _finish_move(self):
        if not self._moving:
            return
        # the move can be slow, and after sending the move command the
        # controller does not like the immediate follow up call
        # 'self.port.read(20)' (it seems to need ~1ms of delay!). So let's
        # wait a sensible amount of time before trying to collect the
        # 'move completed' message:
        counts = abs(self._target_encoder_counts - self._encoder_counts)
        relative_move_mm = self._encoder_counts_to_mm(counts)
        expected_move_time_s = relative_move_mm / self._velocity_mmps
        time_tolerance_s = 0.1 # how much extra time should we allow? (> 1ms!)
        time.sleep(expected_move_time_s + time_tolerance_s)
        self.port.read(20) # MGMSG_MOT_MOVE_COMPLETED
        assert self.port.inWaiting() == 0
        self.port.timeout = self.timeout
        self._get_encoder_counts()
        if self._moving: # i.e. if 'self.stop()' was not called then check move
            assert self._encoder_counts == self._target_encoder_counts
        self._moving = False
        if self.verbose:
            print('%s: -> finished move.'%self.name)
        return None

    def _legalize_move_mm(self, move_mm, relative):
        if self.verbose:
            print('%s: requested move_mm = %0.2f (relative=%s)'%(
                self.name, move_mm, relative))
        if relative:
            move_mm += self.position_mm
        assert self.limits_mm[0] <= move_mm <= self.limits_mm[1], (
            '%s: -> move_mm (%0.2f) exceeds limits_mm (%s)'%(
                self.name, move_mm, self.limits_mm))
        move_counts = self._mm_to_encoder_counts(move_mm)
        legal_move_mm = self._encoder_counts_to_mm(move_counts)
        if self.verbose:
            print('%s: -> legal move_mm  = %0.2f '%(self.name, legal_move_mm) +
                  '(%0.2f requested)'%move_mm)
        return legal_move_mm

    def move_mm(self, move_mm, relative=True, block=True):
        legal_move_mm = self._legalize_move_mm(move_mm, relative)
        if self.verbose:
            print('%s: moving to position_mm = %0.2f'%(
                self.name, legal_move_mm))
        encoder_counts = self._mm_to_encoder_counts(legal_move_mm)
        self._move_to_encoder_count(encoder_counts, block)
        return legal_move_mm

    def stop(self, mode='abrupt'):
        if self.verbose:
            print('%s: stopping (mode=%s)'%(self.name, mode))
        # MGMSG_MOT_MOVE_STOP
        assert mode in ('abrupt', 'profiled')
        if mode == 'abrupt':    cmd = b'\x65\x04\x01\x01\x50\x01'
        if mode == 'profiled':  cmd = b'\x65\x04\x01\x02\x50\x01'
        self._send(cmd, response_bytes=20)
        self._moving = False
        if self.verbose:
            print('%s: -> stopped'%self.name)
        return None

    def close(self):
        if self.verbose: print("%s: closing..."%self.name, end=' ')
        if self._moving:
            self._finish_move()
        self.port.close()
        if self.verbose: print("done.")
        return None

if __name__ == '__main__':
    controller = Controller(
        which_port='COM7', verbose=True, very_verbose=False)

    print('\n# position_mm = %0.2fmm'%controller.position_mm)

    print('\n# Some relative moves:')
    for moves in range(3):
        controller.move_mm(0.5)
    for moves in range(3):
        controller.move_mm(-0.5)

    print('\n# Some small relative moves:')
    for moves in range(3):
        controller.move_mm(1e-3)
        controller.move_mm(-1e-3)

    print('\n# Legalized move:')
    legal_move_mm = controller._legalize_move_mm(0.537846, relative=True)
    controller.move_mm(legal_move_mm)

    print('\n# Some random absolute moves:')
    import random
    for moves in range(3): # tested to 100 moves with 0-50mm range
        print('\n Random test #%i'%moves)
        random_move_mm = random.uniform(0, 5)
        move = controller.move_mm(random_move_mm, relative=False)

    print('\n# Non-blocking move:')
    controller.move_mm(2, block=False)
    controller.move_mm(1, block=False)
    print('(immediate follow up call forces finish on pending move)')
    print('doing something else')
    controller._finish_move()

    print('\n# Move and stop:')
    controller.move_mm(1, relative=False, block=False)
    controller.stop()
    controller.move_mm(1, relative=False, block=False)
    controller.stop(mode='profiled')

    controller.move_mm(0, relative=False)
    controller.close()
