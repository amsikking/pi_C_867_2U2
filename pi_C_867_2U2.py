import time
import serial

class Controller:
    '''
    Basic device adaptor for C-867.2U2 PILine® motion controller, for two-axis
    positioning stages with PILine® ultrasonic piezo motors. Many more commands
    are available and have not been implemented.
    '''
    def __init__(
        self, which_port, name='C-867.2U2', verbose=True, very_verbose=False):
        self.name = name
        self.verbose = verbose
        self.very_verbose = very_verbose
        if self.verbose: print("%s: opening..."%name, end='')
        try:
            self.port = serial.Serial(
                port=which_port, baudrate=115200, timeout=5)
        except serial.serialutil.SerialException:
            raise IOError('No connection to %s on port %s'%(name, which_port))
        if self.verbose: print(" done.")
        # get device identity:
        self.identity = self._send('*IDN?')[0]
        # get physical units:
        self.x_unit, self.y_unit = [a.split('=')[1] for a in self._send(
            'SPA? 1 0x07000601 2 0x07000601')]
        assert self.x_unit == 'MM' and self.y_unit == 'MM'
        x_num, y_num = [float(a.split('=')[1]) for a in self._send(
            'SPA? 1 0xE 2 0xE')] # encoder counts per unit numerator
        x_den, y_den = [float(a.split('=')[1]) for a in self._send(
            'SPA? 1 0xF 2 0xF')] # encoder counts per unit denominator
        self.x_ecpu = x_num / x_den # x encoder counts per unit
        self.y_ecpu = y_num / y_den # y encoder counts per unit
        # get position and position limits:
        self.x, self.y = [
            float(a.split('=')[1]) for a in self._send('MOV? 1 2')]
        self.x_min, self.y_min = [
            float(a.split('=')[1]) for a in self._send('TMN? 1 2')]
        self.x_max, self.y_max = [
            float(a.split('=')[1]) for a in self._send('TMX? 1 2')]
        # get position tolerance and position tolerance limits:
        # -> position != target when encoder count > 'window exit 0' boundary
        self.xptc, self.yptc = [ # encoder counts for 'window exit 0'
            int(a.split('=')[1]) for a in self._send('SPA? 1 0x407 2 0x407')]
        self.xpt = 1e3 * self.xptc / self.x_ecpu # x position tolerance (um)
        self.ypt = 1e3 * self.yptc / self.y_ecpu # y position tolerance (um)        
        # -> tolerance limit = lower bound of next level = 'window enter 1'
        self.xptc_max, self.yptc_max = [ # encoder counts for 'window enter 1'
            int(a.split('=')[1]) for a in self._send('SPA? 1 0x416 2 0x416')]
        self.xpt_max = 1e3 * self.xptc_max / self.x_ecpu # x tol. limit (um)
        self.ypt_max = 1e3 * self.yptc_max / self.y_ecpu # y tol. limit (um)
        # get settling time:
        self.xst, self.yst = [ 1e3 * # s -> ms
            float(a.split('=')[1]) for a in self._send('SPA? 1 0x3F 2 0x3F ')]
        # get velocity and velocity limits:
        self.xv, self.yv = [
            float(a.split('=')[1]) for a in self._send('VEL? 1 2')]
        self.xv_max, self.yv_max = [
            float(a.split('=')[1]) for a in self._send('SPA? 1 0xA 2 0xA')]
        # get acceleration and acceleration limits:
        self.xa, self.ya = [
            float(a.split('=')[1]) for a in self._send('ACC? 1 2')]
        self.xa_max, self.ya_max = [
            float(a.split('=')[1]) for a in self._send('SPA? 1 0x4A 2 0x4A')]
        # get deceleration and decelerationlimits:
        self.xd, self.yd = [
            float(a.split('=')[1]) for a in self._send('DEC? 1 2')]
        self.xd_max, self.yd_max = [
            float(a.split('=')[1]) for a in self._send('SPA? 1 0x4B 2 0x4B')]
        # set state:
        self._enable_servo(True)
        self._enable_joystick(True)
        self._moving = False
        if self.verbose:
            self._print_attributes()
        return None

    def _print_attributes(self):
        print("%s: device identity"%self.name)
        print("%s:  = %s"%(self.name, self.identity))
        print("%s: units (x, y)"%self.name)
        print("%s:  = %s, %s"%(self.name, self.x_unit, self.y_unit))
        print("%s: encoder counts per unit (x, y)"%self.name)
        print("%s:  = %i, %i "%(self.name, self.x_ecpu, self.y_ecpu))
        print("%s: position (x, y)"%self.name)
        print("%s:  = %10.06f, %10.06f (mm)"%(self.name, self.x, self.y))
        print("%s: position limits (x_min, y_min, x_max, y_max)"%self.name)
        print("%s:  = %10.06f, %10.06f, %10.06f, %10.06f (mm)"%(
            self.name, self.x_min, self.y_min, self.x_max, self.y_max))
        print("%s: position tolerance (xpt, ypt)"%self.name)
        print("%s:  = %10.06f, %10.06f (um)"%(
            self.name, self.xpt, self.ypt))           
        print("%s: position tolerance limits (xpt_max, ypt_max)"%self.name)
        print("%s:  = %10.06f, %10.06f (um)"%(
            self.name, self.xpt_max, self.ypt_max))
        print("%s: settling time (xst, yst)"%self.name)
        print("%s:  = %10.06f, %10.06f (ms)"%(
            self.name, self.xst, self.yst))
        print("%s: velocity (xv, yv)"%self.name)
        print("%s:  = %10.06f, %10.06f (mm/s)"%(
            self.name, self.xv, self.yv))
        print("%s: velocity limits (xv_max, yv_max)"%self.name)
        print("%s:  = %10.06f, %10.06f (mm/s)"%(
            self.name, self.xv_max, self.yv_max))
        print("%s: acceleration (xa, ya)"%self.name)
        print("%s:  = %10.06f, %10.06f (mm/s^2)"%(self.name, self.xa, self.ya))
        print("%s: acceleration limits (xa_max, ya_max)"%self.name)
        print("%s:  = %10.06f, %10.06f (mm/s^2)"%(
            self.name, self.xa_max, self.ya_max))
        print("%s: deceleration (xd, yd)"%self.name)
        print("%s:  = %10.06f, %10.06f (mm/s^2)"%(
            self.name, self.xd, self.yd))
        print("%s: deceleration limits (xd_max, yd_max)"%self.name)
        print("%s:  = %10.06f, %10.06f (mm/s^2)"%(
            self.name, self.xd_max, self.yd_max))
        return None

    def _send(self, cmd, respond=True):
        if self.very_verbose:
            print("%s: sending cmd = "%self.name, cmd)
        cmd = bytes(cmd, encoding='ascii')
        self.port.write(cmd + b'\n')
        if respond:
            responses = []
            while True:
                response = self.port.readline()
                assert response.endswith(b'\n') # default terminator
                responses.append(response.rstrip().decode('ascii')) # strip ' '
                if len(response) == 1: break # = 1 for self._reboot()
                if response[-2] != 32: break # ASCII #32 = space -> not finished
        else:
            responses = None
        if self.very_verbose:
            print("%s: response    = "%self.name, responses)
        assert self.port.in_waiting == 0
        self._check_errors()
        return responses

    def _check_errors(self):
        self.port.write(b'ERR?\n')  # Get Error Number -> check with manual
        self.error = self.port.readline()
        if self.error != b'0\n':    # 0 = no error
            raise RuntimeError(
                "%s: error = "%self.name, self.error.decode("ascii"))
        return None

    def _get_cmd_list(self):
        if self.verbose:
            print("%s: getting list of available commands"%self.name)
        self.cmd_list = self._send('HLP?')
        if self.verbose:
            print("%s:  available commands -> "%self.name)
            for cmd in self.cmd_list:
                print(cmd)
        return self.cmd_list

    def _get_parameter_list(self):
        if self.verbose:
            print("%s: getting list of available parameters"%self.name)
        self.parameter_list = self._send('HPA?')
        if self.verbose:
            print("%s:  available parameters -> "%self.name)
            for parameter in self.parameter_list:
                print(parameter)
        return self.parameter_list

    def _get_parameter(self, p_id):
        if self.verbose:
            print("%s: getting parameter %s"%(self.name, p_id))
        value = self._send('SPA? 1 %s 2 %s'%(2*(p_id,)))
        if self.verbose:
            print("%s:  parameter value %s"%(self.name, value))
        return value

    def _set_parameter(self, p_id, value): # type(p_id) = int, float or char
        # -> check with docs or ._get_parameter_list() to see type and options
        v = str(value)
        if self.verbose:
            print("%s: setting parameter %s = "%(self.name, p_id) + v )
        self._send('SPA 1 '+ p_id + ' ' + v +
                      ' 2 '+ p_id + ' ' + v, respond=False)
        if self.verbose:
            print("%s:  finished setting parameter"%self.name)
        return None

    def _enable_servo(self, enable):
        if enable:
            self._send('SVO 1 1 2 1', respond=False)
        if not enable:
            self._send('SVO 1 0 2 0', respond=False)
        if self.very_verbose:
            print("%s: enable servo = %s"%(self.name, enable))
        self._servo_enabled = enable
        return None

    def _enable_joystick(self, enable):
        if enable:
            self._send('HIN 1 1', respond=False)
            self._send('HIN 2 1', respond=False)
        if not enable:
            self._send('HIN 1 0', respond=False)
            self._send('HIN 2 0', respond=False)
        if self.very_verbose:
            print("%s: joystick enable = %s"%(self.name, enable))
        self._joystick_enabled = enable
        return None

    def _reboot(self, finish_macro=True): # same as power cycle
        if self.verbose: print('%s: rebooting...'%self.name, end='')
        self.port.write(b'RBT\n')
        time.sleep(0.2) # time to reboot
        self._check_errors()
        if finish_macro:
            self.verbose, old_verbose = False, self.verbose
            while self._send('RMC?')[0] != '': # List Running Macros
                print('.', sep='', end='')
                time.sleep(0.3) # wait...
            self.verbose = old_verbose
        if self.verbose: print('done.')
        return None

    def get_position_mm(self):
        if self.verbose:
            print("%s: getting position"%self.name)
        self.x, self.y = [
            float(a.split('=')[1]) for a in self._send('MOV? 1 2')]
        if self.verbose:
            print("%s:  = (%6.03f, %6.03f) (mm)"%(self.name, self.x, self.y))
        return self.x, self.y

    def _finish_moving(self):
        if not self._moving:
            return None
        while True:
            self.port.write(b'\x05') # Request Motion Status
            response = self.port.read(2)
            if response == b'0\n': break
        self._moving = False
        if self._joystick_enabled: # re-enable
            self._send('HIN 1 1', respond=False)
            self._send('HIN 2 1', respond=False)
        if self.verbose: print('%s:  -> finished moving'%self.name)
        self._check_errors()
        return None

    # For max speed -> disable the joystick before calling move
    def move_mm(self, x, y, relative=True, block=True):
        self._finish_moving()
        if self._joystick_enabled: # disable
            self._send('HIN 1 0', respond=False)
            self._send('HIN 2 0', respond=False)
        if relative:
            self.get_position_mm() # must update self.x, self.y due to joystick
            self.x, self.y = float(self.x + x), float(self.y + y)
            cmd = 'MOV 1 %0.9f 2 %0.9f '%(self.x, self.y)
        if not relative: # Abolute move
            self.x, self.y = float(x), float(y)
            cmd = 'MOV 1 %0.9f 2 %0.9f '%(self.x, self.y)
        assert self.x_min <= self.x <= self.x_max
        assert self.y_min <= self.y <= self.y_max
        if self.verbose:
            print("%s: moving to (x, y)"%self.name)
            print("%s:  = %10.06f, %10.06f (mm)"%(self.name, self.x, self.y))
        self._send(cmd, respond=False)
        self._moving = True
        if block:
            self._finish_moving()
        return None

    def set_positional_tolerance_um(self, xpt=None, ypt=None, margin=0.1):
        if xpt is None: xpt = self.xpt
        if ypt is None: ypt = self.ypt
        self.xptc = int(1e-3 * xpt * self.x_ecpu) # um -> mm -> counts
        self.yptc = int(1e-3 * ypt * self.x_ecpu)
        margin_counts = int(1e-3 * margin * self.x_ecpu)
        assert self.xptc < self.xptc_max and self.yptc <= self.yptc_max
        # lower bound on tolerance > 0 (at least 1 encoder count)
        assert self.xptc - margin_counts > 0 and self.yptc - margin_counts > 0
        if self.verbose:
            print("%s: setting positional tolerance (xpt, ypt)"%self.name)
        self._send('SPA 1 0x407 %i 2 0x407 %i '%( # counts for 'window exit 0'
            self.xptc, self.yptc), respond=False)
        self._send('SPA 1 0x406 %i 2 0x406 %i '%( # counts for 'window enter 0'
            self.xptc - margin, self.yptc - margin), respond=False)
        self.xpt = 1e3 * self.xptc / self.x_ecpu # counts -> mm -> um
        self.ypt = 1e3 * self.yptc / self.y_ecpu
        if self.verbose:
            print("%s:  = %10.06f, %10.06f (um) -> finished"%(
                self.name, self.xpt, self.ypt))
        return None

    def set_settling_time_ms(self, xst=None, yst=None):
        if xst is None: xst = self.xst
        if yst is None: yst = self.yst
        self.xst, self.yst = float(xst), float(yst)
        assert 0 <= self.xst <= 1000 and 0 <= self.yst <= 1000
        if self.verbose:
            print("%s: setting settling time (xst, yst)"%self.name)
        self._send('SPA 1 0x3F %0.9f 2 0x3F %0.9f '%(
            1e-3 * self.xst, 1e-3 * self.yst), respond=False) # ms -> s
        if self.verbose:
            print("%s:  = %10.06f, %10.06f (ms) -> finished"%(
                self.name, self.xst, self.yst))
        return None

    def set_velocity(self, xv=None, yv=None):
        if xv is None: xv = self.xv
        if yv is None: yv = self.yv
        self.xv, self.yv = float(xv), float(yv)
        assert 0 <= self.xv <= self.xv_max and 0 <= self.yv <= self.yv_max
        if self.verbose:
            print("%s: setting velocity (xv, yv)"%self.name)
        self._send('VEL 1 %0.9f 2 %0.9f '%(self.xv, self.yv), respond=False)
        if self.verbose:
            print("%s:  = %10.06f, %10.06f (mm/s) -> finished"%(
                self.name, self.xv, self.yv))
        return None

    def set_acceleration(self, xa=None, ya=None):
        if xa is None: xa = self.xa
        if ya is None: ya = self.ya
        self.xa, self.ya = float(xa), float(ya)
        assert 0 <= self.xa <= self.xa_max and 0 <= self.ya <= self.ya_max
        if self.verbose:
            print("%s: setting acceleration (xa, ya)"%self.name)
        self._send('ACC 1 %0.9f 2 %0.9f '%(self.xa, self.ya), respond=False)
        if self.verbose:
            print("%s:  = %10.06f, %10.06f (mm/s^2) -> finished"%(
                self.name, self.xa, self.ya))
        return None

    def set_deceleration(self, xd=None, yd=None):
        if xd is None: xd = self.xd
        if yd is None: yd = self.yd
        self.xd, self.yd = float(xd), float(yd)
        assert 0 <= self.xd <= self.xd_max and 0 <= self.yd <= self.yd_max
        if self.verbose:
            print("%s: setting deceleration (xd, yd)"%self.name)
        self._send('ACC 1 %0.9f 2 %0.9f '%(self.xd, self.yd), respond=False)
        if self.verbose:
            print("%s:  = %10.06f, %10.06f (mm/s^2) -> finished"%(
                self.name, self.xd, self.yd))
        return None

    def close(self):
        if self.verbose: print("%s: closing..."%self.name, end=' ')
        self.port.close()
        if self.verbose: print("done.")
    
if __name__ == '__main__':
    stage = Controller(which_port='COM3', verbose=True)
    # test developer functions:
##    stage._get_cmd_list()
##    stage._get_parameter_list()
##    stage._get_parameter('0x3F')
##    stage._set_parameter('0x3F', 0.001)
##    stage._reboot()
##    stage._enable_servo(False)
##    stage._enable_servo(True)

    print('\nGet position (joystick causes nonconcurrent attributes)')
    stage.get_position_mm()

    print('\nAbsolute and relative moves:')
    stage.move_mm(0, 0, relative=False)
    stage.move_mm(1, 1)

    print('\nNon-blocking call:')
    stage.move_mm(1, 1, relative=False, block=False)
    print(' do something else...')
    stage.move_mm(0, 0, relative=False)

    print("\nTesting speed...")
    moves = 3
    move_x_mm, move_y_mm = 0.5, 0.5
    stage.set_velocity(120, 120)            # 0 -> 120
    stage.set_acceleration(500, 500)        # 0 -> 500
    stage.set_deceleration(500, 500)        # 0 -> 500
    stage.set_settling_time_ms(1, 1)        # 1 -> 1000
    stage.set_positional_tolerance_um(1, 1) # 0.2 -> 9.8
    
    stage.verbose = False
    stage._enable_joystick(False)           # turn off joystick for max speed
    start = time.perf_counter()
    for i in range(moves):
        stage.move_mm(0, 0, relative=False)
        stage.move_mm(move_x_mm, move_y_mm)
    end = time.perf_counter()
    time_per_move_s = (end - start) / moves
    print(time_per_move_s, ' -> seconds per move')
    stage._enable_joystick(True)            # re-enable joystick

##    print("\nTesting 384 move calls:")
##    # set stage properties for max speed:
##    stage.set_velocity(120, 120)            # 0 -> 120
##    stage.set_acceleration(500, 500)        # 0 -> 500
##    stage.set_deceleration(500, 500)        # 0 -> 500
##    stage.set_settling_time_ms(1, 1)        # 1 -> 1000
##    stage.set_positional_tolerance_um(1, 1) # 0.2 -> 9.8
##    # generate move count:
##    rows, cols = 16, 24
##    # make calls:
##    stage.move_mm(0, 0, relative=False)
##    stage.verbose = False
##    t0 = time.perf_counter()
##    for move in range(cols * rows):
##        stage.move_mm(0, 0, relative=False)
##    stage.move_mm(0, 0, relative=False)
##    t1 = time.perf_counter()
##    # results:
##    total_time = t1 - t0
##    print('total_time = %0.2fs'%total_time)                 # 43.12s
##    time_per_move_s = total_time / (cols * rows)
##    print('time_per_move_s = %0.3fs'%time_per_move_s)       # 0.112s

##    print("\nTesting 384 well plate:")
##    # set stage properties for max speed:
##    stage.set_velocity(120, 120)            # 0 -> 120
##    stage.set_acceleration(500, 500)        # 0 -> 500
##    stage.set_deceleration(500, 500)        # 0 -> 500
##    stage.set_settling_time_ms(1, 1)        # 1 -> 1000
##    stage.set_positional_tolerance_um(1, 1) # 0.2 -> 9.8
##    # find A1 center:
##    x_mm_A1_ul, y_mm_A1_ul = (50.2282, -34.8627) # A1 upper left
##    x_mm_A1_lr, y_mm_A1_lr = (47.3277, -31.8921) # A1 lower right
##    x_mm_A1_c, y_mm_A1_c = (x_mm_A1_lr + (x_mm_A1_ul - x_mm_A1_lr)/2,
##                            y_mm_A1_lr + (y_mm_A1_ul - y_mm_A1_lr)/2)
##    # generate position array:
##    rows, cols = 16, 24
##    well_spacing_mm = 4.5
##    XY_stage_positions = []
##    for c in range(cols):
##        col_positions = []
##        for r in range(rows): # move y-axis more frequently
##            well = 'r%02ic%02i'%(r, c)
##            col_positions.append(
##                (well,
##                (x_mm_A1_c - c * well_spacing_mm,   # -ve for x
##                 y_mm_A1_c + r * well_spacing_mm,   # +ve for y
##                 'absolute')))
##        # snake scan:
##        if not c % 2:   # odd number:
##            XY_stage_positions.append(col_positions)
##        else:           # even number:
##            XY_stage_positions.append(col_positions[::-1]) # reverse
##    # tile:
##    stage.move_mm(0, 0, relative=False)
##    stage.verbose = False
##    t0 = time.perf_counter()
##    for c in range(cols):
##        for r in range(rows):
##            well =  XY_stage_positions[c][r][0]
##            XY_mm = XY_stage_positions[c][r][1]
##            print('-> well: %s (%s)'%(well, XY_mm))
##            # Move to XY position:
##            stage.move_mm(XY_mm[0], XY_mm[1], relative=False)
##    stage.move_mm(0, 0, relative=False)    
##    t1 = time.perf_counter()
##    # results:
##    total_time = t1 - t0
##    print('total_time = %0.2fs'%total_time)                 # 167.50s
##    time_per_move_s = total_time / (cols * rows)
##    print('time_per_move_s = %0.3fs'%time_per_move_s)       # 0.436s

    stage.close()
