
import sys
import time
import threading
import platform
import subprocess
import os
from fibre.utils import Event

try:
    if platform.system() == 'Windows':
        import win32console
        import colorama
        colorama.init()
except ModuleNotFoundError:
    print("Could not init terminal features.")
    print("Refer to install instructions at http://docs.odriverobotics.com/#downloading-and-installing-tools")
    sys.stdout.flush()
    pass

data_rate = 100
plot_rate = 10
num_samples = 1000

class OperationAbortedException(Exception):
    pass

def start_liveplotter(get_var_callback):
    """
    Starts a liveplotter.
    The variable that is plotted is retrieved from get_var_callback.
    This function returns immediately and the liveplotter quits when
    the user closes it.
    """

    import matplotlib.pyplot as plt

    cancellation_token = Event()

    global vals
    vals = []
    def fetch_data():
        global vals
        while not cancellation_token.is_set():
            try:
                data = get_var_callback()
            except Exception as ex:
                print(str(ex))
                time.sleep(1)
                continue
            vals.append(data)
            if len(vals) > num_samples:
                vals = vals[-num_samples:]
            time.sleep(1/data_rate)

    # TODO: use animation for better UI performance, see:
    # https://matplotlib.org/examples/animation/simple_anim.html
    def plot_data():
        global vals

        plt.ion()

        # Make sure the script terminates when the user closes the plotter
        def did_close(evt):
            cancellation_token.set()
        fig = plt.figure()
        fig.canvas.mpl_connect('close_event', did_close)

        while not cancellation_token.is_set():
            plt.clf()
            plt.plot(vals)
            fig.canvas.draw()
            fig.canvas.start_event_loop(1/plot_rate)

    threading.Thread(target=fetch_data, daemon=True).start()
    #threading.Thread(target=plot_data, daemon=True).start()

    plot_data()
    return cancellation_token;
    #plot_data()

def print_drv_regs(name, motor):
    """
    Dumps the current gate driver regisers for the specified motor
    """
    fault = motor.gate_driver.drv_fault
    status_reg_1 = motor.gate_driver.status_reg_1
    status_reg_2 = motor.gate_driver.status_reg_2
    ctrl_reg_1 = motor.gate_driver.ctrl_reg_1
    ctrl_reg_2 = motor.gate_driver.ctrl_reg_2
    print(name + ": " + str(fault))
    print("DRV Fault Code: " + str(fault))
    print("Status Reg 1: " + str(status_reg_1) + " (" + format(status_reg_1, '#010b') + ")")
    print("Status Reg 2: " + str(status_reg_2) + " (" + format(status_reg_2, '#010b') + ")")
    print("Control Reg 1: " + str(ctrl_reg_1) + " (" + format(ctrl_reg_1, '#013b') + ")")
    print("Control Reg 2: " + str(ctrl_reg_2) + " (" + format(ctrl_reg_2, '#09b') + ")")

def show_oscilloscope(odrv):
    size = 18000
    values = []
    for i in range(size):
        values.append(odrv.get_oscilloscope_val(i))

    import matplotlib.pyplot as plt
    plt.plot(values)
    plt.show()

def pe(o):
    def axis_errors(e):
        es = ''
        if e & 0x001:   es += 'invalid_state         '
        if e & 0x002:   es += 'under_voltage         '
        if e & 0x004:   es += 'over_voltage          '
        if e & 0x008:   es += 'current_meas_timeout  '
        if e & 0x010:   es += 'brake_res_disarmed    '
        if e & 0x020:   es += 'motor_disarmed        '
        if e & 0x040:   es += 'motor_failed          '
        if e & 0x080:   es += 'sensorless_est_failed '
        if e & 0x100:   es += 'encoder_failed        '
        if e & 0x200:   es += 'controller_failed     '
        if e & 0x400:   es += 'pos_ctrl_during_sless '
        return es
    def motor_errors(e):
        es = ''
        if e & 0x001:   es += 'phase_R_out_of_range '
        if e & 0x002:   es += 'phase_L_out_of_range '
        if e & 0x004:   es += 'adc_failed           '
        if e & 0x008:   es += 'drv_failed           '
        if e & 0x010:   es += 'ctrl_deadline_missed '
        if e & 0x020:   es += 'not_implemented_type '
        if e & 0x040:   es += 'brake_I_out_of_range '
        if e & 0x080:   es += 'modulation_magnitude '
        if e & 0x100:   es += 'brake_deadtime_viol  '
        if e & 0x200:   es += 'unexpected_tmr_cback '
        return es

    print('axis0.error:      %04x %s' % (o.axis0.error, axis_errors(o.axis0.error)))
    print('axis0.motor.error:%04x %s' % (o.axis0.motor.error, motor_errors(o.axis0.motor.error)))
    print('axis1.error:      %04x %s' % (o.axis1.error, axis_errors(o.axis1.error)))
    print('axis1.motor.error:%04x %s' % (o.axis1.motor.error, motor_errors(o.axis1.motor.error)))
    
def rate_test(device):
    """
    Tests how many integers per second can be transmitted
    """

    import matplotlib.pyplot as plt
    plt.ion()

    print("reading 10000 values...")
    numFrames = 10000
    vals = []
    for _ in range(numFrames):
        vals.append(device.motor0.loop_counter)

    plt.plot(vals)

    loopsPerFrame = (vals[-1] - vals[0])/numFrames
    loopsPerSec = (168000000/(2*10192))
    FramePerSec = loopsPerSec/loopsPerFrame
    print("Frames per second: " + str(FramePerSec))

def usb_burn_in_test(get_var_callback, cancellation_token):
    """
    Starts background threads that read a values form the USB device in a spin-loop
    """

    def fetch_data():
        global vals
        i = 0
        while not cancellation_token.is_set():
            try:
                get_var_callback()
                i += 1
            except Exception as ex:
                print(str(ex))
                time.sleep(1)
                i = 0
                continue
            if i % 1000 == 0:
                print("read {} values".format(i))
    threading.Thread(target=fetch_data, daemon=True).start()

def setup_udev_rules(logger):
    if platform.system() != 'Linux':
        logger.error("This command only makes sense on Linux")
    if os.getuid() != 0:
        logger.warn("you should run this as root, otherwise it will probably not work")
    with open('/etc/udev/rules.d/50-odrive.rules', 'w') as file:
        file.write('SUBSYSTEM=="usb", ATTR{idVendor}=="1209", ATTR{idProduct}=="0d3[0-9]", MODE="0666"\n')
    subprocess.check_call(["udevadm", "control", "--reload-rules"])
    subprocess.check_call(["udevadm", "trigger"])
    logger.info('udev rules configured successfully')

def yes_no_prompt(question, default=None):
    if default is None:
        question += " [y/n] "
    elif default == True:
        question += " [Y/n] "
    elif default == False:
        question += " [y/N] "

    while True:
        print(question, end='')

        choice = input().lower()
        if choice in {'yes', 'y'}:
            return True
        elif choice in {'no', 'n'}:
            return False
        elif choice == '' and default is not None:
            return default
