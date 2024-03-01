from argparse import ArgumentParser, Namespace
import os
import sys
import json
from pathlib import Path
import logging
import subprocess

def camera_kernel_enabled():
    seconds_to_wait = 5
    time_between_checks = 0.2
    attemps_counter = 0
    max_failed_attemps = seconds_to_wait/time_between_checks
    current_device = f"/dev/video0"
    while True:
        if os.path.exists(current_device):
            device_selection_process = subprocess.Popen(["v4l2-ctl", "-d", current_device])
            device_selection_process.wait()
            device_info_process = subprocess.Popen(["v4l2-ctl","-D"], stdout=subprocess.PIPE)
            device_info_process.wait()
            card_type = device_info_process.stdout.read().decode().splitlines()[2]
            if card_type.find("imx477") != -1:
                return True
        if attemps_counter > max_failed_attemps:
            return False
         
        time.sleep(time_between_checks)
        attemps_counter+= 1

def enable_gpio_permission():
    command_0 = "chmod 777 /dev/gpiochip0"
    command_1 = "chmod 777 /dev/gpiochip1"
    sudo_passwd = 'Bentayga1234'
    sudo_command_0 = 'echo %s | sudo -S %s' % (sudo_passwd, command_0)
    sudo_command_1 = 'echo %s | sudo -S %s' % (sudo_passwd, command_1)
    seconds_to_wait = 5
    time_between_checks = 0.2
    attemps_counter = 0
    max_failed_attemps = seconds_to_wait/time_between_checks

    while True:
        if os.path.exists("/dev/gpiochip0") and os.path.exists("/dev/gpiochip1"):
            p_0 = os.system(sudo_command_0)
            p_1 = os.system(sudo_command_1)
            if p_0 == 0 and p_1 == 0:
                return True
            else:
                return False
        if attemps_counter > max_failed_attemps:
            return False
         
        time.sleep(time_between_checks)
        attemps_counter+= 1
        
def enable_backupdisk_permission(path: str) -> bool:
    command = f"chmod 777 {path}"
    sudo_passwd = 'Bentayga1234'
    sudo_command = 'echo %s | sudo -S %s' % (sudo_passwd, command)
    seconds_to_wait = 5
    time_between_checks = 0.2
    attemps_counter = 0
    max_failed_attemps = seconds_to_wait/time_between_checks

    while True:
        if os.path.exists(path):
            p = os.system(sudo_command)
            if p == 0:
                return True
            else:
                return False
        if attemps_counter > max_failed_attemps:
            return False
         
        time.sleep(time_between_checks)
        attemps_counter+= 1

def check_rtc_connected():
    rtc_address = "68"
    seconds_to_wait = 5
    time_between_checks = 0.2
    attemps_counter = 0
    max_failed_attemps = seconds_to_wait/time_between_checks

    while True:
        result = subprocess.check_output(['i2cdetect', '-r', '-y', '0']).decode('utf-8')
        if result.find(rtc_address) != -1:
            return True

        if attemps_counter > max_failed_attemps:
            return False
         
        time.sleep(time_between_checks)
        attemps_counter+= 1      

def get_arguments() -> Namespace:
    parser = ArgumentParser(description=__doc__)
    parser.add_argument('--log-out',
        type=str, required=False, action="store",
        help='Log with the information of the detection process',
    )
    parser.add_argument('--backup_path',
        type=str, action="store",
        help='Path to the USB in which the backup images will be stored',
    )

    return parser.parse_args()

def configure_logging(log_file: Path, debug: bool=True):
    if not os.path.isdir(log_file.parent):
        os.mkdir(log_file.parent)
    file_handler = logging.FileHandler(filename=log_file)
    terminal_handler = logging.StreamHandler()

    debug_format = '%(asctime)-16s [%(levelname)s](%(filename)s@%(funcName)s%(lineno)d%(threadName)s):%(message)s'
    info_format = '[%(levelname)s](%(filename)s %(lineno)d):%(message)s'
    logging.basicConfig(
        handlers=(file_handler, terminal_handler),
        level=logging.DEBUG if debug else logging.INFO,
        format=debug_format if debug else info_format
    )

def main():
    args = get_arguments()

    log_file = Path(args.log_out) if args.log_out else Path("sensor-connection.log")

    configure_logging(log_file)

    logging.info("Checking if the camera is connected.")
    result = camera_kernel_enabled()
    if not result:
        logging.critical("Camera not connected to the system.")
        sys.exit(1)
    else:
        logging.info("Succeed!")
        
    logging.info("Allowing system to use the GPIO.")
    result = enable_gpio_permission()
    if not result:
        logging.critical("Failed to enable GPIO.")
        sys.exit(2)
    else:
        logging.info("Succeed!")
    
    logging.info("Checking if the RTC is connected.")
    result = check_rtc_connected()
    if not result:
        logging.critical("RTC not detected.")
        sys.exit(4)
    else:
        logging.info("Succeed!")
    sys.exit(0)


if __name__=="__main__":
    main()

