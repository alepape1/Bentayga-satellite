import time
import math
import os
import subprocess
import serial
import threading
import Jetson.GPIO as GPIO
from pathlib import Path

def camera_gpio_control(
    capturing_event: threading.Event,
    memory_l25_event: threading.Event,
    memory_l50_event: threading.Event,
    memory_l75_event: threading.Event,
    memory_full_event: threading.Event,
    error_event: threading.Event
    ):
    camera_capturing_pin = 12
    memory_lsb = 16
    memory_msb = 18
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(camera_capturing_pin, GPIO.OUT)
    GPIO.setup(memory_lsb, GPIO.OUT)
    GPIO.setup(memory_msb, GPIO.OUT)
    GPIO.output(camera_capturing_pin, GPIO.HIGH)
    GPIO.output(memory_lsb, GPIO.HIGH)
    GPIO.output(memory_msb, GPIO.HIGH)
    try:
        while not error_event.is_set():
            if capturing_event.is_set(): GPIO.output(camera_capturing_pin, GPIO.LOW)
            else: GPIO.output(camera_capturing_pin, GPIO.HIGH)
            
            if memory_full_event.is_set():
                GPIO.output(memory_msb, GPIO.LOW)
                GPIO.output(memory_lsb, GPIO.LOW)
            elif memory_l75_event.is_set():
                GPIO.output(memory_msb, GPIO.LOW)
                GPIO.output(memory_lsb, GPIO.HIGH)
            elif memory_l50_event.is_set():
                GPIO.output(memory_msb, GPIO.HIGH)
                GPIO.output(memory_lsb, GPIO.LOW)
            elif memory_l25_event.is_set():
                GPIO.output(memory_msb, GPIO.HIGH)
                GPIO.output(memory_lsb, GPIO.HIGH)
                
            time.sleep(0.5)
    except Exception as e:
        raise e
    finally:
        GPIO.output(camera_capturing_pin, GPIO.HIGH)
        GPIO.output(memory_lsb, GPIO.HIGH)
        GPIO.output(memory_msb, GPIO.HIGH)
        GPIO.cleanup()
        
def set_camera_capturing():
    camera_capturing_pin = 12
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(camera_capturing_pin, GPIO.OUT)
    GPIO.output(camera_capturing_pin, GPIO.LOW)

def set_camera_not_capturing():
    camera_capturing_pin = 12
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(camera_capturing_pin, GPIO.OUT)
    GPIO.output(camera_capturing_pin, GPIO.HIGH)

def wait_for_capture(
    capture_folder: Path,
    name: str,
    timeout: int, 
    backup_folder: Path = None) -> bool:
    
    if not capture_folder.is_dir():
        raise FileNotFoundError("Capture folder not found")

    file_path = capture_folder.joinpath(name)
    max_attemps = int(math.ceil(timeout/0.5))
    failed_attemps = 0
    while True:
        if file_path.exists():
            time.sleep(0.5)
            if backup_folder is not None:
                subprocess.run(['cp', str(file_path), str(backup_folder)])
            return True
        else:
            failed_attemps += 1
            time.sleep(0.5)

        if failed_attemps > max_attemps:
            raise TimeoutError("Camera is not capturing")

def check_capture_size(capture_folder: Path, name: str) -> bool:
    file_path = capture_folder.joinpath(name)
    if file_path.stat().st_size < 18495300 or file_path.stat().st_size > 18495400:
        raise MemoryError
    else:
        return True

def memory_check_events(
    memory_l25_event: threading.Event,
    memory_l50_event: threading.Event,
    memory_l75_event: threading.Event,
    memory_full_event: threading.Event):
    
    memory_gb_limit = 62
    bytes_used = float(subprocess.check_output(
        ['du','-sb', '/home/bentayga/bentayga/outputs']).split()[0].decode('utf-8'))
    if bytes_used < memory_gb_limit/4*1024**3:
        memory_l25_event.set()
        memory_l50_event.clear()
        memory_l75_event.clear()
        memory_full_event.clear()
    elif bytes_used >= memory_gb_limit/4*1024**3 and bytes_used < (memory_gb_limit/4)*2*1024**3:
        memory_l25_event.clear()
        memory_l50_event.set()
        memory_l75_event.clear()
        memory_full_event.clear()
    elif bytes_used >= (memory_gb_limit/4)*2*1024**3 and bytes_used < (memory_gb_limit/4)*3*1024**3:
        memory_l25_event.clear()
        memory_l50_event.clear()
        memory_l75_event.set()
        memory_full_event.clear()
    elif bytes_used > (memory_gb_limit/4)*3*1024**3:
        memory_l25_event.clear()
        memory_l50_event.clear()
        memory_l75_event.clear()
        memory_full_event.set()
      
def memory_check(control_event: threading.Event):
    memory_lsb = 18
    memory_msb = 16
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(memory_lsb, GPIO.OUT)
    GPIO.setup(memory_msb, GPIO.OUT)
    bytes_used = float(subprocess.check_output(
        ['du','-sb', '/home/bentayga/bentayga/outputs']).split()[0].decode('utf-8'))
    if bytes_used < 64/4*1024**3:
        GPIO.output(memory_lsb, GPIO.HIGH)
        GPIO.output(memory_msb, GPIO.HIGH)
    elif bytes_used >= 64/4*1024**3 and bytes_used < (64/4)*2*1024**3:
        GPIO.output(memory_lsb, GPIO.HIGH)
        GPIO.output(memory_msb, GPIO.LOW)
    elif bytes_used >= (64/4)*2*1024**3 and bytes_used < (64/4)*3*1024**3:
        GPIO.output(memory_lsb, GPIO.HIGH)
        GPIO.output(memory_msb, GPIO.LOW)
    elif bytes_used > (64/4)*3*1024**3:
        GPIO.output(memory_lsb, GPIO.LOW)
        GPIO.output(memory_msb, GPIO.LOW)
        control_event.set()

def gpio_watchdog_test(control_event: threading.Event):
    arduino_out = 12
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(arduino_out, GPIO.OUT)
    GPIO.output(arduino_out, GPIO.HIGH)
    try:
        heartbeat_lost = 0
        while not control_event.is_set():
            time.sleep(0.75)
            heartbeat_lost += 1
            if heartbeat_lost < 40:
                GPIO.output(arduino_out, GPIO.LOW)
                time.sleep(0.25)
                GPIO.output(arduino_out, GPIO.HIGH)
            else:
                control_event.set()
    finally:
        GPIO.output(arduino_out, GPIO.LOW)
        GPIO.cleanup()

def reboot_board():
    """Turn off all the GPIO and then reboot the board"""
    # Tu
    GPIO.cleanup()
    memory_lsb = 18
    memory_msb = 12
    camera_capturing_pin = 16
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(camera_capturing_pin, GPIO.OUT)
    GPIO.setup(memory_lsb, GPIO.OUT)
    GPIO.setup(memory_msb, GPIO.OUT)
    GPIO.output(camera_capturing_pin, GPIO.LOW)
    GPIO.output(memory_lsb, GPIO.LOW)
    GPIO.output(memory_msb, GPIO.LOW)
    GPIO.cleanup()
    
    command = 'reboot -h now'
    sudo_passwd = 'Bentayga1234'
    sudo_command = 'echo %s|sudo -S %s' % (sudo_passwd, command)
    try:
        p = os.system(sudo_command)

    except:
        logging.error('Trying to REBOOT again')
        os.popen("sudo -S %s"%(command), 'w').write(sudo_passwd)
    
    

def serial_watchdog(control_event: threading.Event):
    baudrate = 9600
    port = "/dev/ttyUSB0"
    arduino_interface = serial.Serial(port, baudrate)
    arduino_interface.open()
    time.sleep(1)
    if not arduino_interface.is_open:
        raise IOError(f"Error opening serial port with name {port}")
    try:
        heartbeat_lost = 0
        while not control_event.is_set():
            initial_time = time.time()
            while arduino_interface.in_waiting < 1:
                if (time.time() - initial_time) > 1.5:
                    heartbeat_lost += 1
                    break
            if arduino_interface.in_waiting > 1:
                heartbeat = arduino_interface.read(5).decode('utf-8')
                if heartbeat == "ACK?\n":
                    arduino_interface.write("ACK\n".encode('utf-8'))

            if heartbeat_lost >= 5: control_event.set()
    finally:
        arduino_interface.close()

