
import time
import os
import logging
from pathlib import Path
import threading
import argparse
import numpy
import shlex, subprocess

# Adding custom modules
import sys
base_dir = Path(__file__).parent.parent.resolve()
sys.path.append(str(base_dir.joinpath("sensors/multispectral-arducam")))
sys.path.append(str(base_dir.joinpath("sensors/rtc-ds1307")))

from multispectral_cam import*
from ds1307_jetson import *
from watchdog import *

def configure_logging(log_file: Path, log_level=logging.DEBUG):
    if not os.path.isdir(log_file.parent):
        logging.warning(f"{log_file.parent} directory does not exist.")
        logging.info(f"Creating directory {log_file.parent}.")
        os.mkdir(log_file.parent)

    file_handler = logging.FileHandler(filename=log_file)
    terminal_handler = logging.StreamHandler()
    file_handler.setLevel(logging.DEBUG)
    terminal_handler.setLevel(log_level)
    terminal_handler.setFormatter(logging.Formatter(fmt='[%(levelname)s](%(filename)s@%(lineno)d in %(threadName)s):%(message)s'))
    logging.basicConfig(
        handlers=(file_handler, terminal_handler),
        level=logging.NOTSET,
        format='%(asctime)-15s [%(levelname)s](%(filename)s@%(funcName)s%(lineno)d%(threadName)s):%(message)s',
    )
def arguments_generator():
    # Set the arguments for the main script using argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("gain_range", help="Camera gain", type=int, action="store")
    parser.add_argument("exp_time_range", help="Camera exposure time", type=int, action="store")
    parser.add_argument("spf", help="Camera capturing rate in seconds", type=int, default=10, action="store")
    parser.add_argument('-i', '--i2c-bus', help='Set i2c bus, for Jetson Nano it is 1, for Jetson Xavier NX it is 8.', type=int, nargs=None, required=True)
    parser.add_argument("--data_capturing_path", help="Path to the outputs of the cubesat.")
    parser.add_argument(
        "--backup_path",
        help="Path to the folder in which the backups will be saved.",
        type=str,
        action="store"
    )
    parser.add_argument(
        "--calibration_path",
        help="Path to the folder in which the calibration images are stored.",
        type=str,
        action="store"
    )
    args = parser.parse_args()
    return args
    
def main(
        capturing_event: threading.Event,
        memory_l25_event: threading.Event,
        memory_l50_event: threading.Event,
        memory_l75_event: threading.Event,
        memory_full_event: threading.Event,
        error_event: threading.Event
    ):
    args = arguments_generator()
    # Flags used to control and monitorize the behaviour of the code.
    backup_data = False
    perform_calibration = False
    
    # Capturing flags
    period_between_captures = args.spf
    capturing_count = 0
    capture_timeout = 8
   
    # RTC Flags
    max_rtc_fails = 5
    rtc_incorrect_readings = 0
    
    # NDVI flags
    captures_until_ndvi = 30
    return_code = None
    ndvi_calculation = None
    finish = False
    
    
    

    # Checks if all the folders that were introduced as arguments are correct    
    data_capturing_path = Path(args.data_capturing_path)
    if not data_capturing_path.is_dir(): 
        print("[bentayga/src/main.py]: ERROR, mission data folder doesn't exist")
        exit()
        
    # There are two optional folders, one for the backups and another for the 
    # calibration. Check both and if some of it are correct, active new func-
    # tionality
    if args.backup_path is not None:
        backup_path = Path(args.backup_path)
        if not backup_path.is_dir(): 
            print("[bentayga/src/main.py]: ERROR, backup folder does not exist")
            print(
                "[bentayga/src/main.py]: WARNING, data will not be saved twice")
        else:
            backup_data = True
            
    if args.calibration_path is not None:
        calibration_path = Path(args.calibration_path)
        if not calibration_path.is_dir(): 
            print("[bentayga/src/main.py]: ERROR, calibration folder does not exist")
            print(
                "[bentayga/src/main.py]: WARNING, images won't be calibrated'")
        else:
            perform_calibration = True
            
    ndvi_script_path = base_dir.joinpath("bin","nv12toNDVI")
    if not ndvi_script_path.exists():
        print("[bentayga/src/main.py]: ERROR, ndvi script does not exist")
        print(f"[bentayga/src/main.py]: can't access {ndvi_script_path}'")
        exit()
        
    # The folder in which the images will be saved is named after the date when
    # it is created. If there is alredy a folder on that date, the following folders
    # will be named as "{date}_{number of folders}"
    print("[bentayga/src/main.py]: Creating RTC object")
    rtc = DS1307_JN(0)
    mission_folder_name = rtc.read_complete_date().replace('-', '_')
    mission_folder = data_capturing_path.joinpath(mission_folder_name)
    if mission_folder.is_dir():
        count = 0
        while True:
            if not Path(f"{mission_folder}_{count}").is_dir():
                mission_folder = Path(f"{mission_folder}_{count}")
                break
            else:
                count += 1
    print("[bentayga/src/main.py]: Creating mission folder")
    mission_folder.mkdir()
    
    if backup_data:
        mission_backup_folder = backup_path.joinpath(mission_folder_name)
        if mission_backup_folder.is_dir():
            count = 0
            while True:
                if not Path(f"{mission_backup_folder}_{count}").is_dir():
                    mission_backup_folder = Path(f"{mission_backup_folder}_{count}")
                    break
                else:
                    count += 1
        mission_backup_folder.mkdir()
    
    # Once the folder of the mission are created. The process starts by initializing
    # all the elements of the camera
    camera_log = mission_folder.joinpath("pandora.log")
    configure_logging(camera_log)
    logging.getLogger().setLevel(logging.DEBUG)
    
    logging.info("Initializating camera oject with parameters:")
    multi_camera = Multi_Cam()
    gain_range = f"{args.gain_range} {args.gain_range}"
    exp_time_range = f"{args.exp_time_range} {args.exp_time_range}"
    fps = 10 # It has to be set to 10
    parameters = "GAIN: \"%s\", EXP_TIME: \"%s\", FPS: %d" % (gain_range, exp_time_range, fps)
    logging.info(parameters)
    try:      
        while not finish:
            try:
                # Creates the folder in which the image will be stored
                logging.debug("Reading time throug RTC")
                last_capture_time = rtc.read_time_object()
                rtc_incorrect_readings = 0
                current_capture_folder = str(last_capture_time).replace(':', '_')
                logging.info("Capturing frame with timestamp: %s" % (current_capture_folder))
                capturing_path = multi_camera.make_time_frame_folder(
                    mission_folder, 
                    current_capture_folder
                )
                # Starts the capturing process. As the camera need some time to
                # capture the frame, the scripts waits until the capture process
                # is complete
                multi_camera.set_pipeline_to_single_capture(
                    fps, gain_range, exp_time_range, capturing_path)
                
                if multi_camera.capturing:
                    if backup_data:
                        current_backup_folder = mission_backup_folder.joinpath(current_capture_folder)
                        current_backup_folder.mkdir()
                        result = wait_for_capture(
                            capturing_path, 
                            "nv12frame", 
                            capture_timeout, 
                            current_backup_folder)
                    else:
                        result = wait_for_capture(
                            capturing_path, 
                            "nv12frame", 
                            capture_timeout)
                    if result:
                        capturing_count += 1
                        multi_camera.stop_capturing_rtc()
                        capturing_event.set()
                        logging.info("Capture took %02d seconds" % (rtc.read_time_object() - last_capture_time).seconds)
                    else:
                        capturing_event.clear()
                
                # When {captures_until_ndvi} number of captures are performed
                # the ndvi is calculated
                if capturing_count % captures_until_ndvi == 0:
                    check_memory_thread = threading.Thread(
                        target=memory_check_events,
                        args=(memory_l25_event,
                            memory_l50_event,
                            memory_l75_event,
                            memory_full_event,))
                    check_memory_thread.start()
                    
                    ndvi_file_path = f"{capturing_path.name}_ndvi/ndvi.pgm"
                    ndvi_path = capturing_path.parent.joinpath(ndvi_file_path)
                    
                    if backup_data:
                        ndvi_backup_folder_path = f"{capturing_path.name}_ndvi"
                        ndvi_backup_path = current_backup_folder.parent.joinpath(ndvi_backup_folder_path)
                        ndvi_backup_path.mkdir()
                        
                    command_line = f"{ndvi_script_path}"\
                        + f" {capturing_path.joinpath('nv12frame')} {ndvi_path}"
                    if perform_calibration:
                        command_line += " " + str(calibration_path)
                        
                    ndvi_calculation = subprocess.Popen(shlex.split(command_line))
            
            except ValueError as ve:
                rtc_incorrect_readings += 1
                error_msg = f"Invalid RTC read. Number of incorrect readings: {rtc_incorrect_readings}"
                logging.warning(error_msg)
                if rtc_incorrect_readings > max_rtc_fails:
                    logging.error("Critical problem with the RTC, reboting board ")
                    raise ve
                    
            except FileNotFoundError as fnfe:
                error_msg = f"Failed to create capturing folder at timestamp "\
                    + str(last_capture_time)
                logging.error(error_msg)
                raise fnfe
            
            except TimeoutError as te:
                error_msg = f"Failed to capture an image at timestamp "\
                    + str(last_capture_time)
                logging.error(error_msg)
                raise te
                
            except Exception as e:
                logging.error(
                    "Something failed while capturing frame at timestamp "\
                    + str(last_capture_time)
                )
                logging.error(e)
                raise e
            
            while (rtc.read_time_object() - last_capture_time).seconds < period_between_captures:
                if ndvi_calculation != None:
                    return_code = ndvi_calculation.poll()
                    if return_code != None:
                        if return_code == 0:
                            if backup_data:
                                if ndvi_backup_path.exists():
                                    subprocess.run(['cp', str(ndvi_path), str(ndvi_backup_path)])
                        elif return_code == 1:
                            logging.warning("Fail to calculate NDVI: error in input arguments")
                        elif return_code == 2:
                            logging.error("Fail to calculate NDVI: input image does not exist")
                        elif return_code == 3:
                            logging.warning("Fail to calculate NDVI: no calibration images")
                        elif return_code == 4:
                            logging.warning("Fail to calculate NDVI: calibration folder empty")

            
    except Exception as e:
        logging.error("Something failed in capturing loop")
        logging.error(e)
        if multi_camera.capturing:
            multi_camera.stop_capturing()
        error_event.set()
        raise e
        
    finally:
        if multi_camera.capturing:
            multi_camera.stop_capturing()
        error_event.set()
        time.sleep(1)
        
if __name__ == "__main__":
    # The following events are used to control the GPIO behaviour to notify
    # to the Arduino the state of the capturing process
    capturing_event = threading.Event()
    memory_l25_event = threading.Event()
    memory_l50_event = threading.Event()
    memory_l75_event = threading.Event()
    memory_full_event = threading.Event()
    error_event = threading.Event()
    
    camera_gpio_thread = threading.Thread(
        target=camera_gpio_control,
        args=(capturing_event,
            memory_l25_event,
            memory_l50_event,
            memory_l75_event,
            memory_full_event,
            error_event,))
    camera_gpio_thread.start()
    
    try:
        main(capturing_event,
            memory_l25_event,
            memory_l50_event,
            memory_l75_event,
            memory_full_event,
            error_event)
            
    except Exception as e:
        logging.error("Camera process stop due to:")
        logging.error(e)
        camera_gpio_thread.join()
        #reboot_board()
    finally:
        logging.info("Finishing capture process")
        camera_gpio_thread.join()
        #reboot_board()
        time.sleep(1)
        
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
