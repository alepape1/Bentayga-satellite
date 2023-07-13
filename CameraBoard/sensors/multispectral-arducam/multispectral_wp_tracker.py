import time
import os
import sysv_ipc
import logging
import asyncio
import argparse
import numpy
from multispectral_cam import*
from pathlib import Path

def configure_logging(log_file, log_level=logging.DEBUG):
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

async def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("gain_range", help="Camera gain", type=int, action="store")
    parser.add_argument("exp_time_range", help="Camera exposure time", type=int, action="store")
    parser.add_argument("fps", help="Camera capturing rate per seconds", type=int, default=9, action="store")
    parser.add_argument('-i', '--i2c-bus', help='Set i2c bus, for Jetson Nano it is 1, for Jetson Xavier NX it is 8.', type=int, nargs=None, required=True)
    parser.add_argument("data_capturing_path", help="Path where the output data is stored (mision directory)")
    parser.add_argument("shm_key", help="Shared memory key for controlling the capturing state")
    args = parser.parse_args()

    data_capturing_path = args.data_capturing_path
    camera_log = Path(os.path.join(data_capturing_path, 'Multispectral_camera.log'))
    configure_logging(camera_log)
    logging.getLogger().setLevel(logging.DEBUG)
    shm_key = int(args.shm_key)

    # Notify that the process correctly started
    logging.info('SUCCESS (multispectral_wp_tracker.py): Started OK')

    try:
        shmPointer_capturingStatus = sysv_ipc.SharedMemory(shm_key, sysv_ipc.IPC_CREAT, size=2)
    except Exception as e:
        logging.error(e)
        exit()
    # Generate the outptu directory and file
    #    Verify that the mission directory exists
    logging.info(f"Verifiying if {data_capturing_path} exists")
    if not os.path.isdir(data_capturing_path):
            logging.error('multispectral_wp_tracker.py): The mission directory does not exists: ({}) \n'.format(data_capturing_path))
            exit()

    Multispectral_cam = Multi_Cam()
    gain_range = f"{args.gain_range} {args.gain_range}"
    exp_time_range = f"{args.exp_time_range} {args.exp_time_range}"
    fps = args.fps
    cam_path = args.data_capturing_path
    capturing_task = None 
    last_waypoint = -1

    finish = False
    try:
        while not finish:
            # Get the values from the shared memory
            array_value = shmPointer_capturingStatus.read(2)
            numpy_value = numpy.frombuffer(array_value, dtype=numpy.uint8)
            current_wp = numpy_value[0]
            current_action = numpy_value[1]
            if current_action == 0:
                if Multispectral_cam.capturing:
                    Multispectral_cam.stop_capturing()
                    #await capturing_task
            if current_action == 1:
                #print(cam_path)    # Checks if the camera thread is active
                if not Multispectral_cam.capturing:
                    try:
                        # Initialize the folder and the log file for the current
                        # way_point
                        capturing_path = Multispectral_cam.make_frame_folder(cam_path, current_wp)
                        #await asyncio.sleep(2)
                        # Launch the thread that saves the frames as binary files in
                        # the specified folder
                        capturing_task = asyncio.create_task(Multispectral_cam.capture(fps, gain_range, exp_time_range, capturing_path, current_wp))
                        # Save the current way_point
                        await capturing_task
                        last_waypoint = current_wp

                    except Exception as e:
                        print(f"\n 1 ERROR (multispectral_wp_tracker.py - Task): {e}")

                # Checks if the way_point has been changed    
                if current_wp != last_waypoint:
                    try:
                        if Multispectral_cam.capturing:
                            Multispectral_cam.stop_capturing()
                        #

                            # Initialize the process again but saving the frames in the
                            # correct folder
                            capturing_path = Multispectral_cam.make_frame_folder(cam_path, current_wp)
                            capturing_task = asyncio.create_task(Multispectral_cam.capture(fps, gain_range, exp_time_range, capturing_path, current_wp))
                            await capturing_task
                        # Save the current way_point
                            last_waypoint = current_wp

                    except Exception as e:
                        print(f"\n 2 ERROR (multispectral_wp_tracker.py - Update Task): {e}")

            # Stops the capturing proccess and end the loop
            if current_action >= 2:
                if Multispectral_cam.capturing:
                    Multispectral_cam.stop_capturing()
        #await capturing_task

                finish = True

                # Sleep
            await asyncio.sleep(0.25)
            
    except Exception as e:
        print(f"\n 3 ERROR (multispectral_wp_tracker.py - Loop): {e}")
        if Multispectral_cam.capturing:
            Multispectral_cam.stop_capturing()
        #await capturing_task
        
if __name__ == "__main__":
    asyncio.run(main())
