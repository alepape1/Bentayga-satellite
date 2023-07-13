import time
import argparse
from pathlib import Path
from multispectral_cam import*
import logging

def configure_logging(log_file: Path, log_level=logging.DEBUG):
    if not log_file.parent.is_dir():
        logging.warning(f"{log_file.parent} directory does not exist.")
        logging.info(f"Creating directory {log_file.parent}.")
        log_file.parent.mkdir()

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
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("gain_range", help="Camera gain", type=int, action="store")
    parser.add_argument("exp_time_range", help="Camera exposure time", type=int, action="store")
    parser.add_argument("fps", help="Camera capturing rate per seconds", type=int, default=9, action="store")
    parser.add_argument('-i', '--i2c-bus', help='Set i2c bus, for Jetson Nano it is 1, for Jetson Xavier NX it is 8.', type=int, nargs=None, required=True)
    parser.add_argument("data_capturing_path", help="Path where the output data is stored (mision directory)")
    args = parser.parse_args()

    data_capturing_path = Path(args.data_capturing_path)
    camera_log = data_capturing_path.joinpath('multi_camera.log')
    configure_logging(camera_log)
    logging.getLogger().setLevel(logging.DEBUG)
    multispectral_cam = Multi_Cam()
    gain_range = f"{args.gain_range} {args.gain_range}"
    exp_time_range = f"{args.exp_time_range} {args.exp_time_range}"
    fps = args.fps
    capturing_path = multispectral_cam.make_frame_folder(data_capturing_path, 0)