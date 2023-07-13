import os, shutil
import asyncio
from datetime import datetime
import gi
import time
from pathlib import Path
import logging
gi.require_version("Gst", "1.0")

from gi.repository import Gst, GObject

DEF_LOG_FOLDER  = "cameraLog"

class Multi_Cam():

    def __init__(self):

        #self.id          = devId
        GObject.threads_init()
        Gst.init(None)
        self.pipeline = None
        self.capturing   = False
        self._log_folder = None
    
    def __repr__(self):
        return self.__str__()

    def print_current_values(self):
        """
        Summary:
            Dummy function used to debug
        Arguments:
            N/A
        Returns:
            N/A
        """
        pass
        
    def stop_capturing_rtc(self):
        """
        Summary:
            Stops the capturing process and close the log file
        Arguments:
            N/A
        Returns:
            N/A
        """
        self.pipeline.set_state(Gst.State.NULL)
        self.capturing  = False

        
    def stop_capturing(self):
        """
        Summary:
            Stops the capturing process and close the log file
        Arguments:
            N/A
        Returns:
            N/A
        """
        #print("Llega")
        #loop = asyncio.get_running_loop()
        
        #mainloop.quit()
        #loop.quit()
        self.pipeline.set_state(Gst.State.NULL)
        self.capturing  = False
        #self.pipeline.get_property("gain_range")
        time.sleep(2)

    def set_pipeline(self,
            framerate: int,
            gain_range: str,
            exp_time_range: str, 
            location: Path, 
            way_point):
        self.capturing  = True
        self.pipeline = Gst.Pipeline()

        fps = framerate
        videosrc = Gst.ElementFactory.make('nvarguscamerasrc', "videosrc")
        videosrc.set_property("gainrange", gain_range)
        videosrc.set_property("ispdigitalgainrange", "1 1")
        videosrc.set_property("aelock", "true")
        videosrc.set_property("awblock", "true")
        videosrc.set_property("exposuretimerange", exp_time_range)
        loct = location.joinpath("capture%02d")

        sink = Gst.ElementFactory.make("multifilesink", 'sink')
        sink.set_property("location", str(loct))
        caps = Gst.Caps.from_string("video/x-raw(memory:NVMM),format=NV12,width=4056,height=3040,framerate={}/1".format(fps))
        capsfilter = Gst.ElementFactory.make("capsfilter")
        capsfilter.props.caps = caps
        
        converter = Gst.ElementFactory.make("nvvidconv","converter")

        capsn = Gst.Caps.from_string("video/x-raw,format=NV12")
        capsfilt = Gst.ElementFactory.make("capsfilter")
        capsfilt.props.caps = capsn
        
        self.pipeline.add(videosrc)
        self.pipeline.add(sink)
        self.pipeline.add(capsfilter)
        self.pipeline.add(converter)
        self.pipeline.add(capsfilt)

        videosrc.link(capsfilter)
        capsfilter.link(converter)
        converter.link(capsfilt)
        capsfilt.link(sink)
        
        self.pipeline.set_state(Gst.State.PLAYING)
    
    def set_pipeline_to_single_capture(self,
            framerate: int,
            gain_range: str,
            exp_time_range: str, 
            location: Path):
        self.capturing  = True
        self.pipeline = Gst.Pipeline()

        videosrc = Gst.ElementFactory.make('nvarguscamerasrc', "videosrc")
        videosrc.set_property("gainrange", gain_range)
        videosrc.set_property("ispdigitalgainrange", "1 1")
        videosrc.set_property("aelock", "true")
        videosrc.set_property("awblock", "true")
        videosrc.set_property("exposuretimerange", exp_time_range)

        loct = location.joinpath("nv12frame")

        sink = Gst.ElementFactory.make("multifilesink", 'sink')
        sink.set_property("location", str(loct))
        camera_parameters = "video/x-raw(memory:NVMM),format=NV12"
        camera_parameters += f",width=4056,height=3040,framerate={framerate}/1"
        caps = Gst.Caps.from_string(camera_parameters)
        capsfilter = Gst.ElementFactory.make("capsfilter")
        capsfilter.props.caps = caps
        
        converter = Gst.ElementFactory.make("nvvidconv","converter")

        capsn = Gst.Caps.from_string("video/x-raw,format=NV12")
        capsfilt = Gst.ElementFactory.make("capsfilter")
        capsfilt.props.caps = capsn
        
        self.pipeline.add(videosrc)
        self.pipeline.add(sink)
        self.pipeline.add(capsfilter)
        self.pipeline.add(converter)
        self.pipeline.add(capsfilt)

        videosrc.link(capsfilter)
        capsfilter.link(converter)
        converter.link(capsfilt)
        capsfilt.link(sink)
        
        self.pipeline.set_state(Gst.State.PLAYING)
        self.capturing = True

    def make_time_frame_folder(
            self, camera_path: Path, current_time: str) -> Path:
        """
        Summary:
            Make the folder in which will be stored all the frames captured
            in the current way_point
        Arguments:
            camera_path  -- Path to the folder of the camera
            way_point    -- Current way_point of the mission
        Returns:
            frame_folder -- Path to the folder used to store the frames 
        """
        if not camera_path.is_dir():
            raise Exception("Path to the camera folder not found")

        frame_folder = camera_path.joinpath(current_time)
        if frame_folder.is_dir():
            count = 0
            while True:
                if not frame_folder.joinpath(f"_{count}").is_dir():
                    frame_folder = frame_folder.joinpath(f"_{count}")
                    break
                else:
                    count +=1
        frame_folder.mkdir()

        return frame_folder

    def make_frame_folder(self, camera_path, way_point):
        """
        Summary:
            Make the folder in which will be stored all the frames captured
            in the current way_point
        Arguments:
            camera_path  -- Path to the folder of the camera
            way_point    -- Current way_point of the mission
        Returns:
            frame_folder -- Path to the folder used to store the frames 
        """
        if not os.path.isdir(camera_path):
            raise Exception("Path to the camera folder not found")

        frame_folder = os.path.join(camera_path, str(way_point))
        os.mkdir(frame_folder)

        return frame_folder
           
    async def capture(self, framerate, gain_range, exp_time_range, location, way_point):
        self.capturing = True
        loop = asyncio.get_running_loop()
        time_stamp = await loop.run_in_executor(None, self.set_pipeline, framerate, gain_range, exp_time_range, location, way_point)

if __name__ == "__main__":
    MECam = Multi_Cam()
    MECam.print_current_values()
