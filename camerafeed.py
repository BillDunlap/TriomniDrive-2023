#!/usr/bin/env python3

import cv2
import numpy as np
from cscore import CameraServer
import cscore
import logging
import ntcore

def main():
    CameraServer.enableLogging()

    # The Microsoft LifeCam HD-3000 has no unique id number
    camera = CameraServer.startAutomaticCapture(name = "Microsoft_HD_3000", path = "/dev/v4l/by-id/usb-Microsoft_Microsoft®_LifeCam_HD-3000-video-index0")
    frame_size = (640, 480) # in pixels
    # The built-in Raspberry Pi cameras and the Logitech cameras have a unique identifier - look in
    #   iPi's /dev/v4l/by-path directory to see what they are.  E.g.,
    # camera = CS.startAutomaticCapture(name = "rPi Internal Camera", path = "/dev/v4l/by-path/platform-3f801000.csi-video-index1")
    # camera = CS.startAutomaticCapture(name = "Logitech", path = "/dev/v4l/by-id/usb-046d_0809_B834D1B7-video-index0")
    camera.setResolution(frame_size[0], frame_size[1])
    # camera.setVideoMode(cscore._cscore.VideoMode.PixelFormat.kMJPEG, 320, 240, 120)

    # Get a CvSink. This will capture images from the camera
    cvSink = CameraServer.getVideo()

    # (optional) Setup a CvSource. This will send images back to the Dashboard
    outputStream = CameraServer.putVideo("Raspberry Pi Camera", frame_size[0], frame_size[1])

    # Allocating new images is very expensive, always try to preallocate
    img = np.zeros(shape=(frame_size[0], frame_size[1], 3), dtype=np.uint8)

    team = 4173
    print("Setting up NetworkTables client for team " + str(team));
    ntinst = ntcore.NetworkTableInstance.getDefault()
    ntinst.startClient4("wpilibpi")
    ntinst.setServerTeam(team)
    # Is following line useful?
    # ntinst.startDSClient()

    prev_grab_time = 0
    while True:
        # Tell the CvSink to grab a frame from the camera and put it
        # in the source image.  If there is an error notify the output.
        grab_time, img = cvSink.grabFrame(img)
        if grab_time == 0:
            # Send the output the error.
            outputStream.notifyError(cvSink.getError())
        else:
            outputStream.putFrame(img)
            # print("delta time = " + str((grab_time - prev_grab_time)/1e6) + "")

if __name__ == "__main__":

    # To see messages from networktables, you must setup logging
    import logging

    logging.basicConfig(level=logging.DEBUG)

    # You should uncomment these to connect to the RoboRIO
    # import ntcore
    # nt = ntcore.NetworkTableInstance.getDefault()
    # nt.setServerTeam(XXXX)
    # nt.startClient4(__file__)

    main()
