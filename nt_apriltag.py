#!/usr/bin/env python3

import cv2
import numpy as np
from wpimath.geometry import Transform3d
import math
import time
from cscore import CameraServer as CS
import ntcore # network tables
import logging
import robotpy_apriltag

# This function is called once to initialize the apriltag detector and the pose estimator
def get_apriltag_detector_and_estimator(frame_size):
    detector = robotpy_apriltag.AprilTagDetector()
    # FRC 2023 uses tag16h5 (game manual 5.9.2)
    assert detector.addFamily("tag16h5")
    estimator = robotpy_apriltag.AprilTagPoseEstimator(
    robotpy_apriltag.AprilTagPoseEstimator.Config(
            0.2,
            500,
            500,
            frame_size[1] / 2.0,
            frame_size[0] / 2.0
        )
    )
    return detector, estimator
    
# This function is called for every detected tag. It uses the `estimator` to 
# return information about the tag, including its id, pose, and centerpoint.
# (The corners are also available.)
def process_apriltag(estimator, tag):
    tag_id = tag.getId()
    center = tag.getCenter()
    # hamming = tag.getHamming()
    # decision_margin = tag.getDecisionMargin()
    est = estimator.estimateOrthogonalIteration(tag, 50)
    return {'id':tag_id, 'pose':est.pose1, 'center':center}

# This function is called once for every frame captured by the Webcam. For testing, it can simply
# be passed a frame capture loaded from a file.
def detect_and_process_apriltag(frame, detector, estimator):
    assert frame is not None
    # Convert the frame to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # Detect apriltag
    tag_info = detector.detect(gray)
    DETECTION_MARGIN_THRESHOLD = 100
    filter_tags = [tag for tag in tag_info if tag.getDecisionMargin() > DETECTION_MARGIN_THRESHOLD]
    results = [ process_apriltag(estimator, tag) for tag in filter_tags ]
    return results

def setup_network_table_publishers(teamNumber, clientName, maxNumberOfAprilTags):
    global table
    logging.basicConfig(level=logging.DEBUG)
    instance = ntcore.NetworkTableInstance.getDefault()
    instance.setServerTeam(teamNumber)
    instance.startClient4(clientName)
    table = instance.getTable("/Apriltag")
    publishers = [ table.getDoubleArrayTopic("id_pose_center_" + str(id+1)).publish() for id in range(maxNumberOfAprilTags) ]
    [ publisher.setDefault([]) for publisher in publishers ]
    return publishers
    
def publish(publishers, results): # assume 'results' are output of detect_and_process_apriltag
    ids_not_seen = set(range(1, len(publishers)+1))
    for result in results:
        try:
            id = result['id'] # which apriltag
            ids_not_seen.remove(id)
            a = [ float(id) ] 
            po = result['pose']
            tr = po.translation()
            a.extend( [tr.x, tr.y, tr.z] ) # in meters
            # rotation() gives counterclockwise rotation angle around x, y, and z axes (in radians?)
            ro = po.rotation()
            a.extend( [ro.x, ro.y, ro.z] )
            ce = result['center']
            a.extend( [ce.x, ce.y] )
            publishers[result['id']-1].set(a)
        except:
            continue
    for id in ids_not_seen:
        try:
            publishers[id-1].set([]) # empty message means this id was not seen in this frame
        except:
            continue

#######
def main():
    outputImage = False
    maxNumberOfAprilTags = 30
    CS.enableLogging()

    # The Microsoft LifeCam HD-3000 has no unique id number
    camera = CS.startAutomaticCapture(name = "Micro$ HD 3000", path = "/dev/v4l/by-id/usb-Microsoft_Microsoft®_LifeCam_HD-3000-video-index0")
    frame_size = (640, 480) # in pixels
    # The built-in Raspberry Pi cameras and the Logitech cameras have a unique identifier - look in
    #   iPi's /dev/v4l/by-path directory to see what they are.  E.g.,
    # camera = CS.startAutomaticCapture(name = "rPi Internal Camera", path = "/dev/v4l/by-path/platform-3f801000.csi-video-index1")
    # camera = CS.startAutomaticCapture(name = "Logitech", path = "/dev/v4l/by-id/usb-046d_0809_B834D1B7-video-index0")
    camera.setResolution(frame_size[0], frame_size[1])

    # Get a CvSink. This will capture images from the camera
    cvSink = CS.getVideo()

    # (optional) Setup a CvSource. This will send images back to the Dashboard
    if (outputImage):
        outputStream = CS.putVideo("Raspberry Pi", frame_size[0], frame_size[1])

    # Allocating new images is very expensive, always try to preallocate
    img = np.zeros(shape=(frame_size[0], frame_size[1], 3), dtype=np.uint8)

    # Making apriltag detector is expensive
    detector, estimator = get_apriltag_detector_and_estimator(frame_size)

    global publishers
    publishers = setup_network_table_publishers(4173, "rPi", maxNumberOfAprilTags)

    prev_grab_time = 0
    while True:
        # Tell the CvSink to grab a frame from the camera and put it
        # in the source image.  If there is an error notify the output.
        grab_time, img = cvSink.grabFrame(img)
        if grab_time == 0:
            # Send the output the error.
            if (outputImage):
                outputStream.notifyError(cvSink.getError())
            # skip the rest of the current iteration
            continue

        print("delta(grab_time)=" + str( (grab_time-prev_grab_time)/1e6 ) + " s.")
        prev_grab_time = grab_time
        global res # for debugging
        results = detect_and_process_apriltag(img, detector, estimator)
        # normalize the within-frame coordinates to the range [-1,1]
        for result in results:
            result['center'].x = (result['center'].x - frame_size[0]/2) / (frame_size[0]/2)
            result['center'].y = (result['center'].y - frame_size[1]/2) / (frame_size[1]/2)
        publish(publishers, results)

        # Give the output stream a new image to display
        if (outputImage):
            outputStream.putFrame(img)
        # If sleep is too long, you won't see the decorated image in img
        # time.sleep(.020)


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
