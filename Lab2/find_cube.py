import cv2
import numpy as np
import time

def filter_image(img, hsv_lower, hsv_upper):
    img_filt = cv2.GaussianBlur(img, (5,5),0)
    hsv = cv2.cvtColor(img_filt, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, hsv_lower, hsv_upper)
    return mask

    ###############################################################################
    ### You might need to change the parameter values to get better results
    ###############################################################################
def detect_blob(mask):
    img = cv2.medianBlur(mask, 15)
   # Set up the SimpleBlobdetector with default parameters.
    params = cv2.SimpleBlobDetector_Params()
    # Change thresholds
    params.minThreshold = 0;
    params.maxThreshold = 255;
    #filter by color (on binary)
    params.filterByColor = True
    params.blobColor = 255  # this looks at binary image 0 for looking for dark areas
    # Filter by Area.
    params.filterByArea = True
    params.minArea = 500

    params.filterByConvexity = True
    params.minConvexity = .25

    params.filterByCircularity = True
    params.minCircularity = .1

    params.filterByInertia = True
    params.minInertiaRatio = 0.1
    detector = cv2.SimpleBlobDetector_create(params)
    # Detect blobs.
    keypoints = detector.detect(img)
    return keypoints

def find_cube(img, hsv_lower, hsv_upper):
    """Find the cube in an image.
        Arguments:
        img -- the image
        hsv_lower -- the h, s, and v lower bounds
        hsv_upper -- the h, s, and v upper bounds
        Returns [x, y, radius] of the target blob, and [0,0,0] or None if no blob is found.
    """
    mask = filter_image(img, hsv_lower, hsv_upper)
    keypoints = detect_blob(mask)

    if len(keypoints) == 0:
        return None
    
    ###############################################################################
    # Todo: Sort the keypoints in a certain way if multiple key points get returned
    ###############################################################################

    keypoints = sorted(keypoints, key=lambda keypoint: keypoint.size, reverse=True)
    return [keypoints[0].pt[0], keypoints[0].pt[1], keypoints[0].size]

