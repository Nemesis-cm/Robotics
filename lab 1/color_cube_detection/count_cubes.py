import cv2
import numpy as np

#TODO: Modify these values for yellow color range. Add thresholds for detecting green also.
yellow_lower = np.array([10,170,140])
yellow_upper = np.array([29,255,255])

green_lower = np.array([5, 17, 5])
green_upper = np.array([120, 255, 120])



#TODO: Change this function so that it filters the image based
# on color using the hsv range for each color.
def filter_image(img, hsv_lower, hsv_upper):
    img_filt = cv2.medianBlur(img, 5)
    img = cv2.cvtColor(img_filt, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(img, hsv_lower, hsv_upper)
    return mask

    
#TODO: Change the parameters to make blob detection more accurate.
#Hint: You might need to set some parameters to specify features such as color, size, and shape.
#The features have to be selected based on the application. 
def detect_blob(mask):
    img_filt = cv2.medianBlur(mask, 5)

    img = mask
   # Set up the SimpleBlobdetector with default parameters with specific values
    params = cv2.SimpleBlobDetector_Params()
    
    params.filterByArea = True
    params.minArea = 1200
    params.minThreshold = 10
    params.maxThreshold = 180

    params.filterByColor = True
    params.blobColor = 255

    params.filterByInertia = True
    params.minInertiaRatio = 0.01
    params.filterByConvexity = True
    params.minConvexity = 0.12
    params.filterByCircularity = False

    

    #ADD CODE HERE

    # builds a blob detector with the given parameters 
    detector = cv2.SimpleBlobDetector_create(params)

    # use the detector to detect blobs.
    keypoints = detector.detect(img)
    keypointsImage = cv2.drawKeypoints(
    img, keypoints,
    np.array([]), (0, 0, 255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    cv2.imshow("Blobs Detected", keypointsImage)
    return len(keypoints)

    
def count_cubes(img):
    mask_yellow = filter_image(img, yellow_lower, yellow_upper)
    mask_green = filter_image(img, green_lower, green_upper)
    num_yellow = detect_blob(mask_yellow)
    num_green = detect_blob(mask_green)

    #TODO: Modify to return number of detected cubes for both yellow and green (instead of 0)
    return num_yellow, 1
    return num_green, 1

