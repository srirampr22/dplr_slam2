#!/usr/bin/env python3

import cv2
# import apriltag
from pupil_apriltags import Detector

# Initialize video capture from the webcam
cap = cv2.VideoCapture(8)

# Initialize the AprilTag detector
# options = apriltag.DetectorOptions(families='tag16h5')
# detector = apriltag.Detector(options)

at_detector = Detector(families='tag16h5',nthreads=1, quad_decimate=1.0, quad_sigma=0.0, refine_edges=1, decode_sharpening=0.25,debug=0)



# result =  at_detector.detect(gray_img,estimate_tag_pose=False, camera_params=K, tag_size=0.147)
# detector = 


while True:
    # Read frame from the webcam
    ret, frame = cap.read()
    if not ret:
        break

    # Convert the frame to grayscale (required for AprilTag detection)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Detect AprilTags in the image
    # results = detector.detect(gray)
    K = ([594.5518798828125/1000, 594.532958984375/1000, 327.4517517089844/1000, 240.4869384765625/1000])

    results =  at_detector.detect(gray,estimate_tag_pose=True, camera_params=K, tag_size=0.147)
    print(results)

    # Draw a rectangle around each detected tag
    for r in results:
        (ptA, ptB, ptC, ptD) = r.corners
        ptA = (int(ptA[0]), int(ptA[1]))
        ptB = (int(ptB[0]), int(ptB[1]))
        ptC = (int(ptC[0]), int(ptC[1]))
        ptD = (int(ptD[0]), int(ptD[1]))

        cv2.line(frame, ptA, ptB, (0, 255, 0), 2)
        cv2.line(frame, ptB, ptC, (0, 255, 0), 2)
        cv2.line(frame, ptC, ptD, (0, 255, 0), 2)
        cv2.line(frame, ptD, ptA, (0, 255, 0), 2)

        # Put the tag ID near the center
        tag_id = str(r.tag_id)
        cv2.putText(frame, tag_id, (int(r.center[0]), int(r.center[1])), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    # Display the result
    cv2.imshow("AprilTag Detection", frame)

    # Break the loop if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the webcam and close all OpenCV windows
cap.release()
cv2.destroyAllWindows()
