#!/usr/bin/env python
  
'''
Welcome to the ArUco Marker Generator!
  
This program:
  - Generates ArUco markers using OpenCV and Python
'''
  
from __future__ import print_function # Python 2/3 compatibility
import cv2 # Import the OpenCV library
import numpy as np # Import Numpy library
import random
import sys
  
# Project: ArUco Marker Generator
# Date created: 12/17/2021
# Python version: 3.8

# The different ArUco dictionaries built into the OpenCV library. 
ARUCO_DICT = {
  "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
  "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
  "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
  "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
  "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
  "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
  "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
  "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
  "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
  "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
  "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
  "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
  "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
  "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
  "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
  "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
  "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL
}
  
def main():
    desired_aruco_dictionary = "DICT_ARUCO_ORIGINAL"
    # Generate a random ID between 0 and 249 (standard for ORIGINAL/250)
    aruco_marker_id = random.randint(0, 249)
    output_filename = f"{desired_aruco_dictionary}_id{aruco_marker_id}.png"

    # Check that we have a valid ArUco marker
    if ARUCO_DICT.get(desired_aruco_dictionary, None) is None:
        print("[INFO] ArUCo tag of '{}' is not supported".format(
            desired_aruco_dictionary))
        return

    # Load the ArUco dictionary
    this_aruco_dictionary = cv2.aruco.getPredefinedDictionary(
        ARUCO_DICT[desired_aruco_dictionary]
    )

    print("[INFO] generating ArUCo tag type '{}' with ID '{}'".format(
        desired_aruco_dictionary, aruco_marker_id))
    print("[INFO] saved as '{}'".format(output_filename))
    print("[INFO] Press any key or Ctrl+C to close")

    # Generate marker
    this_marker = cv2.aruco.generateImageMarker(
        this_aruco_dictionary,
        aruco_marker_id,
        300
    )

    # Save and display
    cv2.imwrite(output_filename, this_marker)
    
    try:
        cv2.imshow("ArUco Marker", this_marker)
        # Wait until a key is pressed or window is closed
        while cv2.getWindowProperty("ArUco Marker", cv2.WND_PROP_VISIBLE) >= 1:
            keyCode = cv2.waitKeyEx(100)
            if keyCode != -1:
                break
    except KeyboardInterrupt:
        print("\n[INFO] Closing...")
    finally:
        cv2.destroyAllWindows()

if __name__ == '__main__':
  main()

   
if __name__ == '__main__':
  print(__doc__)
  main()
