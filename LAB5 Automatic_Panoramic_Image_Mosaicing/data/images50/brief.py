import cv2
import numpy as np
import os
from glob import glob

# Path to the image folder
image_folder = "/home/pablo/imagestichingharris/images"

# Parameters for Harris Corner Detector
blockSize = 10
ksize = 25
k = 0.04

def harris_corners(img):
    """Detects Harris corners and returns keypoints for feature matching."""
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    gray = np.float32(gray)
    harris_response = cv2.cornerHarris(gray, blockSize, ksize, k)
    harris_response = cv2.dilate(harris_response, None)
    
    # Threshold for detecting strong corners
    corners = np.argwhere(harris_response > 0.01 * harris_response.max())
    keypoints = [cv2.KeyPoint(float(x[1]), float(x[0]), 1) for x in corners]
    
    # Compute descriptors for keypoints using BRIEF (or ORB)
    brisk = cv2.BRISK_create()
    keypoints, descriptors = brisk.compute(img, keypoints)
    return keypoints, descriptors

def stitch_images(img1, img2):
    """Stitches two images using Harris corners and homography."""
    # Detect keypoints and descriptors with Harris corner detector
    kp1, des1 = harris_corners(img1)
    kp2, des2 = harris_corners(img2)
    
    # Match descriptors using BFMatcher
    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
    matches = bf.match(des1, des2)
    matches = sorted(matches, key=lambda x: x.distance)
    
    # Extract matched keypoints
    src_pts = np.float32([kp1[m.queryIdx].pt for m in matches]).reshape(-1, 1, 2)
    dst_pts = np.float32([kp2[m.trainIdx].pt for m in matches]).reshape(-1, 1, 2)
    
    # Find homography and warp images
    H, _ = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)
    height, width = img2.shape[:2]
    panorama = cv2.warpPerspective(img1, H, (width + img1.shape[1], height))
    panorama[0:height, 0:width] = img2
    
    return panorama

def load_images(image_folder):
    """Loads images from a folder in sorted order."""
    image_paths = sorted(glob(os.path.join(image_folder, "*.png")))
    images = [cv2.imread(img) for img in image_paths]
    return images

def create_panorama(images):
    """Creates a panorama by stitching multiple images."""
    panorama = images[0]
    for i in range(1, len(images)):
        panorama = stitch_images(panorama, images[i])
    return panorama

# Load images from folder
images = load_images(image_folder)

# Create panorama
panorama = create_panorama(images)

# Save and display the result
cv2.imwrite("panorama.png", panorama)
cv2.imshow("Panorama", panorama)
cv2.waitKey(0)
cv2.destroyAllWindows()
