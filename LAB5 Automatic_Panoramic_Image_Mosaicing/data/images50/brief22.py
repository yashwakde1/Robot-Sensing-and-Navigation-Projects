import numpy as np
import cv2
import matplotlib.pyplot as plt
import glob

def resize_image(image, width=1500, height=1500):
    return cv2.resize(image, (width, height), interpolation=cv2.INTER_AREA)

def harris_corner_detector(image, block_size=2, ksize=3, k=0.04, threshold=0.01):
    gray = np.float32(image)
    dst = cv2.cornerHarris(gray, block_size, ksize, k)
    dst = cv2.dilate(dst, None)
    corners = np.argwhere(dst > threshold * dst.max())
    keypoints = [cv2.KeyPoint(float(pt[1]), float(pt[0]), 1) for pt in corners]
    return keypoints

def extract_and_match_features(image1, image2):
    keypoints1 = harris_corner_detector(cv2.cvtColor(image1, cv2.COLOR_BGR2GRAY))
    keypoints2 = harris_corner_detector(cv2.cvtColor(image2, cv2.COLOR_BGR2GRAY))

    brief = cv2.xfeatures2d.BriefDescriptorExtractor_create()
    keypoints1, descriptors1 = brief.compute(image1, keypoints1)
    keypoints2, descriptors2 = brief.compute(image2, keypoints2)

    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
    matches = bf.match(descriptors1, descriptors2)
    matches = sorted(matches, key=lambda x: x.distance)
    return keypoints1, keypoints2, matches

def detect_and_compute_keypoints(img):
    """
    Detect keypoints and compute descriptors for each image using SIFT.
    """
    sift = cv2.SIFT_create()
    keypoints, descriptors = sift.detectAndCompute(img, None)
    return keypoints, descriptors

def stitch_images_with_sift(img1, img2):
    """
    Stitch two images together using matched keypoints and RANSAC to find homography.
    """
    # Detect SIFT keypoints and descriptors
    keypoints1, descriptors1 = detect_and_compute_keypoints(cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY))
    keypoints2, descriptors2 = detect_and_compute_keypoints(cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY))
    
    # Match features
    index_params = dict(algorithm=1, trees=5)
    search_params = dict(checks=50)
    flann = cv2.FlannBasedMatcher(index_params, search_params)
    matches = flann.knnMatch(descriptors1, descriptors2, k=2)

    # Filter matches using the ratio test
    good_matches = []
    for m, n in matches:
        if m.distance < 0.75 * n.distance:
            good_matches.append(m)
    
    # Draw matches for visualization
    draw_matches(img1, img2, keypoints1, keypoints2, good_matches)
    
    # Extract matched keypoint coordinates
    points1 = np.float32([keypoints1[m.queryIdx].pt for m in good_matches]).reshape(-1, 1, 2)
    points2 = np.float32([keypoints2[m.trainIdx].pt for m in good_matches]).reshape(-1, 1, 2)

    # Compute homography with RANSAC
    H, mask = cv2.findHomography(points2, points1, cv2.RANSAC, 5.0)
    if H is None:
        print("Homography calculation failed.")
        return img1  # Return the base image if homography fails

    height, width = img1.shape[:2]
    panorama = cv2.warpPerspective(img2, H, (width * 2, height))
    panorama[0:height, 0:width] = img1
    return panorama

def stitch_multiple_images(image_files):
    # Load and resize the first image
    panorama = cv2.imread(image_files[0])
    panorama = resize_image(panorama, width=1500, height=1500)

    # Iterate over the remaining images and stitch them one by one using SIFT-based stitching
    for i, image_file in enumerate(image_files[1:], start=2):
        next_image = cv2.imread(image_file)
        next_image = resize_image(next_image, width=750, height=100)

        # Stitch the current panorama with the next image using SIFT logic
        print(f"Stitching image {i}...")
        panorama = stitch_images_with_sift(panorama, next_image)

    return panorama

def draw_matches(image1, image2, keypoints1, keypoints2, matches, num_matches=30):
    matched_image = cv2.drawMatches(image1, keypoints1, image2, keypoints2, matches[:num_matches], None, flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
    plt.figure(figsize=(10, 5))
    plt.imshow(cv2.cvtColor(matched_image, cv2.COLOR_BGR2RGB))
    plt.title("Feature Matches")
    plt.axis("off")
    plt.show()

if __name__ == "__main__":
    image_dir = '/home/pablo/imagestichingharris/images'
    image_files = sorted(glob.glob(f'{image_dir}/*.jpeg'))

    # Stitch all images to create the panorama using SIFT-based stitching logic
    panorama = stitch_multiple_images(image_files)

    # Resize the final panorama for display
    panorama_display = cv2.resize(panorama, (1500, 750))

    # Display and save the result
    cv2.imshow('Panorama', panorama_display)
    cv2.imwrite('panorama.jpg', panorama)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
