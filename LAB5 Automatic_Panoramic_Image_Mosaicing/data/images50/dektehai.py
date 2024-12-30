import cv2
import numpy as np
from scipy import signal
from matplotlib import pyplot as plt

def harris_detector(I, N=100, sigma=1):
    """
    Harris corner detector integrated into this script.
    Parameters:
        I: Grayscale image
        N: Number of corners to return
        sigma: Standard deviation for Gaussian smoothing
    Returns:
        corners_y, corners_x: y and x coordinates of detected corners
    """
    # Compute gradients
    Ix = signal.convolve2d(I, np.array([[-1, 0, 1], [-1, 0, 1], [-1, 0, 1]]) / 3, boundary='symm', mode='same')
    Iy = signal.convolve2d(I, np.array([[-1, -1, -1], [0, 0, 0], [1, 1, 1]]) / 3, boundary='symm', mode='same')

    # Compute products of derivatives
    Ix2 = Ix**2
    Iy2 = Iy**2
    Ixy = Ix * Iy

    # Gaussian filter for smoothing
    g = cv2.getGaussianKernel(ksize=3, sigma=sigma)
    g = g * g.T

    Sx2 = signal.convolve2d(Ix2, g, mode='same')
    Sy2 = signal.convolve2d(Iy2, g, mode='same')
    Sxy = signal.convolve2d(Ixy, g, mode='same')

    # Compute the response R for each pixel
    detM = Sx2 * Sy2 - Sxy**2
    traceM = Sx2 + Sy2
    R = detM - 0.04 * (traceM**2)

    # Threshold and get coordinates of corners
    R_flat = R.ravel()
    idx = np.argsort(R_flat)[-N:]  # N strongest corners
    corners_y, corners_x = np.unravel_index(idx, R.shape)
    
    return corners_y, corners_x

def orb_descriptor(image, corners):
    # Ensure the corner coordinates are converted to floats
    corners_y, corners_x = corners
    keypoints = [cv2.KeyPoint(float(x), float(y), 1) for y, x in zip(corners_y, corners_x)]
    orb = cv2.ORB_create()
    keypoints, descriptors = orb.compute(image, keypoints)
    return keypoints, descriptors

def stitch_images(images):
    # Start with the first image as the initial panorama
    panorama = images[0]
    
    for i in range(1, len(images)):
        image = images[i]
        gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        # Detect Harris corners on the current image
        corners_y, corners_x = harris_detector(gray_image)
        
        # Extract ORB descriptors around Harris corners
        keypoints, descriptors = orb_descriptor(gray_image, (corners_y, corners_x))
        
        # Detect and compute ORB features on the current panorama as well
        gray_panorama = cv2.cvtColor(panorama, cv2.COLOR_BGR2GRAY)
        pan_corners_y, pan_corners_x = harris_detector(gray_panorama)
        pan_keypoints, pan_descriptors = orb_descriptor(gray_panorama, (pan_corners_y, pan_corners_x))
        
        # Match descriptors between panorama and the new image
        bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
        matches = bf.match(descriptors, pan_descriptors)
        
        # Sort matches by distance
        matches = sorted(matches, key=lambda x: x.distance)
        
        # Check if there are enough good matches to continue stitching
        if len(matches) < 10:  # Adjust this threshold as needed
            print(f"Not enough matches found between images {i-1} and {i}. Skipping this image.")
            continue  # Skip this image if not enough matches are found

        # Get the matching keypoints
        src_pts = np.float32([keypoints[m.queryIdx].pt for m in matches]).reshape(-1, 1, 2)
        dst_pts = np.float32([pan_keypoints[m.trainIdx].pt for m in matches]).reshape(-1, 1, 2)
        
        # Find homography
        M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)
        
        # Warp the new image to align with the panorama
        h, w = panorama.shape[:2]
        warped_image = cv2.warpPerspective(image, M, (w + image.shape[1], h))
        
        # Overlay the current panorama onto the warped image
        warped_image[0:h, 0:w] = panorama
        
        # Update the panorama with the newly added image
        panorama = warped_image
    
    # Crop black regions from the final stitched panorama
    panorama = crop_black_regions(panorama)
    return panorama

def crop_black_regions(image):
    """
    Crop the black regions from the right side of the stitched image.
    """
    # Convert to grayscale and create a binary mask
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    _, thresh = cv2.threshold(gray, 1, 255, cv2.THRESH_BINARY)
    
    # Find the bounding box of the non-black area
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    x, y, w, h = cv2.boundingRect(contours[0])
    
    # Crop the image to this bounding box
    cropped_image = image[y:y+h, x:x+w]
    return cropped_image

# Load images and convert them to a list, ensuring all images exist
images = [cv2.imread(f'/home/pablo/imagestichingharris/images/{i}.png') for i in range(1, 6)]
images = [img for img in images if img is not None]  # Exclude any None entries

# Stitch images if images were successfully loaded
if len(images) < 2:
    print("Not enough images to stitch. Ensure at least two images are available.")
else:
    panorama = stitch_images(images)
    
    if panorama is not None:
        # Display the result
        plt.imshow(cv2.cvtColor(panorama, cv2.COLOR_BGR2RGB))
        plt.axis('off')
        plt.title("Panorama Image")
        plt.show()
    else:
        print("Failed to create panorama.")
