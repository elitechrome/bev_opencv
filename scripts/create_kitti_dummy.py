import cv2
import numpy as np
import sys

def create_kitti_dummy(filename):
    width = 1242
    height = 375
    
    # Create black image
    img = np.zeros((height, width, 3), dtype=np.uint8)
    
    # Fill sky
    cv2.rectangle(img, (0, 0), (width, int(height/2)), (200, 200, 255), -1) 
    # Fill ground
    cv2.rectangle(img, (0, int(height/2)), (width, height), (50, 50, 50), -1)
    
    # Draw Lane Lines (Approximate)
    # Left Lane
    cv2.line(img, (0, height), (int(width/2) - 50, int(height/2)), (255, 255, 255), 5)
    # Right Lane
    cv2.line(img, (width, height), (int(width/2) + 50, int(height/2)), (255, 255, 255), 5)
    
    # Save
    cv2.imwrite(filename, img)
    print(f"Generated synthetic KITTI image: {filename}")

if __name__ == "__main__":
    create_kitti_dummy("data/kitti_sample.png")
