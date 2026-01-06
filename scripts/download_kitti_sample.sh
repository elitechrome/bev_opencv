#!/bin/bash
# Download a sample KITTI image from GitHub raw content
# Source: https://github.com/yanii/kitti-pcl/blob/master/KITTI_data/2011_09_26/2011_09_26_drive_0005_sync/image_02/data/0000000000.png

# Alternative reliable source (Kitti Sample from a repo)
IMAGE_URL="https://raw.githubusercontent.com/yanii/kitti-pcl/master/KITTI_data/2011_09_26/2011_09_26_drive_0005_sync/image_02/data/0000000000.png"
OUTPUT_FILE="kitti_sample.png"

echo "Downloading KITTI sample image..."
curl -L -o "$OUTPUT_FILE" "$IMAGE_URL" --fail

if [ $? -eq 0 ]; then
    echo "Download successful: $OUTPUT_FILE"
else
    echo "Download failed! Creating a dummy image for testing..."
    # Create a dummy image using ImageMagick if available or just touch it
    convert -size 1242x375 xc:gray "$OUTPUT_FILE" 2>/dev/null || touch "$OUTPUT_FILE"
fi
