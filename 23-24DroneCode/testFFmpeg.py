# List of input image files
image_files = ['/home/enorda/Desktop/Auto12/DetectedMarkerImage1.jpg',
 '/home/enorda/Desktop/Auto12/DetectedMarkerImage2.jpg']  # Replace with your image filenames
import subprocess

# List of input image files
#image_files = ['image1.jpg', 'image2.jpg', 'image3.jpg', ...]  # Replace with your image filenames

# Output video file
output_video_file = 'output_video.mp4'

# Define FFmpeg command
ffmpeg_cmd = [
    'ffmpeg',                # FFmpeg executable
    '-framerate', '25',      # Frame rate
    '-i', 'DetectedMarkerImage%d.jpg',    # Input image pattern
    '-c:v', 'libx264',       # Video codec
    '-pix_fmt', 'yuv420p',   # Pixel format for compatibility
    output_video_file       # Output video file
]

# Run FFmpeg command
subprocess.run(ffmpeg_cmd,stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

print("Video saved successfully!")
