import os
import subprocess

# Directory containing the images
input_dir = '/home/enorda/Desktop/Auto12/images'

# Output video file path
output_video = '/home/enorda/Desktop/Auto12/output_video.mp4'

# Get all image files in the directory
image_files = [filename for filename in os.listdir(input_dir) if filename.startswith('OutputImage') and filename.endswith('.jpg')]

# Sort the image files based on frame number
image_files.sort(key=lambda x: int(x.split('OutputImage')[1].split('.jpg')[0]))

# Use ffmpeg to create video from images
ffmpeg_cmd = ['ffmpeg', '-framerate', '30', '-i', os.path.join(input_dir, 'OutputImage%d.jpg'), '-c:v', 'libx264', '-pix_fmt', 'yuv420p', output_video]

# Execute ffmpeg command
subprocess.run(ffmpeg_cmd)
