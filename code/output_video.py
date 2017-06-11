#!/usr/bin/env python
import argparse
import imageio
imageio.plugins.ffmpeg.download()
from moviepy.editor import VideoFileClip, ImageSequenceClip
from os import listdir
from os.path import isfile, join


def write_video(image_folder, output_folder, video_name):
	image_files = [join(image_folder,f) for f in listdir(image_folder) if isfile(join(image_folder, f))]
	image_files = [f for f in image_files if f[-3:]=='jpg']

	#image_files.sort()

	clip = ImageSequenceClip(image_files, fps = 60)
	#new_clip = clip.fl_image(process_image)
	output_file = join(output_folder,video_name)
	print(output_file)
	clip.write_videofile(output_file, audio = False)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Write Video')
    parser.add_argument(
        'image_folder',
        type=str,
        nargs='?',
        default='',
        help='Path to image folder. This is where the images from the run are found.'
    )
    parser.add_argument(
    	'output_folder',
    	type=str,
        nargs='?',
        default='',
        help='Path to image folder. This is where the video of the run will be saved.')
    parser.add_argument(
    	'video_name',
    	type=str,
        nargs='?',
        default='',
        help='Path to image folder. This is where the video of the run will be saved.')
    args = parser.parse_args()

    write_video(args.image_folder, args.output_folder, args.video_name)