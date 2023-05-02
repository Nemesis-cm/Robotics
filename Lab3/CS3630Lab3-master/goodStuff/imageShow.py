import cozmo
import cozmo.util
import sys
import numpy as np
import time

from find_cube import find_cube
fixed_gain,exposure,mode = 3.90,67,1
try:
    from PIL import ImageDraw, ImageFont, Image
except ImportError:
    sys.exit('run `pip3 install --user Pillow numpy` to run this example')

def showImage(info, image):
    image_settings = [(image, Image.NEAREST)]
    face_images = []
    for image_name, resampling_mode in image_settings:
        image = Image.open(image_name)

        # resize to fit on Cozmo's face screen
        resized_image = image.resize(cozmo.oled_face.dimensions(), resampling_mode)

        # convert the image to the format used by the oled screen
        face_image = cozmo.oled_face.convert_image_to_screen_data(resized_image,
                                                                  invert_image=True)
        face_images.append(face_image)
    for image in face_images:
        info.display_oled_face_image(image, 2000.0).wait_for_completed(timeout=3)