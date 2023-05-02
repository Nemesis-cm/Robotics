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

YELLOW_LOWER = np.array([79,116,188])
YELLOW_UPPER = np.array([116, 180, 255])
RED_LOWER = np.array([116,116,188])
RED_UPPER = np.array([143, 180, 255])



class redCube:

    def getName(self):
        return "red"

    def run(self, item: cozmo.robot.Robot):
           # print("x")


            image_settings = [("ExampleTasks/RED.png", Image.NEAREST)]
            face_images =[]
            for image_name, resampling_mode in image_settings:
                image = Image.open(image_name)

                # resize to fit on Cozmo's face screen
                resized_image = image.resize(cozmo.oled_face.dimensions(), resampling_mode)

                # convert the image to the format used by the oled screen
                face_image = cozmo.oled_face.convert_image_to_screen_data(resized_image,
                                                                          invert_image=True)
                face_images.append(face_image)
            for image in face_images:
                item.display_oled_face_image(image, 1000.0).wait_for_completed(timeout=20)

             #   time.sleep(duration_s)
            #item.display_oled_face_image(Image.open("/home/jwjibilian/Desktop/RobotsLab3/CS3630Lab3/ExampleTasks/Untitled.png"),1000)
            event = item.world.wait_for(cozmo.camera.EvtNewRawCameraImage, timeout=30)   #get camera image
            if event.image is not None:
                image = np.asarray(event.image)#cv2.cvtColor(np.asarray(event.image), cv2.COLOR_BGR2RGB)


                if 1:
                    item.camera.enable_auto_exposure = True
                else:
                    item.camera.set_manual_exposure(exposure,fixed_gain)

                        #find the cube
                cubeY = find_cube(image, YELLOW_LOWER, YELLOW_UPPER)
                cubeR = find_cube(image, RED_LOWER, RED_UPPER)
                if (cubeY is not None) & (cubeR is not None) :
                    return("both", item)
                elif cubeY is not None :
                    return("yellow", item)
                elif cubeR is not None:
                    return ("red", item)
        #         print(cube)
                 #i = 0
                 #while not cube and i < 5:
                     #cube = find_cube(image, YELLOW_LOWER, YELLOW_UPPER)
        # #            print(cube)
        #             #BoxAnnotator.cube = cube
             #        i+=1
            return "findCube", item