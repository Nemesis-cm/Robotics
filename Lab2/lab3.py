#!/usr/bin/env python3
#!c:/Python35/python3.exe -u
import asyncio
import sys
import cv2
import numpy as np
import cozmo
import time
import os
from glob import glob
from find_cube import *
try:
    from PIL import ImageDraw, ImageFont
except ImportError:
    sys.exit('run `pip3 install --user Pillow numpy` to run this example')
def nothing(x):
    pass
YELLOW_LOWER = np.array([9, 115, 151])
YELLOW_UPPER = np.array([179, 215, 255])
GREEN_LOWER = np.array([14,21,0])
GREEN_UPPER = np.array([56, 205, 52])

from cozmo.util import degrees, distance_mm, speed_mmps

try:
    from PIL import ImageDraw, ImageFont
except ImportError:
    sys.exit('run `pip3 install --user Pillow numpy` to run this example')
def nothing(x):
    pass

YELLOW_LOWER = np.array([9, 115, 151])
YELLOW_UPPER = np.array([179, 215, 255])

# Define a decorator as a subclass of Annotator; displays the keypoint
class BoxAnnotator(cozmo.annotate.Annotator):

    cube = None

    def apply(self, image, scale):
        d = ImageDraw.Draw(image)
        bounds = (0, 0, image.width, image.height)

        if BoxAnnotator.cube is not None:

            #double size of bounding box to match size of rendered image
            BoxAnnotator.cube = np.multiply(BoxAnnotator.cube,2)

            #define and display bounding box with params:
            #msg.img_topLeft_x, msg.img_topLeft_y, msg.img_width, msg.img_height
            box = cozmo.util.ImageBox(BoxAnnotator.cube[0]-BoxAnnotator.cube[2]/2,
                                      BoxAnnotator.cube[1]-BoxAnnotator.cube[2]/2,
                                      BoxAnnotator.cube[2], BoxAnnotator.cube[2])
            cozmo.annotate.add_img_box_to_image(image, box, "yellow", text=None)

            BoxAnnotator.cube = None

async def run(robot: cozmo.robot.Robot):

    robot.world.image_annotator.annotation_enabled = False
    robot.world.image_annotator.add_annotator('box', BoxAnnotator)

    robot.camera.image_stream_enabled = True
    robot.camera.color_image_enabled = True
    robot.camera.enable_auto_exposure = True

    gain,exposure,mode = 390,3,1

    global old_state
    global new_state
    global right
    global left
    global activeSpeed
    global face_images
    global cubePose
    global angle
    global oldcube

cozmo.oled_face.convert_image_to_screen_data(resized_image,invert_image=True)

right = True
baseTurn = 29
activeSpeed = baseTurn
oldcube = None
    
        old_state = 0
        new_state = 0
        right = True
        left = False

#FSM States: (0) start, (1) searchcube, (2) gotocube, (3) searchcolor, (4)
followcolor
    class CozmoFSM:
        def __init__(self, robot: cozmo.robot.Robot):
            self.robot = robot
        async def start(self):

            global new_state
            await robot.say_text("Starting machine of doom",
    use_cozmo_voice=True,duration_scalar=2.0, voice_pitch=-10).wait_for_completed()
#face cozmo straight ahead
    await
    robot.set_head_angle(cozmo.util.radians(-.22),in_parallel=True).wait_for_completed()
#reset lift height
        if robot.lift_height.distance_mm > cozmo.robot.MIN_LIFT_HEIGHT_MM:
            await
    robot.set_lift_height(0.0,in_parallel=True).wait_for_completed()

        new_state = 1

async def searchcube(self):
    
    global new_state
    global cubePose
    global angle
    try:
        await robot.say_text("search and destroy",use_cozmo_voice=True,duration_scalar=2.0, voice_pitch=-10).wait_for_completed()
        print("waiting")
        event = await
robot.world.wait_for(cozmo.objects.EvtObjectObserved, timeout = 5)
        cubePose = event.pose
        angle = cozmo.util.degrees(cubePose.rotation.angle_z.degrees)
        new_state = 2
    except Exception as e:
        if right:
            await robot.drive_wheels(activeSpeed/2,-(activeSpeed/2))
        elif left:
            await robot.drive_wheels(-(activeSpeed/2),activeSpeed/2)



    async def gotocube(self):
        print("CubePose", cubePose)
        await robot.say_text("Going to detroy",
use_cozmo_voice=True,duration_scalar=2.0, voice_pitch=-10).wait_for_completed()

        global new_state
            #(rel to cozmo)/(rel to catesian coord)
        if angle.degrees > -135 and angle.degrees < -45:
            #right/down
            newPose = cozmo.util.pose_z_angle(cubePose.position.x,
cubePose.position.y + 110, cubePose.position.z, angle)
        elif angle.degrees > -45 and angle.degrees < 45:
            #away/right
            newPose = cozmo.util.pose_z_angle(cubePose.position.x - 110,
cubePose.position.y, cubePose.position.z, angle)
        elif angle.degrees > 45 and angle.degrees < 135:
            #left/up
            newPose = cozmo.util.pose_z_angle(cubePose.position.x,
cubePose.position.y - 110, cubePose.position.z, angle)
        else:
            #toward/left
            newPose = cozmo.util.pose_z_angle(cubePose.position.x + 110,
cubePose.position.y, cubePose.position.z, angle)
            print("newPose", newPose)

            await robot.go_to_pose(newPose).wait_for_completed()
            print("Cozmo", robot.pose)
            new_state = 3
async def searchcolor(self):
    await robot.say_text("looking for color",
    use_cozmo_voice=True,duration_scalar=2.0, voice_pitch=-10).wait_for_completed()

    event = await
robot.world.wait_for(cozmo.camera.EvtNewRawCameraImage, timeout=30) #get camera image

    if event.image is not None:
        image = cv2.cvtColor(np.asarray(event.image), cv2.COLOR_BGR2RGB)
        if mode == 1:
            robot.camera.enable_auto_exposure = True
        else:
            robot.camera.set_manual_exposure(exposure,fixed_gain)
    cube = find_cube(image, YELLOW_LOWER, YELLOW_UPPER)
    BoxAnnotator.cube = cube
    
    global activeSpeed
    global new_state
    global oldcube

    if right:
        await robot.drive_wheels(activeSpeed,-(activeSpeed))
    elif left:
        await robot.drive_wheels(-(activeSpeed),activeSpeed)
    if cube is not None:
        oldcube = cube
        new_state = 4
async def followcolor(self):
    
    await robot.say_text("Targeting color",
    use_cozmo_voice=True,duration_scalar=2.0, voice_pitch=-10).wait_for_completed()
    event = await
    
robot.world.wait_for(cozmo.camera.EvtNewRawCameraImage, timeout=30) #get camera image

    if event.image is not None:
        image = cv2.cvtColor(np.asarray(event.image), cv2.COLOR_BGR2RGB)

        if mode == 1:
            robot.camera.enable_auto_exposure = True
        else:
            robot.camera.set_manual_exposure(exposure,fixed_gain)
    cube = find_cube(image, GREEN_LOWER, GREEN_UPPER)
    BoxAnnotator.cube = cube

    global new_state
    global oldcube
    global left
    global right
    
    if oldcube is not None:
        if oldcube[0] <= 160:
            left = True
            right = False
        elif oldcube[0] > 160:
            right = True
            left = False
    if cube is None:
        new_state = 3
    else:
        oldcube = cube
        if (cube[0] >= 200):
            await robot.drive_wheels(activeSpeed/2,-(activeSpeed/2),l_wheel_acc=150,r_wheel_acc=-150)
        elif (cube[0] <= 120):
            await robot.drive_wheels(-(activeSpeed/2),(activeSpeed/2),l_wheel_acc=-150,r_wheel_acc=150)
        elif (cube[2] <= 115):
            await robot.drive_wheels((activeSpeed),(activeSpeed),l_wheel_acc=200,r_wheel_acc=200)
        else:
            robot.stop_all_motors()

    async def state_change(self):
        global old_state
        global new_state
        global face_images

        if old_state != new_state:
            print("old state: %d -> new state: %d" % (old_state, new_state))
            await robot.say_text("going from %d to %d" % (old_state,
            new_state), use_cozmo_voice=True,duration_scalar=2.0, voice_pitch= -10).wait_for_completed()

            if new_state == 0:
                await self.robot.say_text("restart", duration_scalar = 0.8,
                in_parallel=True).wait_for_completed()
            if new_state == 1:
                await self.robot.say_text("searchcube", duration_scalar =
                0.8, in_parallel=True).wait_for_completed()
            if new_state == 2:
                await self.robot.say_text("gotocube", duration_scalar = 0.8,
                in_parallel=True).wait_for_completed()
            if new_state == 3:
                await self.robot.say_text("searchcolor", duration_scalar =
                0.8, in_parallel=True).wait_for_completed()

            old_state = new_state
    if new_state == 0:
        await self.start()
    if new_state == 1:
        await self.searchcube()
    if new_state == 2:
        await self.gotocube()
    if new_state == 3:
        await self.searchcolor()
    if new_state == 4:
        await self.followcolor()
        
fsm = CozmoFSM(robot)
try:
    while True:
        await fsm.state_change()
except KeyboardInterrupt:
    print("")
    print("Exit requested by user")
except cozmo.RobotBusy as e:
    print(e)
    
if __name__ == '__main__':
    cozmo.run_program(run, use_viewer = True, force_viewer_on_top = True)








