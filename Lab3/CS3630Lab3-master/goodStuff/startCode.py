import asyncio
import sys
import cv2
import numpy as np
import cozmo
import time
import os

from cozmo.util import degrees, distance_mm, Speed, radians

from generalSearch import generalSearch
from statemachine import StateMachine
from colorTracking import colorTracking
from searchRight import searchRight
from searchLeft import searchLeft

def run(robot: cozmo.robot.Robot):

    robot.camera.image_stream_enabled = True
    robot.camera.color_image_enabled = True
    robot.camera.enable_auto_exposure = True

    FSM = StateMachine()
    gSearch = generalSearch()
    cTrack = colorTracking()
    rightSearch = searchRight()
    leftSearch = searchLeft()
    FSM.setStartState(gSearch)
    FSM.addState(gSearch)
    FSM.addState(cTrack)
    FSM.addState(rightSearch)
    FSM.addState(leftSearch)
    FSM.run(robot)

if __name__ == '__main__':
    cozmo.run_program(run, use_viewer = True, force_viewer_on_top = True)