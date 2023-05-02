import asyncio

import cozmo
from cozmo.util import degrees, Pose

from imageShow import showImage


class generalSearch:
    def getName(self):
        return "findCube"

    def run(self, robot: cozmo.robot.Robot):
        robot.set_head_angle(degrees(-15)).wait_for_completed()
        robot.move_lift(-5)

        look_around = robot.start_behavior(cozmo.behavior.BehaviorTypes.LookAroundInPlace)

        cube = None

        showImage(robot, "ARCUBE.png")

        try:
            cube = robot.world.wait_for_observed_light_cube(timeout=30)
        except asyncio.TimeoutError:
            print("Cube not found")
        finally:
            look_around.stop()

        if cube:
            deg = cube.pose.rotation.angle_z.degrees
            if deg < 25 and deg >= -20:
                pose = Pose(cube.pose.position.x - 80, cube.pose.position.y, cube.pose.position.z,
                            q0=cube.pose.rotation.q0, angle_z=cube.pose.rotation.angle_z, origin_id=cube.pose.origin_id)
            elif deg < 70 and deg >= 25:
                pose = Pose(cube.pose.position.x - 60, cube.pose.position.y - 60, cube.pose.position.z,
                            q0=cube.pose.rotation.q0, angle_z=cube.pose.rotation.angle_z, origin_id=cube.pose.origin_id)
            elif deg < 110 and deg >= 70:
                pose = Pose(cube.pose.position.x, cube.pose.position.y - 80, cube.pose.position.z,
                            q0=cube.pose.rotation.q0, angle_z=cube.pose.rotation.angle_z, origin_id=cube.pose.origin_id)
            elif deg < 160 and deg >= 110:
                pose = Pose(cube.pose.position.x + 80, cube.pose.position.y - 60, cube.pose.position.z,
                            q0=cube.pose.rotation.q0, angle_z=cube.pose.rotation.angle_z, origin_id=cube.pose.origin_id)
            elif deg < -160 or deg >= 160:
                pose = Pose(cube.pose.position.x + 80, cube.pose.position.y, cube.pose.position.z,
                            q0=cube.pose.rotation.q0, angle_z=cube.pose.rotation.angle_z, origin_id=cube.pose.origin_id)
            elif deg < -120 and deg >= -160:
                pose = Pose(cube.pose.position.x + 60, cube.pose.position.y + 60, cube.pose.position.z,
                            q0=cube.pose.rotation.q0, angle_z=cube.pose.rotation.angle_z, origin_id=cube.pose.origin_id)
            elif deg < -60 and deg >= -120:
                pose = Pose(cube.pose.position.x, cube.pose.position.y + 80, cube.pose.position.z,
                            q0=cube.pose.rotation.q0, angle_z=cube.pose.rotation.angle_z, origin_id=cube.pose.origin_id)
            else:
                pose = Pose(cube.pose.position.x - 40, cube.pose.position.y + 60, cube.pose.position.z,
                            q0=cube.pose.rotation.q0, angle_z=cube.pose.rotation.angle_z, origin_id=cube.pose.origin_id)

            print("test:", cube.pose.rotation.angle_z.degrees)
            action = robot.go_to_pose(pose)
            action.wait_for_completed()
            return "colorTrack", robot
