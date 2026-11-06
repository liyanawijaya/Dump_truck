
import asyncio
import numpy as np
import omni.usd

from omni.isaac.core import World
from omni.isaac.core.utils.types import ArticulationAction

from omni.isaac.wheeled_robots.robots import WheeledRobot
from omni.isaac.wheeled_robots.controllers.differential_controller import DifferentialController


# ---------------------------------------
# 1. Robot configuration
# ---------------------------------------

ROBOT_PATH = "/World/MyRobot"

WHEEL_JOINTS = [
    "front_left_joint",
    "rear_left_joint",
    "front_right_joint",
    "rear_right_joint"
]

WHEEL_RADIUS = 0.10
WHEEL_BASE = 0.40


# ---------------------------------------
# 2. Initialise robot and simulation
# ---------------------------------------

async def setup_robot():

    stage = omni.usd.get_context().get_stage()

    # Check that the robot exists
    robot_prim = stage.GetPrimAtPath(ROBOT_PATH)

    if not robot_prim.IsValid():
        print("ERROR: Robot not found:", ROBOT_PATH)
        return

    # Clear previous World Python instance
    if World.instance() is not None:
        World.instance().clear_instance()

    world = World(stage_units_in_meters=1.0)

    await world.initialize_simulation_context_async()

    # Connect to robot already in USD stage
    robot = world.scene.add(
        WheeledRobot(
            prim_path=ROBOT_PATH,
            name="my_rover",
            wheel_dof_names=WHEEL_JOINTS
        )
    )

    # Create differential controller
    controller = DifferentialController(
        name="rover_controller",
        wheel_radius=WHEEL_RADIUS,
        wheel_base=WHEEL_BASE
    )

    # Initialise physics and articulation
    await world.reset_async()

    print("Robot initialised successfully!")

    # ---------------------------------------
    # 3. Drive function
    # ---------------------------------------

    def drive_robot(linear, angular):

        action = controller.forward(
            command=np.array([linear, angular])
        )

        left, right = action.joint_velocities

        wheel_action = ArticulationAction(
            joint_velocities=np.array([
                left,
                left,
                right,
                right
            ])
        )

        robot.apply_wheel_actions(wheel_action)

    # ---------------------------------------
    # 4. Physics callback
    # ---------------------------------------

    elapsed = 0.0
    finished = False

    def robot_control_step(dt):
        nonlocal elapsed, finished

        elapsed += dt

        if elapsed < 4.0:
            drive_robot(0.4, 0.0)

        elif elapsed < 6.0:
            drive_robot(0.0, 0.5)

        elif elapsed < 10.0:
            drive_robot(0.4, 0.0)

        else:
            drive_robot(0.0, 0.0)

            if not finished:
                print("Robot movement completed!")
                finished = True

    world.add_physics_callback(
        "rover_control",
        callback_fn=robot_control_step
    )

    # Start simulation
    await world.play_async()


# ---------------------------------------
# 5. Execute from Script Editor
# ---------------------------------------

asyncio.ensure_future(setup_robot())
