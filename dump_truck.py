
import numpy as np

from isaacsim.core.api import World
from isaacsim.core.utils.types import ArticulationAction

from isaacsim.robot.wheeled_robots.robots import WheeledRobot
from isaacsim.robot.wheeled_robots.controllers.differential_controller import DifferentialController


# ---------------------------------------
# 1. Connect to existing robot
# ---------------------------------------

world = World(stage_units_in_meters=1.0)

robot = world.scene.add(
    WheeledRobot(
        prim_path="/World/MyRobot",
        name="my_rover",
        wheel_dof_names=[
            "front_left_joint",
            "rear_left_joint",
            "front_right_joint",
            "rear_right_joint"
        ]
    )
)


# ---------------------------------------
# 2. Define DifferentialController
# ---------------------------------------

controller = DifferentialController(
    name="rover_controller",
    wheel_radius=0.10,
    wheel_base=0.40
)


# ---------------------------------------
# 3. Initialise robot articulation
# ---------------------------------------

world.reset()


# ---------------------------------------
# 4. Define robot movement function
# ---------------------------------------

def drive_robot(linear, angular):

    action = controller.forward(
        command=np.array([linear, angular])
    )

    left, right = action.joint_velocities

    robot.apply_wheel_actions(
        ArticulationAction(
            joint_velocities=np.array([
                left,
                left,
                right,
                right
            ])
        )
    )


# ---------------------------------------
# 5. Run robot
# ---------------------------------------

for step in range(840):

    if step < 240:
        # Move forward
        drive_robot(0.4, 0.0)

    elif step < 360:
        # Turn left
        drive_robot(0.0, 0.5)

    elif step < 600:
        # Move forward
        drive_robot(0.4, 0.0)

    else:
        # Stop
        drive_robot(0.0, 0.0)

    world.step(render=True)

print("Robot movement completed")
