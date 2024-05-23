from controller import Supervisor
import ikpy
from ikpy.chain import Chain
import numpy as np
import math
import sys
flag = 0
# Load your URDF file and specify the active links
robot_chain = Chain.from_urdf_file("Robot.urdf", active_links_mask=[False, True, True, True, True])
supervisor = Supervisor()
timestep = int(4*supervisor.getBasicTimeStep())
# Get motors and sensors
# Initialize the arm motors and encoders.
motors = []
for link in robot_chain.links:
    if 'motor' in link.name:
        motor = supervisor.getDevice(link.name)
        motor.setVelocity(2.0)
        position_sensor = motor.getPositionSensor()
        position_sensor.enable(timestep)
        motors.append(motor)
print(motors)
IKPY_MAX_ITERATIONS=4
target= supervisor.getFromDef('TARGET')
# Loop 1: Draw a circle on the paper sheet.
print('Draw a circle on the paper sheet...')
while supervisor.step(timestep) != -1:
    t = supervisor.getTime()

    # Use the circle equation relatively to the arm base as an input of the IK algorithm.
    x = 0.07 * math.cos(t) + 0.2
    y = 0.07 * math.sin(t) + 0.2
    z = 0.25
    print(x, y, z)
    # Call "ikpy" to compute the inverse kinematics of the arm.
    initial_position = [0] + [m.getPositionSensor().getValue() for m in motors]
    ikResults = robot_chain.inverse_kinematics([x, y, z], max_iter=IKPY_MAX_ITERATIONS, initial_position=initial_position)
    supervisor.getDevice('pen').write(True)
    # Actuate the 3 first arm motors with the IK results.
    for i in range(3):
        motors[i].setPosition(ikResults[i + 1])
    if supervisor.getTime() > 2 * math.pi :
        break
        