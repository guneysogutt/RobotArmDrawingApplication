from controller import Supervisor
import ikpy
from ikpy.chain import Chain
import math

# Load URDF file and define active links as true or false
robot_chain = Chain.from_urdf_file("Robot.urdf", active_links_mask=[False, True, True, True, True])

# Define supervisor function for further operations
supervisor = Supervisor()
timestep = int(4*supervisor.getBasicTimeStep())

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

# Initially define pen device write fucntion as false 
supervisor.getDevice('pen').write(False)

# Main while loop to draw the heart shape
while supervisor.step(timestep) != -1:
    t = supervisor.getTime()
    
    # Define the heart shape parametric equations relatively to the arm base as an input of the IK algorithm
    scale = 0.008  # Scale of drawing
    x = scale * (16 * math.sin(t)**3) + 0.2
    y = scale * (13 * math.cos(t) - 5 * math.cos(2 * t) - 2 * math.cos(3 * t) - math.cos(4 * t)) + 0.2
    z = 0.25  # Fixed height
    print(x, y, z)
    
    # Get initial position from all sensors
    initial_position = [0] + [m.getPositionSensor().getValue() for m in motors]
    
    # Calculate IK results for robot chain
    ikResults = robot_chain.inverse_kinematics([x, y, z], max_iter=IKPY_MAX_ITERATIONS, initial_position=initial_position)
    
    # Actuate the 3 first arm motors with the IKPY results
    for i in range(3):
        motors[i].setPosition(ikResults[i + 1])
    
    # Break the loop after completing one cycle define write as false
    if supervisor.getTime() < 2 * math.pi:
        if supervisor.getTime() > 0.5:
            supervisor.getDevice('pen').write(True)
    else:
        supervisor.getDevice('pen').write(False)
        break
        