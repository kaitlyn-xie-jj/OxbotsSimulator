from controller import Robot, Motor
import time

def other_go_random():
    TIME_STEP = 64
    MAX_SPEED = 6.28

    robot = Robot()

    # Get motors
    left_motor = robot.getDevice('left wheel motor')
    right_motor = robot.getDevice('right wheel motor')

    # Velocity control mode
    left_motor.setPosition(float('inf'))
    right_motor.setPosition(float('inf'))

    # Initial velocities (same as in your MATLAB)
    left_motor.setVelocity(0.9 * MAX_SPEED)
    right_motor.setVelocity(0.6 * MAX_SPEED)

    while robot.step(TIME_STEP) != -1:
        # Go straight
        left_motor.setVelocity(0.9 * MAX_SPEED)
        right_motor.setVelocity(0.9 * MAX_SPEED)
        time.sleep(1)

        # Rotate in place
        left_motor.setVelocity(-0.9 * MAX_SPEED)
        right_motor.setVelocity(0.9 * MAX_SPEED)
        time.sleep(1)

if __name__ == "__main__":
    other_go_random()