import numpy as np
from numpy.linalg import norm
from parso import parse
from scipy.spatial.distance import pdist, squareform
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from IPython.display import HTML
import random
import argparse

matplotlib.rcParams["animation.writer"] = "avconv"
plt.rcParams["animation.html"] = "html5"

"""robot specific simulation parameters"""
num_robots = 50  # number of robots in the simulation
bot_size = 0.25  # size of each robot
sense_range = 50  # range of robots sensor
avoidence_range = 5  # distance at which other robots will be avoided
max_vel = 100  # robots will move at 'max_vel' m/s at their best
close_enough = 5  # acceptable range for arriving at target
dt = 0.01  # simulation timestep set to 10ms

"""map specific simulation parameters"""
line1 = np.array([[0, 200], [200, 200]])  # co-ordinate of the first line in the arena
line2 = np.array([[200, 200], [200, 0]])  # co-ordinates of the second line in the arena
target1 = line1[1]  # first target that the robots must rach
target2 = line2[1]  # second target taht the robots must reach
lines = np.array([line1, line2])
targets = np.array([target1, target2])

"""generating robots, velocities and distance matrix"""
x_pos = random.sample(range(-50, 51), num_robots)
y_pos = random.sample(range(150, 251), num_robots)
robots_pos = np.array(
    [[i, j] for i, j in zip(x_pos, y_pos)]
)  # 'robots_pos' contains the locations of all robots
robots_vel = np.zeros(
    (num_robots, 2)
)  # 'robots_vel' contains the velocity of each robots


fig = plt.figure(figsize=(6, 6))
ax = fig.add_subplot(111, aspect="equal", xlim=(-70, 270), ylim=(-70, 270))
(line1_plot,) = ax.plot(
    [line1[0, 0], line1[1, 0]], [line1[0, 1], line1[1, 1]], color="black"
)
(line2_plot,) = ax.plot(
    [line2[0, 0], line2[1, 0]], [line2[0, 1], line2[1, 1]], color="black"
)
(robots_plot,) = ax.plot(robots_pos[:, 0], robots_pos[:, 1], "ro", ms=2)

# intitialize the array of robot velocities to random numbers between [0, 1)
# scale them by the maximum allowed velocity
robots_vel = np.random.rand(num_robots, 2) * max_vel


def init():
    line1_plot.set_data([line1[0, 0], line1[1, 0]], [line1[0, 1], line1[1, 1]])
    line2_plot.set_data([line2[0, 0], line2[1, 0]], [line2[0, 1], line2[1, 1]])
    return (line1_plot, line2_plot)


def get_centroid():
    """Returns centroid of the entire system of robots."""
    centroid = (
        np.sum(robots_pos, axis=0) / num_robots
    )  # The centroid is simply the average of the x and y coordinates of all points
    return centroid


def get_average_distance_from_centroid():
    """Returns the average distance of all robots from the centroid."""
    centroid = get_centroid()
    distance = np.mean([norm(centroid - robots_pos[i]) for i in range(num_robots)])
    return distance


def get_distance_matrix():
    """Returns a squared distance matrix of all robots in the system"""
    distances = pdist(robots_pos)
    distance_matrix = squareform(distances)
    return distance_matrix


def get_unit_vector(vector):
    """Given a vector as an np array, this function returns the unit vector in that direction"""
    return vector / norm(vector)


def get_direction(point1, point2):
    """Given 2 points as np arrays, this function returns the unit vector of the distnce from point 1 to point 2"""
    return get_unit_vector(point2 - point1)


def reached_target(target, centroid):
    """Given a taget and the centroid of the system of robots, this function returns true if centroid is close enught to the target"""
    if target[1] == target1[1]:
        if (target[0] - centroid[0]) < close_enough:
            return True
    else:
        if (target[1] - centroid[1]) < close_enough:
            return True

    return False


def homing_towards_line(i, line):
    """Function to make a robot home towards a line
    Takes as argument the index of the robot to move and the line towards which it should move
    Returns the unit velocity vector to be emparted to the robot in order to home towards the line"""
    bot_position = robots_pos[i]
    velocity = np.zeros((2))

    if line is line1:
        if bot_position[1] > line[0, 1]:
            velocity[1] = -1
        else:
            velocity = 1

    if line is line2:
        if bot_position[0] > line[0, 0]:
            velocity[0] = -1
        else:
            velocity = 1

    return velocity


def homing_towards_point(i, point):
    """Function to make a robot home towards a point
    Takes as argument the index of the robot to move and the point towards which it should move
    Returns the velocity to be imparted to the robot in order to home towards the point"""
    bot_position = robots_pos[i]
    unit_vector = get_direction(bot_position, point)
    velocity = unit_vector

    return velocity


def aggregate(i):
    """This function takes as argument the index of a robot
    It checks the robot's sensor readings to see other robots in range
    It then instructs the robot to move towards other robots
    Returns the velocity to be imparted to the robot to for aggregation"""
    bot_position = robots_pos[i]  # retireve position of current robot
    velocity = np.zeros((2))  # This is the vlocity that will be returned
    bot_distances = np.array(
        distance_matrix[i]
    )  # retrieve distances of all robots to this robot
    (close_robots,) = np.where(
        bot_distances < sense_range
    )  # get list of indicies for the robots that are in range
    (index,) = np.where(
        close_robots == i
    )  # find the index of this robot in the above list
    close_robots = np.delete(close_robots, index)  # delete that entry from the list

    # close_robots now contains the indices of all the robots in range of robot i

    # for every robot in sensor range calculate the unit vector to that robot
    # add it to this robot's velocity
    # then find the unit vector of that velocity
    # finally multiply that unit vector for the velocity with the max veocity

    if close_robots.size != 0:
        for bot in close_robots:
            bot_position_2 = robots_pos[bot]
            velocity = velocity + get_direction(bot_position, bot_position_2)
        velocity = get_unit_vector(velocity)

    return velocity


def avoid_peers(i):  # Same as Dispersion
    """This funtion takes as input the index of a robot
    It checks the robot's sensor readings to see other robots that are too close for comfort
    It then instructs the robot to move away from those robots that are within avoidence range
    Returns the velocity to be imparted to the robot to for avoidence"""
    bot_position = robots_pos[i]  # retireve position of current robot
    velocity = np.zeros((2))  # This is the vlocity that will be returned
    bot_distances = distance_matrix[i]  # retrieve distances of all robots to this robot
    (close_robots,) = np.where(
        bot_distances < avoidence_range
    )  # get list of indicies for the robots that are in range
    (index,) = np.where(
        close_robots == i
    )  # find the index of this robot in the above list
    close_robots = np.delete(close_robots, index)  # delete that entry from the list

    # for every robot in avoidence range calculate the unit vector to that robot
    # add it to this robot's velocity (don't worry, we wil negate the whole thing at the end)
    # then find the unit vector of tat velocity vector
    # finally multiply that unit vector of the velocity with negateive one

    if close_robots.size != 0:
        for bot in close_robots:
            bot_position_2 = robots_pos[bot]
            velocity = velocity + get_direction(bot_position, bot_position_2)
        velocity = -1 * get_unit_vector(velocity)

    return velocity


"""weight for motion primitives"""
w_home_point = 1
w_home_line = 0.25
w_aggregate = 2
w_avoid = 4

"""initial path"""
line = line1
point = target1

"""Region Simulation function"""
# This function gets called every step of the animation
def simulate(t):
    global distance_matrix
    global robots_vel
    global robots_pos
    global centroid
    global avg_dist
    global timestep
    global centroid_line_distance
    global line
    global point

    # calculate distance matrix
    distance_matrix = get_distance_matrix()

    # calculate centroid
    centroid = get_centroid()

    # switch tagets when the first target has basically been reached
    if reached_target(point, centroid):
        line = line2
        point = target2

    robots_vel = np.array(
        [
            max_vel
            * get_unit_vector(
                w_home_line * homing_towards_line(i, line)
                + w_home_point * homing_towards_point(i, point)
                + w_aggregate * aggregate(i)
                + w_avoid * avoid_peers(i)
            )
            for i in range(num_robots)
        ]
    )
    robots_pos = robots_pos + dt * robots_vel

    return robots_pos


def update(t):
    positions = simulate(t)
    robots_plot.set_data(positions[:, 0], positions[:, 1])
    return (robots_plot,)


if __name__ == "__main__":
    """Run simulation depending on parsed arguments."""
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--num-robots",
        type=int,
        nargs=1,
        help="Number of robots used in flocking simulation.",
    )
    parser.add_argument(
        "--num-adversarial",
        type=int,
        nargs=1,
        help="Number of adversarial robots used in flocking simulation.",
    )
    parser.add_argument(
        "--iters", type=int, nargs=1, help="Number of iterations of simulation."
    )
    parser.add_argument(
        "--noise",
        type=bool,
        nargs=1,
        help="Amount of Gaussian noise involved in communications between robots.",
    )
    parser.add_argument(
        "--max-velocity", type=int, nargs=1, help="Max velocity of robots."
    )
    parser.add_argument(
        "--sensor-range", type=int, nargs=1, help="Sensor range of each robot."
    )

    args = parser.parse_args()
    print(f"iters: {args.iters}")

    animate = animation.FuncAnimation(
        fig,
        update,
        # init_func=init,
        frames=5000,
        interval=4,
        #  blit=True
    )

    plt.show()
