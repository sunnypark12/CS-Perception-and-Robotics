import setting
from particle import Particle
from utils import add_gaussian_noise, rotate_point, grid_distance
import numpy as np
np.random.seed(setting.RANDOM_SEED)
from itertools import product
import math


def create_random(count, grid):
    """
    Returns a list of <count> random Particles in free space.

    Parameters:
        count: int, the number of random particles to create
        grid: a Grid, passed in to motion_update/measurement_update

    Returns:
        List of Particles with random coordinates in the grid's free space.
    """
    # TODO: implement here
    # -------------------

    # -------------------
    particles = []
    for _ in range(count):
        while True:
            x, y = grid.random_free_place()
            if grid.is_free(x, y):
                particles.append(Particle(x, y, np.random.uniform(0, 360)))
                break
    return particles  

# ------------------------------------------------------------------------
def motion_update(old_particles, odometry_measurement, grid):
    """
    Implements the motion update step in a particle filter. Refer setting.py and utils.py for required functions and noise parameters.

    NOTE: the GUI will crash if you have not implemented this method yet. To get around this, try setting new_particles = old_particles.

    Arguments:
        old_particles: List 
            list of Particles representing the belief before motion update p(x_{t-1} | u_{t-1}) in *global coordinate frame*
        odometry_measurement: Tuple
            noisy estimate of how the robot has moved since last step, (dx, dy, dh) in *local robot coordinate frame*

    Returns: 
        a list of NEW particles representing belief after motion update \tilde{p}(x_{t} | u_{t})
    """
    new_particles = []

    for particle in old_particles:
        # extract the x/y/heading from the particle
        x_g, y_g, h_g = particle.xyh
        # and the change in x/y/heading from the odometry measurement
        dx_r, dy_r, dh_r = odometry_measurement

        # TODO: implement here
        # ----------------------------------
        # align odometry_measurement's robot frame coords with particle's global frame coords (heading already aligned)

        # compute estimated new coordinate, using current pose and odometry measurements. Make sure to add noise to simulate the uncertainty in the robot's movement.

        # create a new particle with this noisy coordinate

        # ----------------------------------
        # Convert local movement to global coordinates
        dx_g, dy_g = rotate_point(dx_r, dy_r, particle.h)

        # Adding Gaussian noise to simulate real-world uncertainty
        x_g += add_gaussian_noise(dx_g, setting.ODOM_TRANS_SIGMA)
        y_g += add_gaussian_noise(dy_g, setting.ODOM_TRANS_SIGMA)
        h_g = (h_g + add_gaussian_noise(dh_r, setting.ODOM_HEAD_SIGMA)) % 360

        new_particles.append(Particle(x_g, y_g, h_g))

    return new_particles

# ------------------------------------------------------------------------
def generate_marker_pairs(robot_marker_list, particle_marker_list):
    """ Pair markers in order of closest distance

        Arguments:
        robot_marker_list -- List of markers observed by the robot
        particle_marker_list -- List of markers observed by the particle

        Returns: List[Tuple] of paired robot and particle markers
    """
    marker_pairs = []
    while len(robot_marker_list) > 0 and len(particle_marker_list) > 0:
        # TODO: implement here
        # ----------------------------------
        # find the (particle marker,robot marker) pair with shortest grid distance
        
        # add this pair to marker_pairs and remove markers from corresponding lists
    
        # ----------------------------------
        robot_marker, particle_marker = min(
            product(robot_marker_list, particle_marker_list),
            key=lambda m: grid_distance(m[0][0], m[0][1], m[1][0], m[1][1])
        )
        marker_pairs.append((robot_marker, particle_marker))
        robot_marker_list.remove(robot_marker)
        particle_marker_list.remove(particle_marker)    
    return marker_pairs

def gaussian(x, mu, sigma):
    return (1.0 / (sigma * np.sqrt(2 * np.pi))) * np.exp(-0.5 * ((x - mu) / sigma) ** 2)

# ------------------------------------------------------------------------
def marker_likelihood(robot_marker, particle_marker):
    """ Calculate likelihood of reading this marker using Gaussian PDF. The 
        standard deviation of the marker translation and heading distributions 
        can be found in settings.py  

        Arguments:
        robot_marker -- Tuple (x,y,theta) of robot marker pose
        particle_marker -- Tuple (x,y,theta) of particle marker pose

        Returns: float probability
    """

    # TODO: implement here
    # ----------------------------------
    # find the distance between the particle marker and robot marker

    # find the difference in heading between the particle marker and robot marker

    # calculate the likelihood of this marker using the gaussian pdf

    # ----------------------------------
    dist = grid_distance(robot_marker[0], robot_marker[1], particle_marker[0], particle_marker[1])
    heading_diff = min(abs(robot_marker[2] - particle_marker[2]), 360 - abs(robot_marker[2] - particle_marker[2]))
    # Calculate the likelihood using Gaussian PDF for distance and heading difference
    dist_likelihood = gaussian(dist, 0, setting.MARKER_TRANS_SIGMA)
    heading_likelihood = gaussian(heading_diff, 0, setting.MARKER_HEAD_SIGMA)

    # Combined likelihood
    l = dist_likelihood * heading_likelihood
    return l

# ------------------------------------------------------------------------
def particle_likelihood(robot_marker_list, particle_marker_list):
    """ Calculate likelihood of the particle pose being the robot's pose

        Arguments:
        robot_marker_list -- List of markers observed by the robot
        particle_marker_list -- List of markers observed by the particle

        Returns: float probability
    """
    l = 1.0
    marker_pairs = generate_marker_pairs(robot_marker_list, particle_marker_list)
    # TODO: implement here
    # ----------------------------------
    # update the particle likelihood using the likelihood of each marker pair
    # HINT: consider what the likelihood should be if there are no pairs generated

    # ----------------------------------

    for robot_marker, particle_marker in marker_pairs:
        l *= marker_likelihood(robot_marker, particle_marker)
    if not marker_pairs:  # No pairs found
        l = 0.0
    return l

# ------------------------------------------------------------------------
def measurement_update(particles, measured_marker_list, grid):
    """ Particle filter measurement update

        Arguments:
        particles -- input list of particle represents belief \tilde{p}(x_{t} | u_{t})
                before measurement update (but after motion update)

        measured_marker_list -- robot detected marker list, each marker has format:
                measured_marker_list[i] = (rx, ry, rh)
                rx -- marker's relative X coordinate in robot's frame
                ry -- marker's relative Y coordinate in robot's frame
                rh -- marker's relative heading in robot's frame, in degree

                * Note that the robot can only see markers which is in its camera field of view,
                which is defined by ROBOT_CAMERA_FOV_DEG in setting.py
				* Note that the robot can see mutliple markers at once, and may not see any one

        grid -- grid world map, which contains the marker information,
                see grid.py and CozGrid for definition
                Can be used to evaluate particles

        Returns: the list of particles represents belief p(x_{t} | u_{t})
                after measurement update
    """
    measured_particles = []
    particle_weights = []
    num_rand_particles = 25
    
    if len(measured_marker_list) > 0:
        for p in particles:
            x, y = p.xy
            if grid.is_in(x, y) and grid.is_free(x, y):
                robot_marker_list = measured_marker_list.copy()
                particle_marker_list =  p.read_markers(grid)

                # TODO: implement here
                # ----------------------------------
                # compute the likelihood of the particle pose being the robot's
                # pose when the particle is in a free space

                # ----------------------------------
                l = 1.0
                marker_pairs = generate_marker_pairs(robot_marker_list, particle_marker_list)
                for robot_marker, particle_marker in marker_pairs:
                    l *= marker_likelihood(robot_marker, particle_marker)
                if not marker_pairs:  # No pairs found
                    l = 0.0
            else:
                # TODO: implement here
                # ----------------------------------
                # compute the likelihood of the particle pose being the robot's pose
                # when the particle is NOT in a free space

                # ----------------------------------
                l = 0.0
            particle_weights.append(l)
    else:
        particle_weights = [1.]*len(particles)

    # TODO: Importance Resampling
    # ----------------------------------
    # if the particle weights are all 0, generate a new list of random particles

    # normalize the particle weights
            
    # create a fixed number of random particles and add to measured particles

    # resample remaining particles using the computed particle weights

    # ----------------------------------
    if all(weight == 0 for weight in particle_weights):
        return create_random(len(particles), grid)

    total_weight = sum(particle_weights)
    normalized_weights = [w / total_weight for w in particle_weights]
    resampled_particles = np.random.choice(particles, len(particles) - num_rand_particles, p=normalized_weights, replace=True)
    random_particles = create_random(num_rand_particles, grid)
    measured_particles = list(resampled_particles) + random_particles
    return measured_particles