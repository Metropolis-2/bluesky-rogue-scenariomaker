"""
This code decides the defaults for the scenario generation.

Code written by: Andres Morfin Veytia
Project: Metropolis 2
"""
import numpy as np

"""general information"""

# The number of paths to generate. This is also the number of rogue aircraft.

n_paths = 4

"""generatring the spawn and despawn points"""

# distance between border of airspace and spawn/despawn points of rogue aircraft

buffer_distance = 64  # meters

# min distance between spawn/despawn points of rogue aircraft

spawn_distance = 100  # meters

"""Path information"""

# minimum straight line disance between an origin and destination point.

min_path_distance = 14000  # meters

# The approximate length of the segments in the random path.
# Note that if using cosntrained airspace, this only applies to open airspace.
# This is the distance seprating the points in the path parallel to the straight line
# from origin to destination.

segment_length = 1000  # meters

# The maximum deviation from the straight line between origin and destination.
# This can be seen as the amplitude of the random path

max_deviation = 3000  # meters

# The simplification tolerance for the random path. See shapely.ops.simplify.

simplify_tolerance = 400  # meters

"""Constrained airspace information"""

# Whether or not to respect the constrained airspace in the paths
respect_constrained_airspace = True

"""Cruise altitude information"""

# The cruise altitudes that rogue aircraft may fly at.
cruise_alts = np.arange(30, 510, 30)  # feet
