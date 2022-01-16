# %%

# import modules
from sqlite3 import Row
import osmnx as ox
import geopandas as gpd
import pandas as pd
from os import path
import numpy as np
from shapely.geometry import LineString, Point
from spawn_despawn_points import get_spawn_despawn_gdfs, get_n_origin_destination_pairs
from scipy.spatial.transform import Rotation as R
from matplotlib import pyplot as plt

def main():

    # import airspace polygon with geopandas
    airspace_path = path.join(path.dirname(__file__), 'gis/airspace/total_polygon.gpkg')
    airspace = gpd.read_file(airspace_path, driver='GPKG', layer='total_polygon')

    # import airspace polygon with geopandas
    airspace_path = path.join(path.dirname(__file__), 'gis/airspace/updated_constrained_airspace.gpkg')
    con_airspace = gpd.read_file(airspace_path, driver='GPKG')

    # get origin and destination points for rogue aircraft
    spawn_gdf, despawn_gdf = get_spawn_despawn_gdfs(airspace, 64, 100, 12000)

    # get n origin and destination pairs
    origin_destination_pairs = get_n_origin_destination_pairs(spawn_gdf, despawn_gdf, 9)
    
    figure, axes = plt.subplots(nrows=3, ncols=3)

    col_counter = 0
    row_counter = 0
    # choose one random path
    for idx, origin_destination_pair in enumerate(origin_destination_pairs):

        random_path = gen_random_path(origin_destination_pair)

        if idx == 3:
            col_counter = 1
            row_counter = 0
        elif idx == 6:
            col_counter = 2
            row_counter = 0
        elif idx == 9:
            col_counter = 3
            row_counter = 0

        # plot the shapely linestring and airspace with matplolib
        axes[row_counter, col_counter].plot(random_path.xy[0], random_path.xy[1])
        axes[row_counter, col_counter].plot(airspace.geometry.boundary.values[0].xy[0], airspace.geometry.boundary.values[0].xy[1])
        axes[row_counter, col_counter].plot(con_airspace.geometry.boundary.values[0].xy[0], con_airspace.geometry.boundary.values[0].xy[1])

        row_counter += 1

    figure.tight_layout()

    plt.show()

def gen_random_path(origin_destination_pair, segment_length=1000, max_deviation=3000, simplify_tolerance=400):
    """
    Generate a random path between two Shapely points. The path is generated by
    randomly moving between the two points. The path is then simplified using
    shapely's simplify function. This path is guranteed not to intersect with itself.

    TODO: generate a random path that does not intersect with the airspace borders.
    TODO: generate a path that can intersect with itself.

    Parameters
    ----------
    origin_destination_pair : tuple
        A tuple of two Shapely points. The first element is the origin point,
        the second element is the destination point.
    destination : Shapely Point
        The destination point.
    segment_length : int
        The approximate length of each segment. This is a straight line distance 
        between the peaks of the path. It is parallel to the vector between the 
        origin and destination points.
    max_deviation : int
        The maximum deviation from the straight line between origin and destination.
    simplify_tolerance : int
        The tolerance for simplification with Douglas-Peucker algorithm. See shapely's 
        documentation for more information.

    Returns
    -------
    Shapely LineString
        The random path.

    """

    # get origin and destination points from pair
    origin, destination = origin_destination_pair[0], origin_destination_pair[1]

    # define a vector from 0,0 to the origin
    start_vector = np.array([origin.x, origin.y])

    # create a straight line between origin and destination points
    straight_path = LineString([origin, destination])

    # Generate a signal with the points spaced approximately by the segment length
    n_splits = int(straight_path.length / segment_length)
    x_signal = np.linspace(0, straight_path.length, n_splits + 1)

    # generate a random deviation from the straight line (first and last value should be zero)
    y_noise = np.random.uniform(low=-max_deviation, high=max_deviation, size=len(x_signal))
    y_noise[0], y_noise[-1] = 0, 0

    # combine into 3d signal so that we can rotate it with scipy [(n_splits + 1) x 3]
    xyz_noise = np.vstack((x_signal, y_noise, np.zeros(len(x_signal)))).T

    # calculate angle between origin and destination points
    angle = np.arctan2(destination.y - origin.y, destination.x - origin.x)

    # find the quadrant and modify angle so that it is angle to the horizontal
    if angle > np.pi/2:
        # quadrant 2 (reflect around y-axis)
        angle = np.pi - angle
        refl_vector = np.array([-1, 1])

    elif angle < -np.pi/2:
        # quadrant 3 (reflect around x-axis, y-axis)
        angle = np.pi + angle
        refl_vector = np.array([-1, -1])

    elif angle < 0:
        # quadrant 4 (reflect around x-axis)
        angle = -angle
        refl_vector = np.array([1, -1])

    else:
        # quadrant 1 (no reflection)
        refl_vector = np.array([1, 1])

    # create a rotation matrix for a 3d signal
    rot_mat = R.from_euler('xyz', [0, 0, angle], degrees=False)

    # rotate the signal with rot_mat (with z-component)
    xy_rot = rot_mat.apply(xyz_noise)

    # reflect the signal with refl_vector (remove z-component)
    xy_ref = xy_rot[:,:2] * refl_vector

    # translate the signal with start vector
    xy_trans = xy_ref + start_vector

    # create a gdf from new_df
    new_df = pd.DataFrame(xy_trans, columns=['x', 'y'])
    new_gdf = gpd.GeoDataFrame(new_df, geometry= new_df.apply(
        lambda row: Point([row['x'], row['y']]), axis=1))

    # Simplify the path
    random_path = LineString(new_gdf.geometry).simplify(simplify_tolerance)

    return random_path

if __name__ == '__main__':
    main()