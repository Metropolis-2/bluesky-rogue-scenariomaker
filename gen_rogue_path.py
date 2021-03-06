"""
Code contains functions that are used to generate paths through the airspace.

There are two important functions in this file:
    1. gen_random_path()
    2. gen_path_through_constrained()

The important imported modules from this project are:
    1. find_border_nodes used for gen_path_through_constrained
    2. rogue_paths_constrained used to get the path only through constrained airspace

The other functions are helper and test functions for the two important functions.

Code written by: Andres Morfin Veytia
Project: Metropolis 2
"""

import osmnx as ox
import geopandas as gpd
import pandas as pd
import numpy as np
from shapely.geometry import LineString, Point, MultiPoint
from shapely.ops import linemerge
from scipy.spatial.transform import Rotation as R

# project modules
from find_border_nodes import find_border_nodes
from rogue_paths_constrained import get_lat_lon_from_osm_route, get_turn_arrays


def gen_random_path(
    origin_destination_pair,
    airspace_polygon,
    segment_length=1000,
    max_deviation=3000,
    simplify_tolerance=400,
):
    """
    Generate a random path between two Shapely points. The path is generated by
    randomly moving between the two points. The path is then simplified using
    shapely's simplify function. This path is guranteed not to intersect with itself.
    It also ensures that the path does not go outside the airspace polygon.

    Lastly, it returns a boolean array that indicates where the aicraft
    should performe altitude changes. At the moment it is hardcoded
    to perform altitude changes 3 times in the path.

    Parameters
    ----------
    origin_destination_pair : tuple
        A tuple of two Shapely points. The first element is the origin point,
        the second element is the destination point.
    airspace_polygon : shapely.geometry.Polygon
        The airspace polygon. This is used to ensure that the generated path
        does not go out of the airspace. Only the first and last point should
        be outside the airspace.
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
    random_path : Shapely LineString
        The random path.

    cruise_alt_changes : numpy.ndarray
        A boolean array that indicates whether a cruise change is needed at a given point.
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
    y_noise = np.random.uniform(
        low=-max_deviation, high=max_deviation, size=len(x_signal)
    )
    y_noise[0], y_noise[-1] = 0, 0

    # combine into 3d signal so that we can rotate it with scipy [(n_splits + 1) x 3]
    xyz_noise = np.vstack((x_signal, y_noise, np.zeros(len(x_signal)))).T

    # calculate angle between origin and destination points
    angle = np.arctan2(destination.y - origin.y, destination.x - origin.x)

    # find the quadrant and modify angle so that it is angle to the horizontal
    if angle > np.pi / 2:
        # quadrant 2 (reflect around y-axis)
        angle = np.pi - angle
        refl_vector = np.array([-1, 1])

    elif angle < -np.pi / 2:
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
    rot_mat = R.from_euler("xyz", [0, 0, angle], degrees=False)

    # rotate the signal with rot_mat (with z-component)
    xy_rot = rot_mat.apply(xyz_noise)

    # reflect the signal with refl_vector (remove z-component)
    xy_ref = xy_rot[:, :2] * refl_vector

    # translate the signal with start vector
    xy_trans = xy_ref + start_vector

    # check if any points are outside airspace
    contains_point = np.array([airspace_polygon.contains(Point(xy)) for xy in xy_trans])

    # set first and last to True since they are outside but we don't care
    contains_point[0] = contains_point[-1] = True

    # select only points inside constrained airspace
    xy_contains = xy_trans[contains_point]

    # create a gdf from new_df
    new_df = pd.DataFrame(xy_contains, columns=["x", "y"])
    new_gdf = gpd.GeoDataFrame(
        new_df, geometry=new_df.apply(lambda row: Point([row["x"], row["y"]]), axis=1)
    )

    # Simplify the path
    random_path = LineString(new_gdf.geometry).simplify(simplify_tolerance)

    # get numper of waypoints
    n_waypoints = len(random_path.coords)

    # get three points evenly distributed between first and last point
    cruise_change_waypoints = np.round(np.linspace(0, n_waypoints - 1, 5)).astype(int)[
        1:-1
    ]

    # get a bool of the cruise altitude changes
    cruise_alt_changes = np.zeros(n_waypoints, dtype=bool)
    cruise_alt_changes[cruise_change_waypoints] = True

    return random_path, cruise_alt_changes


def gen_path_through_constrained(random_path, con_airspace, G):
    """
    Generate a path that does not violate the constrained airspace.

    First, it checks if there are any intersections with constrained
    airspace. If there are none, then it returns the original path.

    If there are intersections, then the function finds the entry
    and exit points and divides the path into a front path and a
    back path.

    The front path is from the origin to the entry point
    of constrained airspace. The back path is from the exit point
    of constrained airspace to the destination.

    The entry and exit points are nodes of the constrained airspace.

    With these nodes, osmnx finds the shortest path through the constrained
    airspace.

    The last thing is to merge the front path, path through constrained airspace,
    and back path into one path.

    Also returns a turn bool that indicates whether there is a turn.
    Note that it is always false for open airspace. Also returns a bool
    that tells if you are in constrained airspace.

    Also creates a cruising altitude change boolean. It sets it to true
    in places where there should be an altitude change. Right now
    it just finds the mid waypoint of all three paths and sets those
    as the locations where the altitude changes.

    Parameters
    ----------
    random_path : shapely.geometry.LineString
        The random path that does not respect the constrained airspace.
    con_airspace : geopandas.GeoDataFrame
        The constrained airspace.
    G : networkx.MultiGraph
        The graph of the constrained airspace
    opt : str
        The option to return the path. can be 'merged' or 'split'.

    Returns
    -------
    merged_path : shapely.geometry.LineString
        The full path that does not violate the constrained airspace.

    turn_bool : numpy.ndarray
        The turn bool.

    in_constrained : numpy.ndarray
        The bool that tells if you are in constrained airspace.

    cruising_altitude_change : numpy.ndarray
        The bool that tells if there is an altitude change.
    """
    # get the airspace polygon
    con_airspace_polygon = con_airspace.geometry.values[0]

    # check if the path intersects the airspace. If not, return the path
    if not con_airspace_polygon.intersects(random_path):
        return random_path

    # get the border nodes and rtree of constrained airspace
    border_node_gdf, node_rtree = find_border_nodes(con_airspace, G)

    # split random path into individual segments to find intersections
    segments = list(
        map(LineString, zip(random_path.coords[:-1], random_path.coords[1:]))
    )

    intersecting_idx = []
    # check which segments intersect with the airspace
    for idx, segment in enumerate(segments):

        # check if the individual segment intesects with the airspace
        if segment.intersects(con_airspace_polygon):
            intersecting_idx.append(idx)

    # split the path into a front and back segments that connect to a constrained airspace polygon
    front_path, first_node = split_path(
        segments,
        intersecting_idx[0],
        border_node_gdf,
        node_rtree,
        con_airspace_polygon,
        "front",
    )
    back_path, last_node = split_path(
        segments,
        intersecting_idx[-1],
        border_node_gdf,
        node_rtree,
        con_airspace_polygon,
        "back",
    )

    # round geometry to avoid floating point errors
    front_path = round_geometry(front_path)
    back_path = round_geometry(back_path)

    # find a path from the front node to the back node in constrained airspace
    const_route = ox.shortest_path(G, first_node, last_node)

    # get lat lon from osm route
    lats_c, lons_c, line_gdf = get_lat_lon_from_osm_route(G, const_route)

    # convert to epsg 32633
    line_gdf = line_gdf.to_crs(epsg=32633)
    const_path = line_gdf.geometry.values[0]
    const_path = round_geometry(const_path)

    # merge the front and back paths
    merged_path = linemerge([front_path, const_path, back_path])

    # get turn bool for constrained airspace, front and back paths
    turn_bool_const, _, _ = get_turn_arrays(lats_c, lons_c, 25)
    turn_bool_front = np.array([False] * (len(front_path.coords) - 1))
    turn_bool_back = np.array([False] * (len(back_path.coords) - 1))

    # combine arrays
    turn_bool = np.concatenate([turn_bool_front, turn_bool_const, turn_bool_back])

    # create another bool that tells if in open or constrained airspace
    const_bool_const = np.array([True] * (len(const_path.coords) - 1))
    const_bool_front = np.array([False] * len(front_path.coords))
    const_bool_back = np.array([False] * (len(back_path.coords) - 1))

    # combine arrays
    in_constrained = np.concatenate(
        [const_bool_front, const_bool_const, const_bool_back]
    )

    # modify cruise altitude in start constrained, and end path.
    cruise_alt_changes = np.array([False] * len(in_constrained))

    # find end and start of constrained path
    start_idx = np.where(in_constrained)[0][0]
    end_idx = np.where(in_constrained)[0][-1]

    # height variations of start and end path (around midpoint waypoint)
    cruise_alt_changes[int(np.floor(start_idx / 2))] = True
    cruise_alt_changes[
        int(np.ceil(end_idx + (len(in_constrained) - end_idx) / 2))
    ] = True

    # height variation of path through constrained airspace. around middle waypoint
    constrained_idx = int(np.floor((end_idx - start_idx) / 2))
    cruise_alt_changes[constrained_idx] = True

    return merged_path, turn_bool, in_constrained, cruise_alt_changes


"""HELPER FUNCTIONS BELOW"""


def split_path(
    segments,
    intersecting_idx,
    border_node_gdf,
    node_rtree,
    con_airspace_polygon,
    loc="front",
):
    """
    Split the path into a front and back segments that connect to a constrained airspace polygon.
    If there are more than two intersections in a given segment, then the function
    finds the closest point to the point to connect to decide which point to use.

    Parameters
    ----------
    segments : list of shapely.geometry.LineString
        The individual segments of the path.
    intersecting_idx : int
        The index of the segment that intersects with the constrained airspace.
    border_node_gdf : geopandas.GeoDataFrame
        The border nodes of the constrained airspace.
    node_rtree : rtree.index.Index
        The rtree of the border nodes of the constrained airspace.
    con_airspace_polygon : shapely.geometry.Polygon
        The constrained airspace polygon.
    loc : str
        The location of the split. Either 'front' or 'back' path.

    Returns
    -------
    shapely.geometry.LineString
        The front or back path.
    int
        The node id of border_nodes_gdf that the front or back path connects to.
    """

    if loc == "front":
        # get the first segments and point to connect new path
        remain_segment = segments[:intersecting_idx]
        remain_path = linemerge(remain_segment)
        point_to_connect = Point(remain_path.coords[-1])

        # find intersecting point of first intersection idx
        intersecting_line = segments[intersecting_idx]
        intersecting_point = con_airspace_polygon.boundary.intersection(
            intersecting_line
        )

        # check if intersecting points are Point or MultiPoint
        intersecting_point = filter_multipoint_geometry(
            intersecting_point, point_to_connect
        )

        # find the nearest node to the intersecting points
        intersecting_node = list(node_rtree.nearest(intersecting_point.bounds, 1))[0]

        # extract geometry from border_node_gdf
        intersecting_node_geom = border_node_gdf.loc[intersecting_node]["geometry"]

        # create a line segment from the first_point_to_connect to the first intersecting node
        new_segment_line = LineString([point_to_connect, intersecting_node_geom])

        # extend the first legs with the first segment line
        connected_path = linemerge([remain_path, new_segment_line])

    if loc == "back":
        # get the first segments and point to connect new path
        remain_segment = segments[intersecting_idx + 1 :]
        remain_path = linemerge(remain_segment)
        point_to_connect = Point(remain_path.coords[0])

        # find intersecting point of first intersection idx
        intersecting_line = segments[intersecting_idx]
        intersecting_point = con_airspace_polygon.boundary.intersection(
            intersecting_line
        )

        # check if intersecting points are Point or MultiPoint
        intersecting_point = filter_multipoint_geometry(
            intersecting_point, point_to_connect
        )

        # find the nearest node to the intersecting points
        intersecting_node = list(node_rtree.nearest(intersecting_point.bounds, 1))[0]

        # extract geometry from border_node_gdf
        intersecting_node_geom = border_node_gdf.loc[intersecting_node]["geometry"]

        # create a line segment from the first_point_to_connect to the first intersecting node
        new_segment_line = LineString([intersecting_node_geom, point_to_connect])

        # extend the first legs with the first segment line
        connected_path = linemerge([new_segment_line, remain_path])

    return connected_path, intersecting_node


def round_geometry(geometry, round_to=1):
    """
    Round the coordinates of a shapely geometry to the given precision.

    Parameters
    ----------
    geometry : shapely.geometry.base.BaseGeometry
        The geometry to round.
    round_to : float, optional
        The precision to round to.

    Returns
    -------
    shapely.geometry.base.BaseGeometry
        The rounded geometry.
    """
    if isinstance(geometry, Point):
        return Point(round(geometry.x, round_to), round(geometry.y, round_to))
    elif isinstance(geometry, LineString):
        return LineString(
            [
                (round(coord[0], round_to), round(coord[1], round_to))
                for coord in geometry.coords
            ]
        )
    else:
        raise ValueError("Unsupported geometry type: {}".format(type(geometry)))


def filter_multipoint_geometry(intersecting_point, points_to_connect):
    """
    Filter the multipoint geometry to only keep the point that is closest to the points_to_connect

    Parameters
    ----------
    intersecting_point : shapely.geometry.multipoint.MultiPoint
        The multipoint geometry that intersects with the airspace
    points_to_connect : shapely.geometry.point.Point
        The point that is being connected to the intersecting_point

    Returns
    -------
    shapely.geometry.point.Point
        The point that is closest to the points_to_connect
    """
    # filter out MultiPoint geometries
    if isinstance(intersecting_point, MultiPoint):

        # extract single points
        list_points = list(intersecting_point)

        # check which point in the list is closest to first_point_to_connect
        intersecting_point = min(
            list_points, key=lambda x: x.distance(points_to_connect)
        )

    return intersecting_point


"""TEST FUNCTION BELOW"""


def test():
    """
    Test the functions in this module. It uses the module spawn_despawn_points
    for choosing the origin and destination points and makes a nice plot of several paths.
    """
    from os import path
    from matplotlib import pyplot as plt

    from spawn_despawn_points import (
        get_spawn_despawn_gdfs,
        get_n_origin_destination_pairs,
    )
    import config as cfg

    # Read config file for defaults
    buffer_distance = cfg.buffer_distance
    spawn_distance = cfg.spawn_distance
    min_path_distance = cfg.min_path_distance
    respect_constrained_airspace = cfg.respect_constrained_airspace

    # import airspace polygon with geopandas
    airspace_path = path.join(path.dirname(__file__), "gis/airspace/total_polygon.gpkg")
    airspace = gpd.read_file(airspace_path, driver="GPKG", layer="total_polygon")
    airspace_polygon = airspace.geometry.values[0]

    # import airspace polygon with geopandas
    airspace_path = path.join(
        path.dirname(__file__), "gis/airspace/updated_constrained_airspace.gpkg"
    )
    con_airspace = gpd.read_file(airspace_path, driver="GPKG")

    # import common elements graph with osmnx
    graph_path = path.join(
        path.dirname(__file__),
        "gis/road_network/crs_4326_cleaned_simplified_network/cleaned_simplified.graphml",
    )
    G = ox.load_graphml(graph_path)

    # convert to undirected graph
    G_undirected = ox.get_undirected(G)

    # get origin and destination points for rogue aircraft
    spawn_gdf, despawn_gdf = get_spawn_despawn_gdfs(
        airspace, buffer_distance, spawn_distance, min_path_distance
    )

    # get n origin and destination pairs
    origin_destination_pairs = get_n_origin_destination_pairs(spawn_gdf, despawn_gdf, 9)

    figure, axes = plt.subplots(nrows=3, ncols=3)

    col_counter = 0
    row_counter = 0
    # choose one random path
    for idx, origin_destination_pair in enumerate(origin_destination_pairs):

        # path that doesn't care about constrained airspace
        random_path, _ = gen_random_path(origin_destination_pair, airspace_polygon)

        if respect_constrained_airspace:
            # path that cares about constrained airspace
            random_path, turn_bool, in_constrained, _ = gen_path_through_constrained(
                random_path, con_airspace, G_undirected
            )

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
        axes[row_counter, col_counter].plot(
            airspace.geometry.boundary.values[0].xy[0],
            airspace.geometry.boundary.values[0].xy[1],
        )
        axes[row_counter, col_counter].plot(
            con_airspace.geometry.boundary.values[0].xy[0],
            con_airspace.geometry.boundary.values[0].xy[1],
        )

        row_counter += 1

    figure.tight_layout()

    plt.show()


if __name__ == "__main__":
    test()
