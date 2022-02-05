"""
This module is used to generate scenario files for rogue aircraft.

The other functions are helper and test functions for the two important functions.

The important imported modules from this project are:
    1. gen_rogue_path used to generate the random path for the rogue aircraft.

Code written by: Andres Morfin Veytia
Project: Metropolis 2
"""

from os import path
import osmnx as ox
import geopandas as gpd
import numpy as np
from pyproj import Transformer

from gen_rogue_path import gen_random_path, gen_path_through_constrained
from rogue_paths_constrained import qdrdist


def generate_scenario(
    acidx,
    origin_destination_pair,
    airspace,
    con_airspace,
    G,
    segment_length,
    max_deviation,
    simplify_tolerance,
    cruise_alts,
    respect_constrained_airspace=True,
):
    """
    Generates a scenario for a given aircraft.

    The airspace GeoDataFrames must be in epsg:32633.
    However, the undirected graph must be in epsg:4326.

    The code converts everything to epsg:32633 for
    calculations. And right before writing the scenario to a file,
    it converts back to epsg:4326.

    Parameters
    ----------
    acidx : int
        The aircraft index.
    origin_destination_pair : tuple
        The origin and destination of the aircraft.
    airspace : geopandas.GeoDataFrame
        The total airspace.
    con_airspace : geopandas.GeoDataFrame
        The constrained airspace.
    G : networkx.MultiGraph
        The constrained airspace graph (undirected).
    segment_length : float
        The approximate length of the segments.
        Note that if using cosntrained airspace, this
        only applies to open airspace.
    max_deviation : float
        The maximum deviation from the straight line.
    simplify_tolerance : float
        The tolerance for simplification of the path.
        See shapely.ops.simplify.
    cruise_alts : numpy.ndarray
        The cruise altitudes that rogue aircraft
        may fly at.
    respect_constrained_airspace : bool
        If True, the aircraft will respect
        constrained airspace.

    Returns
    -------
    A scenario for the aircraft that is saved to scenarios/R{acidx}.scn.
    """
    # transformer object from UTM to WGS84
    transformer = Transformer.from_crs(32633, 4326)

    # get airspace polygons
    airspace_polygon = airspace.geometry.values[0]
    con_airspace_polygon = con_airspace.geometry.values[0]

    # generate a random path that doesn't care about constrained airspace
    random_path, cruise_alt_changes = gen_random_path(
        origin_destination_pair,
        airspace_polygon,
        segment_length,
        max_deviation,
        simplify_tolerance,
    )

    if respect_constrained_airspace:
        # convert random path to one that cares about constrained airspace
        (
            merged_path,
            turn_bool,
            in_constrained,
            cruise_alt_changes,
        ) = gen_path_through_constrained(random_path, con_airspace, G)
    else:
        # when not respecting constrained airspace, just use the random path
        merged_path = random_path

        # also there are no turns in the path so use cruise_alt_changes
        turn_bool = cruise_alt_changes

    # get x,y coordinates of random path
    x, y = merged_path.xy[0], merged_path.xy[1]

    # transform from UTM to WGS84
    lats, lons = transformer.transform(x, y)

    # generate the scenario text and path
    scenario_lines, scenario_path = create_scenario_text(
        acidx, lats, lons, cruise_alts, cruise_alt_changes, turn_bool
    )

    with open(scenario_path, "w") as f:
        for line in scenario_lines:
            f.write(f"{line}\n")


"""HELPER FUNCTIONS BELOW"""


def create_scenario_text(acidx, lats, lons, cruise_alts, cruise_alt_changes, turn_bool):
    """
    Creates the scenario text and file path for a given aircraft.

    Parameters
    ----------
    acidx : int
        The aircraft index.
    lats : numpy.ndarray
        The latitudes of the path.
    lons : numpy.ndarray
        The longitudes of the path.
    cruise_alts : numpy.ndarray
        The possible cruise altitudes of the path.
    cruise_alt_changes : numpy.ndarray
        A boolean array indicating whether the
        cruise altitude changes at a given waypoint.
    turn_bool : numpy.ndarray
        A boolean array indicating whether the
        aircraft turns at a given waypoint.

    Returns
    -------
    scenario_lines : list
        The scenario text. Each item is a line in the scenario.
    scenario_path : str
        The file path to the scenario file scenarios/R{acidx}.scn.
    """
    # bearing of the first segment
    achdg = qdrdist(lats[0], lons[0], lats[1], lons[1])

    # start a list of strings
    scenario_lines = []

    # first line
    scenario_lines.append(
        f"00:00:00>CREROGUE R{acidx} MP30 {lats[0]} {lons[0]} {achdg} {np.random.choice(cruise_alts)} 30"
    )

    # Create the add waypoints command
    addwypoint_lines = [f"00:00:00>ADDWAYPOINTS R{acidx}"]
    # add the rest of the lines as waypoints
    for i in range(1, len(lats)):
        if turn_bool[i] or cruise_alt_changes[i]:
            addwypoint_lines.append(f"{lats[i]} {lons[i]},,30,TURNSPD,5")
        else:
            addwypoint_lines.append(f"{lats[i]} {lons[i]},,30,FLYBY, 0")

    # add the last waypoint twice
    addwypoint_lines.append(f"{lats[-1]} {lons[-1]},,30,FLYBY, 0")

    # join into one string
    scenario_lines.append(",".join(addwypoint_lines))

    # turn vnav and lnav on
    scenario_lines.append(f"00:00:00>LNAV R{acidx} ON")
    scenario_lines.append(f"00:00:00>VNAV R{acidx} ON")

    # find the places where cruise_alt_changes is True
    alt_change_idxs = np.where(cruise_alt_changes)[0]

    # add the cruise altitudes to the scenario except to the last waypoint
    for i in alt_change_idxs:
        curr_alt = np.random.choice(cruise_alts)
        scenario_lines.append(
            f"00:00:00>R{acidx} ATDIST {lats[i]} {lons[i]} 0.01 SPD R{acidx} 0"
        )
        scenario_lines.append(
            f"00:00:00>R{acidx} ATDIST {lats[i]} {lons[i]} 0.01 R{acidx} ATSPD 0 ALT R{acidx} {curr_alt}"
        )
        scenario_lines.append(
            f"00:00:00>R{acidx} ATDIST {lats[i]} {lons[i]} 0.01 R{acidx} ATALT {curr_alt} LNAV R{acidx} ON "
        )
        scenario_lines.append(
            f"00:00:00>R{acidx} ATDIST {lats[i]} {lons[i]} 0.01 R{acidx} ATALT {curr_alt} VNAV R{acidx} ON "
        )

    # # add the last line to delete the aircraft
    scenario_lines.append(
        f"00:00:00>R{acidx} ATDIST {lats[-1]} {lons[-1]} 0.01 DEL R{acidx}"
    )

    # write the scenario to a file
    scenario_path = path.join(path.dirname(__file__), f"scenarios/R{acidx}.scn")

    return scenario_lines, scenario_path


"""TEST FUNCTION BELOW"""


def test():
    from matplotlib import pyplot as plt

    from spawn_despawn_points import (
        get_spawn_despawn_gdfs,
        get_n_origin_destination_pairs,
    )

    # defaul values
    segment_length = 1000
    max_deviation = 3000
    simplify_tolerance = 400
    buffer_distance = 64
    spawn_distance = 100
    min_path_distance = 12000
    n_paths = 2
    respect_constrained_airspace = True
    cruise_alts = np.arange(30, 510, 30)

    # import airspace polygon with geopandas
    airspace_path = path.join(path.dirname(__file__), "gis/airspace/total_polygon.gpkg")
    airspace = gpd.read_file(airspace_path, driver="GPKG", layer="total_polygon")

    # import airspace polygon with geopandas
    con_airspace_path = path.join(
        path.dirname(__file__), "gis/airspace/updated_constrained_airspace.gpkg"
    )
    con_airspace = gpd.read_file(con_airspace_path, driver="GPKG")

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
    origin_destination_pairs = get_n_origin_destination_pairs(
        spawn_gdf, despawn_gdf, n_paths
    )

    # choose one random path
    for idx, origin_destination_pair in enumerate(origin_destination_pairs):

        # creeate the scenario with idx and origin_destination_pair
        generate_scenario(
            idx,
            origin_destination_pair,
            airspace,
            con_airspace,
            G_undirected,
            segment_length,
            max_deviation,
            simplify_tolerance,
            cruise_alts,
            respect_constrained_airspace,
        )


if __name__ == "__main__":
    test()
