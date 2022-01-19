"""
The main code to run for generating scenarios.

Code written by: Andres Morfin Veytia
Project: Metropolis 2
"""

from os import path
import osmnx as ox
import geopandas as gpd
import numpy as np

from spawn_despawn_points import get_spawn_despawn_gdfs, get_n_origin_destination_pairs
from gen_scenario import generate_scenario
import config as cfg


def main():
    """
    The main code to run for generating scenarios.
    """

    # Read config file for defaults
    n_paths = cfg.n_paths
    buffer_distance = cfg.buffer_distance
    spawn_distance = cfg.spawn_distance
    min_path_distance = cfg.min_path_distance
    segment_length = cfg.segment_length
    max_deviation = cfg.max_deviation
    simplify_tolerance = cfg.simplify_tolerance
    respect_constrained_airspace = cfg.respect_constrained_airspace
    cruise_alts = cfg.cruise_alts

    # import airspace polygon with geopandas
    airspace_path = path.join(path.dirname(__file__), 'gis/airspace/total_polygon.gpkg')
    airspace = gpd.read_file(airspace_path, driver='GPKG', layer='total_polygon')

    # import airspace polygon with geopandas
    con_airspace_path = path.join(path.dirname(__file__), 'gis/airspace/updated_constrained_airspace.gpkg')
    con_airspace = gpd.read_file(con_airspace_path, driver='GPKG')

    # import common elements graph with osmnx
    graph_path = path.join(path.dirname(__file__), 'gis/road_network/crs_4326_cleaned_simplified_network/cleaned_simplified.graphml')
    G = ox.load_graphml(graph_path)

    # convert to undirected graph
    G_undirected = ox.get_undirected(G)

    # get origin and destination points for rogue aircraft
    spawn_gdf, despawn_gdf = get_spawn_despawn_gdfs(airspace, buffer_distance, spawn_distance, min_path_distance)

    # get n origin and destination pairs
    origin_destination_pairs = get_n_origin_destination_pairs(spawn_gdf, despawn_gdf, n_paths)

    # choose one random path
    for idx, origin_destination_pair in enumerate(origin_destination_pairs):

        # creeate the scenario with idx and origin_destination_pair
        generate_scenario(idx, origin_destination_pair, airspace, con_airspace, G_undirected, segment_length, 
                        max_deviation, simplify_tolerance, cruise_alts, respect_constrained_airspace)

if __name__ == '__main__':
    main()