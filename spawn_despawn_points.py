"""
This code takes the total airspace border that and offsets by a certain amount
and then takes the resulting polygon and splits it into a set of points that
that are spaced by a certain distance. The resulting points can then be used
as origin and destination points for rogue aircraft.
"""

# import modules
import osmnx as ox
import geopandas as gpd
import pandas as pd
from os import path
import numpy as np
import random

def main():
    # import airspace polygon with geopandas
    airspace_path = path.join(path.dirname(__file__), 'gis/airspace/total_polygon.gpkg')
    airspace = gpd.read_file(airspace_path, driver='GPKG', layer='total_polygon')

    # get origin and destination points for rogue aircraft
    spawn_gdf, despawn_gdf = get_spawn_despawn_gdfs(airspace, 64, 100, 12000)

    # get 10 origin and destination pairs
    origin_destination_pairs = get_n_origin_destination_pairs(spawn_gdf, despawn_gdf, 10)

def get_n_origin_destination_pairs(spawn_gdf, despawn_gdf, n):
    """
    Takes the spawn and despawn points and returns a list of n  
    randomly generated origin and destination pairs.

    Parameters
    ----------
    spawn_gdf : geopandas.GeoDataFrame
        GeoDataFrame with origin points containing a column with a list of
        destination points that are in the despawn gdf.

    despawn_gdf : geopandas.GeoDataFrame
        GeoDataFrame with destination points.

    n : int
        Number of origin and destination pairs to return.
        
    Returns
    -------
    origin_destination_pairs : list
        List of n randomly generated origin and destination pairs.
    """

    # get the number of origin and destination points
    origin_destination_pairs = []
    
    for _ in range(n):
        
        # select a random origin
        origin_gdf = spawn_gdf.sample(1)
        origin_location = origin_gdf['geometry'].values[0]

        # select a random destination from the destination list
        destination_list = origin_gdf['valid_destinations'].values[0]
        destination_idx = random.choice(destination_list)

        # get the destination point from despawn gdf
        destination_location = despawn_gdf.loc[destination_idx].values[0]

        # add to list
        origin_destination_pairs.append((origin_location, destination_location))
    
    return origin_destination_pairs

def get_spawn_despawn_gdfs(airspace, buffer_distance=64, spawn_distance=100, min_path_distance=12000):
    """
    Takes the total airspace border and offsets it by a certain amount and
    then takes the resulting polygon and splits it into a set of points that
    that are spaced by a certain distance. Also there is a column in the 
    origin gdf that gives a list of the destination points further than a
    minimum distance.
    
    The resulting points can then be used
    as origin and destination points for rogue aircraft.
    
    Parameters
    ----------
    airspace : geopandas.GeoDataFrame
        GeoDataFrame with airspace polygon.

    buffer_distance : int
        Distance to offset the airspace polygon.

    spawn_distance : int
        Distance between each spawn point.

    min_path_distance : int
        Minimum distance between each potential spawn/despawn point.

    Returns
    -------
    origin_gdf : geopandas.GeoDataFrame
        GeoDataFrame with origin points. Also contains a column with a list of
        destination points further than a minimum distance (valid_destinations).

    destination_gdf : geopandas.GeoDataFrame
        GeoDataFrame with destination points.
    """

    # offset the airspace by 64 meters (2x separation distance)
    rogue_poly = airspace['geometry'].buffer(buffer_distance)

    # get the boundary as a linestring
    rogue_line = rogue_poly.values[0].boundary

    # get radius, circumference
    circumference = rogue_line.length
    radius = circumference / (2 * np.pi)

    # get the number of points
    num_splits = int(circumference / spawn_distance)

    # split the linestring into multipoints
    rogue_points = [rogue_line.interpolate((i/num_splits), normalized=True) for i in range(1, num_splits + 1)]

    # divide list into those with even indices and those with odd indices
    # and put the coordinates ina geodataframe
    origin_gdf = gpd.GeoDataFrame(geometry=rogue_points[::2], crs=airspace.crs)
    destination_gdf = gpd.GeoDataFrame(geometry=rogue_points[1::2], crs=airspace.crs)

    # get the distance between each origin and destination point
    valid_destinations = []
    for _, row in origin_gdf.iterrows():
        
        # find potential destination points when distance is greater than minimum
        potential_destinations = destination_gdf.loc[
            (destination_gdf['geometry'].distance(row['geometry']) > min_path_distance)
            ].index.tolist()
        
        # append to a list of valid destinations for a particular origin
        valid_destinations.append(potential_destinations)

    # add the list of valid destinations to the origin gdf
    origin_gdf['valid_destinations'] = valid_destinations

    return origin_gdf, destination_gdf

if __name__ == '__main__':
    main()