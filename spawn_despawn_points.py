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

def main():
    # import airspace polygon with geopandas
    airspace_path = path.join(path.dirname(__file__), 'gis/airspace/total_polygon.gpkg')
    airspace = gpd.read_file(airspace_path, driver='GPKG', layer='total_polygon')

    # get origin and destination points for rogue aircraft
    origin_gdf, destination_gdf = get_spawn_despawn(airspace, 64, 100)

    # select minimum distance between points in km
    min_dist = 12 
    # select a random orign and destination point
    origin = origin_gdf.sample(1)
    destination = destination_gdf.sample(1)

    print(origin)
def get_spawn_despawn(airspace, buffer_distance=64, spawn_distance=100):
    """
    Takes the total airspace border and offsets it by a certain amount and
    then takes the resulting polygon and splits it into a set of points that
    that are spaced by a certain distance. The resulting points can then be used
    as origin and destination points for rogue aircraft.
    
    Parameters
    ----------
    airspace : geopandas.GeoDataFrame
        GeoDataFrame with airspace polygon.

    buffer_distance : int
        Distance to offset the airspace polygon.

    spawn_distance : int
        Distance between each spawn point.

    Returns
    -------
    origin_gdf : geopandas.GeoDataFrame
        GeoDataFrame with origin points.

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

    return origin_gdf, destination_gdf

if __name__ == '__main__':
    main()