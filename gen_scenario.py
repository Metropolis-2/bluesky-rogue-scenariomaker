# %%
import osmnx as ox
import geopandas as gpd
import pandas as pd
from os import path
import numpy as np
from shapely.geometry import LineString, Point
from spawn_despawn_points import get_spawn_despawn_gdfs, get_n_origin_destination_pairs
from gen_rogue_path import gen_random_path
from rogue_paths_constrained import qdrdist
from scipy.spatial.transform import Rotation as R
from pyproj import Transformer

# import airspace polygon with geopandas
airspace_path = path.join(path.dirname(__file__), 'gis/airspace/total_polygon.gpkg')
airspace = gpd.read_file(airspace_path, driver='GPKG', layer='total_polygon')

# get origin and destination points for rogue aircraft
spawn_gdf, despawn_gdf = get_spawn_despawn_gdfs(airspace, 64, 100, 12000)

# get n origin and destination pairs
origin_destination_pairs = get_n_origin_destination_pairs(spawn_gdf, despawn_gdf, 1)

# transformer object from UTM to WGS84
transformer = Transformer.from_crs(32633, 4326)

# start a list of strings
scenario_lines = []

# defaul values
segment_length = 1000
max_deviation = 3000
simplify_tolerance = 400

acid = 1
# choose one random path
for idx, origin_destination_pair in enumerate(origin_destination_pairs):

    random_path = gen_random_path(origin_destination_pair, segment_length, max_deviation, simplify_tolerance)
    x, y = random_path.xy[0], random_path.xy[1]

    # transform from UTM to WGS84
    lats, lons = transformer.transform(x, y)

    # bearing of the first segment
    achdg = qdrdist(lats[0], lons[0], lats[1], lons[1])

    # first line
    scenario_lines.append(f'00:00:00>CREROGUE R{acid} MP30 {lats[0]} {lons[0]} {achdg} 30 30')

    # add the rest of the lines as waypoints
    for i in range(1, len(lats)):
        scenario_lines.append(f'00:00:00>ADDWPT R{acid} {lats[i]} {lons[i]}')

    # add the last line to delete the aircraft
    scenario_lines.append(f'00:00:00>R{acid} ATIDST {lats[-1]} {lons[-1]} 0.01 DEL R{acid}')

    acid += 1

# write the scenario to a file
scenario_path = path.join(path.dirname(__file__), 'scenarios/R_1.scn')
with open(scenario_path, 'w') as f:
    for line in scenario_lines:
        f.write(f'{line}\n')

# %%
