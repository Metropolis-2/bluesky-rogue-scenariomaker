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
airspace_polygon = airspace.geometry.values[0]

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

# cruise altitudes (in feet)
cruise_alts = np.arange(30, 510, 30)

acidx = 1
# choose one random path
for idx, origin_destination_pair in enumerate(origin_destination_pairs):

    random_path = gen_random_path(origin_destination_pair, airspace_polygon, segment_length, max_deviation, simplify_tolerance)

    # # split the random path linestring into segments
    # segments = list(map(LineString, zip(random_path.coords[:-1], random_path.coords[1:])))

    x, y = random_path.xy[0], random_path.xy[1]

    # check if any points that are not first or last


    # transform from UTM to WGS84
    lats, lons = transformer.transform(x, y)

    # bearing of the first segment
    achdg = qdrdist(lats[0], lons[0], lats[1], lons[1])

    # first line
    scenario_lines.append(f'00:00:00>CREROGUE R{acidx} MP30 {lats[0]} {lons[0]} {achdg} 30 30')
    
    # Make waypoints flyover
    scenario_lines.append(f'00:00:00>ADDWPTMODE R{acidx} FLYOVER')

    # add the rest of the lines as waypoints
    for i in range(1, len(lats)):
        scenario_lines.append(f'00:00:00>ADDWPT R{acidx} {lats[i]} {lons[i]},,30')

    # turn vnav and lnav on
    scenario_lines.append(f'00:00:00>LNAV R{acidx} ON')
    scenario_lines.append(f'00:00:00>VNAV R{acidx} ON')

    # add the cruise altitudes to the scenario except to the last waypoint
    for i in range(1, len(lats) - 1):
            curr_alt = np.random.choice(cruise_alts)
            scenario_lines.append(f'00:00:00>R{acidx} ATDIST {lats[i]} {lons[i]} 0.01 SPD R{acidx} 0')
            scenario_lines.append(f'00:00:00>R{acidx} ATDIST {lats[i]} {lons[i]} 0.01 R{acidx} ATSPD 0 ALT R{acidx} {curr_alt}')
            scenario_lines.append(f'00:00:00>R{acidx} ATDIST {lats[i]} {lons[i]} 0.01 R{acidx} ATALT {curr_alt} LNAV R{acidx} ON ')
            scenario_lines.append(f'00:00:00>R{acidx} ATDIST {lats[i]} {lons[i]} 0.01 R{acidx} ATALT {curr_alt} VNAV R{acidx} ON ')


    # add the last line to delete the aircraft
    scenario_lines.append(f'00:00:00>R{acidx} ATDIST {lats[-1]} {lons[-1]} 0.01 DEL R{acidx}')

    acidx += 1

# write the scenario to a file
scenario_path = path.join(path.dirname(__file__), f'scenarios/R{acidx-1}.scn')
with open(scenario_path, 'w') as f:
    for line in scenario_lines:
        f.write(f'{line}\n')

# %%
