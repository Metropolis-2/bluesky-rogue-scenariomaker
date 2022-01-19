# %%
import osmnx as ox
import geopandas as gpd
import pandas as pd
from os import path
import numpy as np
from shapely.geometry import LineString, Point, MultiPoint
from spawn_despawn_points import get_spawn_despawn_gdfs, get_n_origin_destination_pairs
from gen_rogue_path import gen_random_path, gen_path_through_constrained

from pyproj import Transformer
from find_border_nodes import find_border_nodes
from rogue_paths_constrained import get_lat_lon_from_osm_route, qdrdist
from shapely.ops import linemerge
from matplotlib import pyplot as plt

# import airspace polygon with geopandas
airspace_path = path.join(path.dirname(__file__), 'gis/airspace/total_polygon.gpkg')
airspace = gpd.read_file(airspace_path, driver='GPKG', layer='total_polygon')
airspace_polygon = airspace.geometry.values[0]

# import airspace polygon with geopandas
con_airspace_path = path.join(path.dirname(__file__), 'gis/airspace/updated_constrained_airspace.gpkg')
con_airspace = gpd.read_file(con_airspace_path, driver='GPKG')
con_airspace_polygon = con_airspace.geometry.values[0]

# import common elements graph with osmnx
graph_path = path.join(path.dirname(__file__), 'gis/road_network/crs_4326_cleaned_simplified_network/cleaned_simplified.graphml')
G = ox.load_graphml(graph_path)

# convert to undirected graph
G_undirected = ox.get_undirected(G)

# get the border nodes
border_node_gdf, node_rtree = find_border_nodes(con_airspace, G)

# get origin and destination points for rogue aircraft
spawn_gdf, despawn_gdf = get_spawn_despawn_gdfs(airspace, 64, 100, 14000)

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
# for idx, origin_destination_pair in enumerate(origin_destination_pairs):
origin_destination_pair = origin_destination_pairs[0]

# generate a random path that doesn't care about constrained airspace
random_path = gen_random_path(origin_destination_pair, airspace_polygon, segment_length, max_deviation, simplify_tolerance)

# convert random path to one that cares about constrained airspace
merged_path, turn_bool, in_constrained, cruise_alt_changes = gen_path_through_constrained(random_path, con_airspace, G_undirected)

# plot random path and airspace
plt.figure(figsize=(8, 8))
plt.plot(merged_path.xy[0], merged_path.xy[1])
plt.plot(airspace.geometry.boundary.values[0].xy[0], airspace.geometry.boundary.values[0].xy[1])
plt.plot(con_airspace.geometry.boundary.values[0].xy[0], con_airspace.geometry.boundary.values[0].xy[1])
plt.show()

# get x,y coordinates of random path
x, y = merged_path.xy[0], merged_path.xy[1]

# transform from UTM to WGS84
lats, lons = transformer.transform(x, y)

# bearing of the first segment
achdg = qdrdist(lats[0], lons[0], lats[1], lons[1])

# first line
scenario_lines.append(f'00:00:00>CASMACHTHR 0.0')
scenario_lines.append(f'00:00:00>CREROGUE R{acidx} MP30 {lats[0]} {lons[0]} {achdg} {np.random.choice(cruise_alts)} 30')

# Create the add waypoints command
addwypoint_lines = [f'00:00:00>ADDWAYPOINTS R{acidx}']
# add the rest of the lines as waypoints
for i in range(1, len(lats)):
    if turn_bool[i] or cruise_alt_changes[i]:
        addwypoint_lines.append(f'{lats[i]} {lons[i]},,30,TURNSPD,5')
    else:
        addwypoint_lines.append(f'{lats[i]} {lons[i]},,30,FLYBY, 0')

# add the last waypoint twice
addwypoint_lines.append(f'{lats[-1]} {lons[-1]},,30,FLYBY, 0')

# join into one string
scenario_lines.append(','.join(addwypoint_lines))

# turn vnav and lnav on
scenario_lines.append(f'00:00:00>LNAV R{acidx} ON')
scenario_lines.append(f'00:00:00>VNAV R{acidx} ON')


# find the places where cruise_alt_changes is True
alt_change_idxs = np.where(cruise_alt_changes)[0]

# add the cruise altitudes to the scenario except to the last waypoint
for i in alt_change_idxs:
        curr_alt = np.random.choice(cruise_alts)
        scenario_lines.append(f'00:00:00>R{acidx} ATDIST {lats[i]} {lons[i]} 0.01 SPD R{acidx} 0')
        scenario_lines.append(f'00:00:00>R{acidx} ATDIST {lats[i]} {lons[i]} 0.01 R{acidx} ATSPD 0 ALT R{acidx} {curr_alt}')
        scenario_lines.append(f'00:00:00>R{acidx} ATDIST {lats[i]} {lons[i]} 0.01 R{acidx} ATALT {curr_alt} LNAV R{acidx} ON ')
        scenario_lines.append(f'00:00:00>R{acidx} ATDIST {lats[i]} {lons[i]} 0.01 R{acidx} ATALT {curr_alt} VNAV R{acidx} ON ')

# # add the last line to delete the aircraft
scenario_lines.append(f'00:00:00>R{acidx} ATDIST {lats[-1]} {lons[-1]} 0.01 DEL R{acidx}')

acidx += 1

# write the scenario to a file
scenario_path = path.join(path.dirname(__file__), f'scenarios/R{acidx-1}.scn')
with open(scenario_path, 'w') as f:
    for line in scenario_lines:
        f.write(f'{line}\n')
