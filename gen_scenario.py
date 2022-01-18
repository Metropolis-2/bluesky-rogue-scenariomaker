# %%
import osmnx as ox
import geopandas as gpd
import pandas as pd
from os import path
import numpy as np
from shapely.geometry import LineString, Point, MultiPoint
from spawn_despawn_points import get_spawn_despawn_gdfs, get_n_origin_destination_pairs
from gen_rogue_path import gen_random_path
from rogue_paths_constrained import qdrdist
from scipy.spatial.transform import Rotation as R
from pyproj import Transformer
from find_border_nodes import find_border_nodes
from rogue_paths_constrained import get_lat_lon_from_osm_route
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
# generate a random path
random_path = gen_random_path(origin_destination_pair, airspace_polygon, segment_length, max_deviation, simplify_tolerance)

# # %%
# # plot random path and airspace
# plt.figure(figsize=(8, 8))
# plt.plot(random_path.xy[0], random_path.xy[1])
# plt.plot(airspace.geometry.boundary.values[0].xy[0], airspace.geometry.boundary.values[0].xy[1])
# plt.plot(con_airspace.geometry.boundary.values[0].xy[0], con_airspace.geometry.boundary.values[0].xy[1])

# %%

# split random path into individual segments
segments = list(map(LineString, zip(random_path.coords[:-1], random_path.coords[1:])))

intersecting_idx = []
# check which segments intersect with the airspace
for idx, segment in enumerate(segments):

    # check if the individual segment intesects with the airspace
    if segment.intersects(con_airspace_polygon):
        intersecting_idx.append(idx)

# get first and last intersection
first_intersecting_idx = intersecting_idx[0]
last_intersecting_idx = intersecting_idx[-1]

# %%
# get the first segments
first_segment = segments[:first_intersecting_idx]
first_legs = linemerge(first_segment)
first_point_to_connect = first_legs.coords[-1]

# get the last segments
last_segment = segments[last_intersecting_idx + 1:]
last_legs = linemerge(last_segment)
last_point_to_connect = last_legs.coords[0]

# # %%
# # plot random path and airspace
# plt.figure(figsize=(8, 8))
# plt.plot(first_legs.xy[0], first_legs.xy[1])
# plt.plot(last_legs.xy[0], last_legs.xy[1])
# plt.plot(airspace.geometry.boundary.values[0].xy[0], airspace.geometry.boundary.values[0].xy[1])
# plt.plot(con_airspace.geometry.boundary.values[0].xy[0], con_airspace.geometry.boundary.values[0].xy[1])

# %%
# find intersecting point of first intersection idx
first_intersecting_line = segments[first_intersecting_idx]
first_intersecting_point = con_airspace_polygon.boundary.intersection(first_intersecting_line)

# find intersecting point of last intersection idx
last_intersecting_line = segments[last_intersecting_idx]
last_intersecting_point = con_airspace_polygon.boundary.intersection(last_intersecting_line)

# check if intersecting points are Point or MultiPoint
if isinstance(first_intersecting_point, MultiPoint):

    # extract single points
    list_points = list(first_intersecting_point)

    # check which point in the list is closest to first_point_to_connect
    first_intersecting_point = min(list_points, key=lambda x: x.distance(Point(first_point_to_connect)))

if isinstance(last_intersecting_point, MultiPoint):

    # extract single points
    list_points = list(last_intersecting_point)

    # check which point in the list is closest to first_point_to_connect
    last_intersecting_point = min(list_points, key=lambda x: x.distance(Point(last_point_to_connect)))


# # %%
# # plot random path and airspace
# plt.figure(figsize=(8, 8))
# plt.plot(first_legs.xy[0], first_legs.xy[1])
# plt.plot(last_legs.xy[0], last_legs.xy[1])
# plt.plot(first_intersecting_point.xy[0], first_intersecting_point.xy[1], 'ro')
# plt.plot(last_intersecting_point.xy[0], last_intersecting_point.xy[1], 'ro')
# plt.plot(airspace.geometry.boundary.values[0].xy[0], airspace.geometry.boundary.values[0].xy[1])
# plt.plot(con_airspace.geometry.boundary.values[0].xy[0], con_airspace.geometry.boundary.values[0].xy[1])

# %%
# find the nearest node to the intersecting points
first_intersecting_node = list(node_rtree.nearest(first_intersecting_point.bounds, 1))[0]
last_intersecting_node = list(node_rtree.nearest(last_intersecting_point.bounds, 1))[0]

# extract geometry from border_node_gdf
first_intersecting_node_geom = border_node_gdf.loc[first_intersecting_node]['geometry']
last_intersecting_node_geom = border_node_gdf.loc[last_intersecting_node]['geometry']

# create a line segment from the first_point_to_connect to the first intersecting node
first_segment_line = LineString([first_point_to_connect, first_intersecting_node_geom])

# extend the first legs with the first segment line
first_legs = linemerge([first_legs, first_segment_line])

# create a line segment from the last_point_to_connect to the last intersecting node
last_segment_line = LineString([last_intersecting_node_geom, last_point_to_connect])

# extend the last legs with the last segment line
last_legs = linemerge([last_legs, last_segment_line])
# %%
# plot random path and airspace
# plt.figure(figsize=(8, 8))
# plt.plot(first_legs.xy[0], first_legs.xy[1])
# plt.plot(last_legs.xy[0], last_legs.xy[1])
# plt.plot(first_intersecting_point.xy[0], first_intersecting_point.xy[1], 'ro')
# plt.plot(last_intersecting_point.xy[0], last_intersecting_point.xy[1], 'ro')
# plt.plot(airspace.geometry.boundary.values[0].xy[0], airspace.geometry.boundary.values[0].xy[1])
# plt.plot(con_airspace.geometry.boundary.values[0].xy[0], con_airspace.geometry.boundary.values[0].xy[1])

# %%
# find a path from the first intersecting node to the last intersecting node
const_route = ox.shortest_path(G_undirected, first_intersecting_node, last_intersecting_node)

# get lat lon from osm route
_, _, line_gdf = get_lat_lon_from_osm_route(G_undirected, const_route)

# convert to epsg 32633
line_gdf = line_gdf.to_crs(epsg=32633)
route_line_string = line_gdf.geometry.values[0]

# merge the first legs and last legs
legs = linemerge([first_legs, route_line_string, last_legs])
# %%
plt.figure(figsize=(8, 8))
plt.plot(first_legs.xy[0], first_legs.xy[1])
plt.plot(last_legs.xy[0], last_legs.xy[1])
plt.plot(route_line_string.xy[0], route_line_string.xy[1])
plt.plot(first_intersecting_point.xy[0], first_intersecting_point.xy[1], 'ro')
plt.plot(last_intersecting_point.xy[0], last_intersecting_point.xy[1], 'ro')
plt.plot(airspace.geometry.boundary.values[0].xy[0], airspace.geometry.boundary.values[0].xy[1])
plt.plot(con_airspace.geometry.boundary.values[0].xy[0], con_airspace.geometry.boundary.values[0].xy[1])

# %%
# get x,y coordinates of random path
x, y = legs.xy[0], legs.xy[1]

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
