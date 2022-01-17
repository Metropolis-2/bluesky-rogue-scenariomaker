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
from find_border_nodes import find_border_nodes
from rogue_paths_constrained import get_lat_lon_from_osm_route
from shapely.ops import linemerge

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

# %%
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
for idx, origin_destination_pair in enumerate(origin_destination_pairs):

    # generate a random path
    random_path = gen_random_path(origin_destination_pair, airspace_polygon, segment_length, max_deviation, simplify_tolerance)

    # see if random path intersects constrained airspace
    if random_path.intersects(con_airspace_polygon):

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
        
        # get the first segments
        first_segment = segments[:first_intersecting_idx]
        first_legs = linemerge(first_segment)
        first_point_to_connect = first_legs.coords[-1]

        # get the last segments
        last_segment = segments[last_intersecting_idx:]
        last_legs = linemerge(last_segment)
        last_point_to_connect = last_legs.coords[0]

        # find intersecting point of first intersection idx
        first_intersecting_line = segments[first_intersecting_idx]
        first_intersecting_point = con_airspace_polygon.boundary.intersection(first_intersecting_line)
        
        # find intersecting point of last intersection idx
        last_intersecting_line = segments[last_intersecting_idx]
        last_intersecting_point = con_airspace_polygon.boundary.intersection(last_intersecting_line)
        print(first_intersecting_point, last_intersecting_point)

        # find the nearest node to the intersecting points
        first_intersecting_node = node_rtree.nearest(first_intersecting_point)
        last_intersecting_node = node_rtree.nearest(last_intersecting_point)
        
        # create a line segment from the first_point_to_connect to the first intersecting node
        first_segment_line = LineString([first_point_to_connect, first_intersecting_node])

        # extend the first legs with the first segment line
        first_legs = linemerge([first_legs, first_segment_line])

        # create a line segment from the last_point_to_connect to the last intersecting node
        last_segment_line = LineString([last_intersecting_node, last_point_to_connect])
        
        # extend the last legs with the last segment line
        last_legs = linemerge([last_legs, last_segment_line])

        # find a path from the first intersecting node to the last intersecting node
        orig_node = ox.nearest_nodes(G_undirected, first_intersecting_node.x, first_intersecting_node.y)
        dest_node = ox.nearest_nodes(G_undirected, last_intersecting_node.x, last_intersecting_node.y)
        const_route = ox.shortest_path(G_undirected, orig_node, dest_node)

        # get lat lon from osm route
        _, _, line_gdf = get_lat_lon_from_osm_route(G_undirected, const_route)

        # convert to epsg 32633
        line_gdf = line_gdf.to_crs(epsg=32633)
        route_line_string = line_gdf.geometry.values[0]

        # merge the first legs and last legs
        legs = linemerge([first_legs, route_line_string, last_legs])

    # get x,y coordinates of random path
    x, y = random_path.xy[0], random_path.xy[1]

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
