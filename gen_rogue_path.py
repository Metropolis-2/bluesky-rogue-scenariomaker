# %%

# import modules
import osmnx as ox
import geopandas as gpd
import pandas as pd
from os import path
import numpy as np
from shapely.geometry import LineString, MultiPoint, Point
from spawn_despawn_points import get_spawn_despawn_gdfs, get_n_origin_destination_pairs

# import airspace polygon with geopandas
airspace_path = path.join(path.dirname(__file__), 'gis/airspace/total_polygon.gpkg')
airspace = gpd.read_file(airspace_path, driver='GPKG', layer='total_polygon')

# get origin and destination points for rogue aircraft
spawn_gdf, despawn_gdf = get_spawn_despawn_gdfs(airspace, 64, 100, 12000)

# get 10 origin and destination pairs
origin_destination_pairs = get_n_origin_destination_pairs(spawn_gdf, despawn_gdf, 10)
origin_destination_pair = origin_destination_pairs[0]

origin = origin_destination_pair[0]
destination = origin_destination_pair[1]

# testing
# origin = Point(0, 0)

## test 1st quadrant
# destination = Point(16000, 16000)
# destination = Point(16000, 1000)
# destination = Point(1000, 16000)

## test 2nd quadrant
# destination = Point(-16000, 16000)
# destination = Point(-1000, 16000) still needs some work
# # destination = Point(16000, 1000)

# ## test 3rd quadrant
# # destination = Point(-16000, -16000)
# # destination = Point(-16000, -1000)
# # destination = Point(-1000, -16000)

# ## test 4th quadrant
# # destination = Point(16000, -16000)
# # destination = Point(16000, -1000)
# destination = Point(1000, -16000) still needs some work



straight_path = LineString([origin, destination])

# calculate angle between origin and destination points
x = destination.x- origin.x
y = destination.y - origin.y
angle = np.arctan2(y, x)

# find the correctt quadrant and decide how to make the unit vector
if angle < 0 and angle > -np.pi/2:
    angle = np.pi/2 - abs(angle)
    quad = 4
    
    if angle > np.pi/4:
        zone_vec = 'flip'
    else:
        zone_vec = 'normal'

# get correct angle if in the 3th quadrant
elif angle < -np.pi/2:
    quad = 3
    angle = np.pi + angle
    
    if angle > np.pi/4:
        zone_vec ='normal'
    else:
        zone_vec = 'flip'    

# get correct angle if in the 2nd quadrant
elif angle > np.pi/2:
    angle = np.pi/2 - (np.pi - angle)
    quad = 2

    if angle > np.pi/4:
        zone_vec ='flip'
    else:
        zone_vec = 'normal'

else:
    quad = 1
    if angle > np.pi/4:
        zone_vec = 'normal'
    else:
        zone_vec = 'flip'

# %%

# split the path into n segments
desired_len = 1000
num_splits = int(straight_path.length / desired_len)

path_points = [origin] + [ straight_path.interpolate((i/num_splits), normalized=True) for i in range(1, num_splits + 1)]

path_points_gdf = gpd.GeoDataFrame(geometry=path_points)

# add x and y as columns to gdf
path_points_gdf['x'] = path_points_gdf.geometry.x
path_points_gdf['y'] = path_points_gdf.geometry.y

# MultiPoint(path_points)

# %%

# find distance between first and all other points (l_0, l_1, ..., l_n)
path_points_gdf['l_n'] = path_points_gdf.apply(
    lambda row: row.geometry.distance(origin), axis=1)

# find perpendicular point to horizontal line (x_p, y_p)
if quad == 1 or quad == 4:
    path_points_gdf['x_p'] = path_points_gdf['l_n'] / np.cos(angle) + path_points_gdf['x']

elif quad == 2 or quad == 3:
    path_points_gdf['x_p'] = path_points_gdf['x'] - path_points_gdf['l_n'] / np.cos(angle) 

path_points_gdf['y_p'] = origin.y
path_points_gdf['gp'] = path_points_gdf.apply(
    lambda row: Point([row['x_p'], row['y_p']]), axis=1)

# define a distance from x,y to x_p, y_p
path_points_gdf['l_p'] = path_points_gdf.apply(
    lambda row: row['gp'].distance(row['geometry']), axis=1)

# define a unit vector from x,y to x_p, y_p
if zone_vec == 'normal':
    path_points_gdf['u_x'] = (path_points_gdf['x_p'] - path_points_gdf['x']) / path_points_gdf['l_p']
    path_points_gdf['u_y'] = (path_points_gdf['y_p'] - path_points_gdf['y']) / path_points_gdf['l_p']
elif zone_vec == 'flip':
    path_points_gdf['u_x'] = (path_points_gdf['y_p'] - path_points_gdf['y']) / path_points_gdf['l_p']
    path_points_gdf['u_y'] = (path_points_gdf['x_p'] - path_points_gdf['x']) / path_points_gdf['l_p']


# set first and last values of u_x and u_y columns to 0
path_points_gdf.loc[0, 'u_x'] = 0
path_points_gdf.loc[0, 'u_y'] = 0

# get last index of gdf
last_index = path_points_gdf.index[-1]

path_points_gdf.loc[last_index, 'u_x'] = 0
path_points_gdf.loc[last_index, 'u_y'] = 0

# variation from straight line
noise_variation = 2000
noise_df = pd.DataFrame(np.random.uniform(low=-noise_variation, high=noise_variation, 
                                        size=len(path_points_gdf)), columns=['noise'])

# multiply noise_df by u_x, u_y
noise_df['x'] = noise_df['noise'] * path_points_gdf['u_x']
noise_df['y'] = noise_df['noise'] * path_points_gdf['u_y']

# sum noise_df['x'] and noise_df['y'] to path_points_gdf['x'] and path_points_gdf['y']
new_df = pd.DataFrame(noise_df['x'] - path_points_gdf['x'], columns=['x'])
new_df['y'] = noise_df['y'] - path_points_gdf['y']

# create a gdf from new_df
new_gdf = gpd.GeoDataFrame(new_df, geometry=new_df.apply(lambda row: Point(row['x'], row['y']), axis=1))

# simplification tolerance
tolerance = 200
final_path = LineString(new_gdf.geometry).simplify(tolerance)

