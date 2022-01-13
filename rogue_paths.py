import osmnx as ox
import networkx as nx
import geopandas as gpd
from osmnx.utils_graph import get_undirected
import pandas as pd
# import graph_funcs as gfc
from os import path
import matplotlib.pyplot as plt
import numpy as np

def main():
    # import common elements graph
    graph_path = path.join(path.dirname(__file__), 'gis/road_network/crs_4326_cleaned_simplified_network/cleaned_simplified.graphml')
    G = ox.load_graphml(graph_path)

    # convert to undirected graph
    G_undirected = ox.get_undirected(G)

    orig = 303933494
    dest = 32637455
    route = ox.shortest_path(G_undirected, orig, dest)

    # fig, ax = ox.plot_graph_route(G_undirected, route, route_color="y", route_linewidth=6, node_size=0)
    # plt.show()

    # get lat and lon from route and turninfor
    lat, lon = get_lat_lon_from_osm_route(G_undirected, route)
    turn_bool, turn_speed, turn_coords = get_turn_arrays(lat, lon)


def get_lat_lon_from_osm_route(G, route):
    """
    Get lat and lon from an osmnx route (list of nodes) and nx.MultGraph.
    The function returns two numpy arrays with the lat and lon of route.

    Parameters
    ----------
    G : nx.MultiGraph
        Graph to get lat and lon from. Graph should be built
        with osmnx.get_undirected.

    route : list
        List of nodes to build edge and to get lat lon from
    
    Returns
    -------
    lat : numpy.ndarray
        Array with latitudes of route

    lon : numpy.ndarray
        Array with longitudes of route

    """
    # add first node to route
    lon = np.array(G.nodes[route[0]]["x"])
    lat = np.array(G.nodes[route[0]]["y"])


    # loop through the rest for loop only adds from second point of edge
    for u, v in zip(route[:-1], route[1:]):
        # if there are parallel edges, select the shortest in length
        data = list(G.get_edge_data(u, v).values())[0]

        # extract coords from linestring
        xs, ys = data["geometry"].xy

        # Check if geometry of edge is in correct order
        if G.nodes[u]["x"] != data["geometry"].coords[0][0]:

            # flip if in wrong order
            xs = xs[::-1]
            ys = ys[::-1]
        
        # only add from the second point of linestring
        lon = np.append(lon, xs[1:])
        lat = np.append(lat, ys[1:])
    
    return lat, lon

def get_turn_arrays(lat, lon, cutoff_angle=25):
    """
    Get turn arrays from latitude and longitude arrays.
    The function returns three arrays with the turn boolean, turn speed and turn coordinates.

    Turn speed depends on the turn angle.

        - Speed set to 0 for no turn.
        - Speed to 10 knots for angles smaller than 45 degrees.
        - Speed to 5 knots for turning angles between 45 and 90 degrees.
        - Speed to 2 knots for turning angles larger tha 90 degrees.

    Parameters
    ----------
    lat : numpy.ndarray
        Array with latitudes of route

    lon : numpy.ndarray
        Array with longitudes of route

    cutoff_angle : int
        Cutoff angle for turning. Default is 25.

    Returns
    -------
    turn_bool : numpy.ndarray
        Array with boolean values for turns.
    
    turn_speed : numpy.ndarray
        Array with turn speed. If no turn, speed is 0.
    
    turn_coords : numpy.ndarray
        Array with turn coordinates. If no turn then it has (-9999.9, -9999.9)

    """
    
    # Define empty arrays that are same size as lat and lon
    turn_speed = np.zeros(len(lat))
    turn_bool = np.array([False]*len(lat), dtype=np.bool8)
    turn_coords = np.array([(-9999.9,-9999.9)]*len(lat), dtype='f,f')
    
    # Initialize variables for the loop
    lat_prev = lat[0]
    lon_prev = lon[0]

    # loop thru the points to calculate turn angles
    for i in range(1, len(lat)-1):
        # reset some values for the loop
        lat_cur = lat[i]
        lon_cur = lon[i]
        lat_next = lat[i+1]
        lon_next = lon[i+1]
        
        # calculate angle between points
        d1 = qdrdist(lat_prev,lon_prev,lat_cur,lon_cur)
        d2 = qdrdist(lat_cur,lon_cur,lat_next,lon_next)

        # fix angles that are larger than 180 degrees
        angle = abs(d2 - d1)
        angle = 360 - angle if angle > 180 else angle

        # give the turn speeds based on the angle
        if angle > cutoff_angle and i != 0:

            # set turn bool to true and get the turn coordinates
            turn_bool[i] = True
            turn_coords[i] = (lat[i],lon[i])

            # calculate the turn speed based on the angle.
            if angle<100:
                turn_speed[i] = 10
            elif angle<150:
                turn_speed[i] = 5
            else:
                turn_speed[i] = 2
        else:
            turn_coords[i] = (-9999.9,-9999.9)

        # update the previous values at the end of the loop
        lat_prev=lat_cur
        lon_prev=lon_cur
    
    return turn_bool, turn_speed, turn_coords

# from geo.py
def rwgs84(latd):
    """ Calculate the earths radius with WGS'84 geoid definition
        In:  lat [deg] (latitude)
        Out: R   [m]   (earth radius) """
    lat    = np.radians(latd)
    a      = 6378137.0       # [m] Major semi-axis WGS-84
    b      = 6356752.314245  # [m] Minor semi-axis WGS-84
    coslat = np.cos(lat)
    sinlat = np.sin(lat)

    an     = a * a * coslat
    bn     = b * b * sinlat
    ad     = a * coslat
    bd     = b * sinlat

    # Calculate radius in meters
    r = np.sqrt((an * an + bn * bn) / (ad * ad + bd * bd))

    return r

# from geo.py
def qdrdist(latd1, lond1, latd2, lond2):
    """ Calculate bearing, using WGS'84
        In:
            latd1,lond1 en latd2, lond2 [deg] :positions 1 & 2
        Out:
            qdr [deg] = heading from 1 to 2
    """

    # Convert to radians
    lat1 = np.radians(latd1)
    lon1 = np.radians(lond1)
    lat2 = np.radians(latd2)
    lon2 = np.radians(lond2)

    # Bearing from Ref. http://www.movable-type.co.uk/scripts/latlong.html
    coslat1 = np.cos(lat1)
    coslat2 = np.cos(lat2)

    qdr = np.degrees(np.arctan2(np.sin(lon2 - lon1) * coslat2,
                                coslat1 * np.sin(lat2) -
                                np.sin(lat1) * coslat2 * np.cos(lon2 - lon1)))

    return qdr
    
if __name__ == "__main__":
    main()