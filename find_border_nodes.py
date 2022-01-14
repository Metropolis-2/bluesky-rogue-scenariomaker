"""
This module finds the border nodes of a graph.
"""

# import modules
import osmnx as ox
import geopandas as gpd
import pandas as pd
from os import path

def main():
    # import constrained airspace polygon with geopandas
    airspace_path = path.join(path.dirname(__file__), 'gis/airspace/updated_constrained_airspace.gpkg')
    airspace = gpd.read_file(airspace_path)

    # import common elements graph with osmnx
    graph_path = path.join(path.dirname(__file__), 'gis/road_network/crs_4326_cleaned_simplified_network/cleaned_simplified.graphml')
    G = ox.load_graphml(graph_path)

    border_node_gdf = find_border_nodes(airspace, G)

def find_border_nodes(airspace_gdf, G):
    """
    Finds the border nodes of the the constrained airspace polygon.
    The funciton needs a geopandas dataframe with the airspace polygon and
    a networkx graph. The function returns gdf with the border nodes.

    Parameters
    ----------
    airspace_gdf : geopandas.GeoDataFrame
        GeoDataFrame with airspace polygon.

    G : nx.MultiGraph
        Graph to find border nodes of. Graph should be built
        with osmnx.get_undirected.

    Returns
    -------
    border_nodes_gdf : geopandas.GeoDataFrame
        GeoDataFrame with border nodes.
    """

    # Convert the crs gdf to a projected crs
    airspace_gdf = airspace_gdf.to_crs(epsg=32633)

    # get node_gdf and project cs
    nodes = ox.graph_to_gdfs(G, edges=False)
    nodes = nodes.to_crs(epsg=32633)

    # buffer the airspace polygon inside -0.5 meters
    airspace_buff = airspace_gdf.buffer(-0.5)

    # see which nodes are inside the airspace border
    nodes_inside = nodes['geometry'].apply(lambda x: x.within(airspace_buff.values[0]))

    # select border nodes based on when nodes_inside is False
    border_nodes_gdf = nodes[nodes_inside == False]

    # convert back to 4326
    border_nodes_gdf = border_nodes_gdf.to_crs(epsg=4326)

    return border_nodes_gdf

if __name__ == "__main__":
    main()