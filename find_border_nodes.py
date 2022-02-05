"""
Code contains function that is used to find the border nodes of the
constrained airspace polygon.

There is one important function in this file:
    1. find_border_nodes()

test() is used to test the functionality of the functions in this file.

Code written by: Andres Morfin Veytia
Project: Metropolis 2
"""

import osmnx as ox
import geopandas as gpd
from rtree import index


def find_border_nodes(airspace_gdf, G):
    """
    Finds the border nodes of the the constrained airspace polygon.
    The funciton needs a geopandas dataframe with the airspace polygon and
    a networkx graph. The function returns gdf with the border nodes. Also
    returns an rtree with the border nodes.

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

    border_nodes_rtree : rtree.index.Index
        Rtree with border nodes.
    """

    # Convert the crs gdf to a projected crs
    airspace_gdf = airspace_gdf.to_crs(epsg=32633)

    # get node_gdf and project cs
    nodes = ox.graph_to_gdfs(G, edges=False)
    nodes = nodes.to_crs(epsg=32633)

    # buffer the airspace polygon inside -0.5 meters
    airspace_buff = airspace_gdf.buffer(-0.5)

    # see which nodes are inside the airspace border
    nodes_inside = nodes["geometry"].apply(lambda x: x.within(airspace_buff.values[0]))

    # select border nodes based on when nodes_inside is False
    border_nodes_gdf = nodes[nodes_inside == False]

    # crete an rtree
    border_nodes_rtree = index.Index()

    # loop through the border nodes and add them to the rtree
    for idx, node in border_nodes_gdf.iterrows():
        border_nodes_rtree.insert(idx, node.geometry.bounds)

    return border_nodes_gdf, border_nodes_rtree


"""TEST FUNCTION BELOW"""


def test():
    """
    Test the functionality of the function in this module.
    """
    from os import path

    # import constrained airspace polygon with geopandas
    airspace_path = path.join(
        path.dirname(__file__), "gis/airspace/updated_constrained_airspace.gpkg"
    )
    airspace = gpd.read_file(airspace_path)

    # import common elements graph with osmnx
    graph_path = path.join(
        path.dirname(__file__),
        "gis/road_network/crs_4326_cleaned_simplified_network/cleaned_simplified.graphml",
    )
    G = ox.load_graphml(graph_path)

    border_node_gdf = find_border_nodes(airspace, G)


if __name__ == "__main__":
    test()
