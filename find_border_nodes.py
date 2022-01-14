"""
This module finds the border nodes of a graph.
"""

# import modules
import osmnx as ox
import geopandas as gpd
from os import path

def main():
    # import constrained airspace polygon with geopandas
    airspace_path = path.join(path.dirname(__file__), 'gis/airspace/updated_constrained_airspace.gpkg')
    airspace = gpd.read_file(airspace_path)

    # import common elements graph with osmnx
    graph_path = path.join(path.dirname(__file__), 'gis/road_network/crs_4326_cleaned_simplified_network/cleaned_simplified.graphml')
    G = ox.load_graphml(graph_path)

    node_osmid = find_border_nodes(airspace, G)

def find_border_nodes(airspace_gdf, G):
    """
    Finds the border nodes of the the constrained airspace polygon.
    The funciton needs a geopandas dataframe with the airspace polygon and
    a networkx graph. The function returns a list of node ids.

    Parameters
    ----------
    airspace_gdf : geopandas.GeoDataFrame
        GeoDataFrame with airspace polygon.

    G : nx.MultiGraph
        Graph to find border nodes of. Graph should be built
        with osmnx.get_undirected.

    Returns
    -------
    border_nodes : list
        List of border nodes.
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

    # select the indixed of thase that are false
    border_nodes = nodes_inside[nodes_inside == False].index.tolist()

    return border_nodes

if __name__ == "__main__":
    main()