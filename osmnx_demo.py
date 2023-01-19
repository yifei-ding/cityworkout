import networkx as nx
import osmnx as ox
import datetime


from queue import PriorityQueue
from shapely.geometry import LineString, Point
from geojson import Feature, FeatureCollection, dump
import pickle

def temp_dijkstra(G, start_vertex, end_vertex):
    """shortest path algorithm based on dijkstra's"""
    nodes = list(G.nodes)
    D = {v: float('inf') for v in nodes}  # initialize shortest distance for all nodes
    path = {v: -1 for v in nodes}  # initialize previous node along shortest path
    D[start_vertex] = 0  # the shortest path to the start vertex is 0
    path[start_vertex] = None  # the previous node of the start_vertex is None
    visited = []
    pq = PriorityQueue()
    pq.put((0, start_vertex))
    while not pq.empty():
        # print('priority queue: {0}'.format(pq.queue))
        (dist, current_vertex) = pq.get()
        if current_vertex == end_vertex:
            result = []
            node = end_vertex
            while node != start_vertex:
                result.append(node)
                node = path[node]
            result.append(start_vertex)
            return list(reversed(result))
        visited.append(current_vertex)
        # print('current vertex: ', current_vertex)
        # print('neighbours: ', list(G.adj[current_vertex]))
        for neighbour in list(G.adj[current_vertex]):
            if neighbour not in visited:
                # relax
                distance = G.adj[current_vertex][neighbour][0][
                    'length']  # get distance between current_vertex and the neighbour
                new_cost = D[current_vertex] + distance
                previous_cost = D[neighbour]
                if new_cost < previous_cost:
                    # if found shorter distance, update cost
                    pq.put((new_cost, neighbour))
                    # print('set ', neighbour,' = ', new_cost)
                    D[neighbour] = new_cost
                    path[neighbour] = current_vertex

    return None

def save_path(G, nodes_on_shortest_path, file_name):
    points = []
    nodes = G.nodes
    if nodes_on_shortest_path is not None:
        for item in nodes_on_shortest_path:
            x = nodes[item]['x']
            y = nodes[item]['y']
            new_point = Point([x,y])
            points.append(new_point)
        linestring_for_export = LineString(points)

        features = [Feature(geometry=linestring_for_export)]
        feature_collection = FeatureCollection(features)
        fileName_str = file_name + '.geojson'
        with open(fileName_str, 'w') as f:
            dump(feature_collection, f)
        print('Saved!')
    else:
        print('No path found. Nothing saved')

G = ox.graph_from_point((40.4455, -3.6915), dist=2000, network_type='walk')
# query = {'city': 'Madrid'}
# G = ox.graph_from_place(query, network_type='walk')
#get_nearest_node
start_location = (40.43, -3.7)
end_location = (40.43,-3.68)
start_node = ox.get_nearest_node(G, start_location)
end_node = ox.get_nearest_node(G, end_location)
# count time
start_time = datetime.datetime.now()
path = temp_dijkstra(G, start_node, end_node)
end_time = datetime.datetime.now()
delta = end_time - start_time
# to milliseconds
delta = round(delta.total_seconds(), 4)
print(path)
print('Time: ',delta, ' s')
today = datetime.date.today()
today_str = today.strftime("%m%d")
file_name = 'test-' + str(start_node) + '-' + str(end_node) + '-' + today_str + '_osmnx'
print(file_name)
# save_path(G, path, 'output/' + file_name)


