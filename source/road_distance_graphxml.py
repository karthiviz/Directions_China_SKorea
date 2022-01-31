# -*- coding: utf-8 -*-
"""
Created on Thu Jan 27 23:20:17 2022

@author: 1026313
"""
import osmnx as ox
import networkx as nx
import numpy as np
import requests
import time
import logging

ox.config(log_console=True, use_cache=True)

def connect_to_endpoint_post(url):  
    headers = {'Content-Type': 'application/json'}
    response = requests.request("GET", url, headers=headers)
    return response.json()  
    
def get_country(origin_point, destination_point):
    o_query = f"https://atlas.microsoft.com/search/address/reverse/json?subscription-key=key&api-version=1.0&query={origin_point[0]}, {origin_point[1]}"
    d_query = f"https://atlas.microsoft.com/search/address/reverse/json?subscription-key=key&api-version=1.0&query={destination_point[0]}, {destination_point[1]}"
    o_response = connect_to_endpoint_post(o_query)
    d_response = connect_to_endpoint_post(d_query)
    o_country = o_response['addresses'][0]['address']['countryCode']
    d_country = d_response['addresses'][0]['address']['countryCode']
    if o_country == 'CN' and d_country == 'CN':
        return 'CN'
    elif o_country == 'KR' and d_country == 'KR':
        return 'KR'
    elif o_country == 'JP' and d_country == 'JP':
        return 'JP'
    else:
        print("Cross-country journey or countries other than China/South Korea\
              /Japan isn't supported currently, try Azure Maps Route API!")
        return
    
class route(object):
    def __init__(self, source, destination, func=get_country):
        self.source = source
        self.destination = destination
        self.graphml_map = {'KR':'south_korea_highways.graphml', \
                                'CN':'china_highways.graphml', \
                                    'JP': 'japan_highways_gml.graphml'}
        self.network_file = self.graphml_map[func(source, destination)]
        self.G = ox.load_graphml(self.network_file)
        self.origin_node = ox.get_nearest_node(self.G, self.source)
        self.destination_node = ox.get_nearest_node(self.G, self.destination)

    def haversine(self, a, b):
        r = 6371
        s_lat = self.G.nodes[a]['y']
        d_lat = self.G.nodes[b]['y']
        s_lon = self.G.nodes[a]['x']
        d_lon = self.G.nodes[b]['x']
        
        phi1 = np.radians(s_lat)
        phi2 = np.radians(d_lat)
        delta_phi = np.radians(d_lat - s_lat)
        delta_lambda = np.radians(d_lon - s_lon)
        
        a = np.sin(delta_phi / 2)**2 + np.cos(phi1) * np.cos(phi2) * np.sin(delta_lambda / 2)**2
        self.hav_dist = r * (2 * np.arctan2(np.sqrt(a), np.sqrt(1 - a)))
        # setting heuristic cost to approximate cost to goal node
        return 5 * np.sqrt(self.hav_dist)
    
    def euclidean(self, a, b):
        s_lat = self.G.nodes[a]['y']
        d_lat = self.G.nodes[b]['y']
        s_lon = self.G.nodes[a]['x']
        d_lon = self.G.nodes[b]['x']
        
        self.eu_dist = np.sqrt((d_lat - s_lat)**2 \
                            + (d_lon - s_lon)**2)
        return self.eu_dist
    
    def landmark(self, a, b):
        raise NotImplementedError

    def calculate_travel_time(self):
        """Calculate shortest path by travel time 
        Search algorithm: A* (default) / Bi-directional Dijkstra
        Heuristic option (A*): Haversine (default) / Euclidean /Landmark
        """
        
        try:
            self.travel_time = nx.astar_path_length(self.G, self.origin_node, \
                                                    self.destination_node, \
                                                        heuristic=self.haversine, \
                                                            weight='travel_time')
            
            # self.travel_time = nx.bidirectional_dijkstra(self.G, self.origin_node, \
            #                                         self.destination_node, \
            #                                             weight='travel_time')[0]
                
            hours = self.travel_time//3600
            if hours >= 8:
                #compensating for driver rest time
                self.travel_time *= 1.375
            print("Estimated time calculated at ave. speed = free flow speed - 20kmph")
            print("Driver rest time factored at 3h of rest every 8h of driving")
            print(f"Estimated travel time: {time.strftime('%H:%M:%S', time.gmtime(self.travel_time))}")
            return self.travel_time
        except nx.NetworkXNoPath:
            logging.warning(f"There is no path between the two nodes: {self.origin_node},{self.destination_node}. Finding Haversine distance instead!")
            distance = self.haversine(self.origin_node, self.destination_node)
            hours = (distance/65)
            self.travel_time = hours*3600
            if hours >= 8:
                #compensating for driver rest time
                self.travel_time *= 1.375
            print(f"Estimated travel time: {time.strftime('%H:%M:%S', time.gmtime(self.travel_time))}")
            return self.travel_time
        
if __name__ == "__main__":
    start_time = time.time()
    origin_point = (31.37859344, 120.656456)
    destination_point =(31.14340019, 121.8050003)
    route = route(origin_point, destination_point)
    route.calculate_travel_time()
    print(f"Pathfinding time: {time.strftime('%H:%M:%S', time.gmtime(round(time.time() - start_time, 2)))}")