# -*- coding: utf-8 -*-
"""
Created on Thu Jan 27 23:20:17 2022

@author: 1026313
"""
import osmnx as ox
import networkx as nx
import pandas as pd
import numpy as np
import logging
import time

ox.config(log_console=True, use_cache=True)
pd.options.mode.chained_assignment = None

distance_cache = {}
no_paths = set()

class route(object):
    def __init__(self, G, source, destination):
        self.source = source
        self.destination = destination
        self.G = G
        self.origin_node = ox.get_nearest_node(self.G, self.source)
        self.destination_node = ox.get_nearest_node(self.G, self.destination)
        
    def haversine(self, a, b):
        r = 6371
        self.s_lat = self.G.nodes[a]['y']
        self.d_lat = self.G.nodes[b]['y']
        self.s_lon = self.G.nodes[a]['x']
        self.d_lon = self.G.nodes[b]['x']
        
        self.phi1 = np.radians(self.s_lat)
        self.phi2 = np.radians(self.d_lat)
        self.delta_phi = np.radians(self.d_lat - self.s_lat)
        self.delta_lambda = np.radians(self.d_lon - self.s_lon)
        
        self.a = np.sin(self.delta_phi / 2)**2 + np.cos(self.phi1) * np.cos(self.phi2) * np.sin(self.delta_lambda / 2)**2
        self.hav_dist = r * (2 * np.arctan2(np.sqrt(self.a), np.sqrt(1 - self.a)))
        return self.hav_dist

    def euclidean(self, a, b):
        self.s_lat = self.G.nodes[a]['y']
        self.d_lat = self.G.nodes[b]['y']
        self.s_lon = self.G.nodes[a]['x']
        self.d_lon = self.G.nodes[b]['x']
        
        self.eu_dist = np.sqrt((self.d_lat - self.s_lat)**2 \
                            + (self.d_lon - self.s_lon)**2)
        return self.eu_dist

    def calculate_travel_time(self):
        """Calculate shortest path by travel time 
        Search algorithm: A*
        Heuristic option: Euclidean distance (default) / Haversine distance
        
        Haversine is the more accurate approximation for distances given
        GPS and the error between Euclidean-Haversine increases with distance 
        between O-D, but I've chosen Euclidean as the default because expected 
        distances are short"""
        
        if (self.origin_node,self.destination_node) in distance_cache.keys():
            self.hours = distance_cache[(self.origin_node,self.destination_node)]//3600
            if self.hours >= 8:
                #compensating for driver rest time
                distance_cache[(self.origin_node,self.destination_node)] *= 1.375
                print(f"Estimated travel time: {time.strftime('%H:%M:%S', time.gmtime(distance_cache[(self.origin_node,self.destination_node)]))}")
            return distance_cache[(self.origin_node,self.destination_node)]
        else:
            try:
                self.travel_time = nx.astar_path_length(self.G, self.origin_node, \
                                                        self.destination_node, \
                                                            heuristic=self.euclidean, \
                                                                weight='travel_time')
                self.hours = self.travel_time//3600
                if self.hours >= 8:
                    #compensating for driver rest time
                    self.travel_time *= 1.375
                print(f"Estimated travel time: {time.strftime('%H:%M:%S', time.gmtime(self.travel_time))}")
                distance_cache[(self.origin_node,self.destination_node)] = self.travel_time
                return self.travel_time
            except nx.NetworkXNoPath:
                no_paths.add((self.origin_node,self.destination_node))
                logging.warning(f"There is no path between the two nodes: {self.origin_node},{self.destination_node}. Finding Haversine distance instead!")
                self.distance = self.haversine(self.origin_node, self.destination_node)
                self.hours = (self.distance/65)
                self.travel_time = (self.distance/65)*3600
                if self.hours >= 8:
                    #compensating for driver rest time
                    self.travel_time *= 1.375
                print(f"Estimated travel time: {time.strftime('%H:%M:%S', time.gmtime(self.travel_time))}")
                distance_cache[(self.origin_node,self.destination_node)] = self.travel_time
                return self.travel_time
        
def travel_wrapper(G, source, destination):
    route_instance = route(G, source, destination)
    travel_time = route_instance.calculate_travel_time()
    return travel_time

start_time = time.time()
ship_leg_df = pd.read_csv("sample.csv")
graphml_map = {'KOR':'south_korea_highways.graphml', \
                                'CN':'china_highways.graphml', \
                                    'JP': 'japan_highways.graphml', \
                                       'KR':'south_korea_highways.graphml'}
country_list = list(set(ship_leg_df['dest_country'].values))
country_df_list =[]
print("Estimated time calculated at ave. speed = free flow speed - 20kmph")
print("Driver rest time factored at 3h of rest every 8h of driving")

for country in country_list:
    network_file = graphml_map[country]
    G = ox.load_graphml(network_file)
    ship_leg_df_filtered = ship_leg_df.loc[ship_leg_df.dest_country == country]
    ship_leg_df_filtered['travel_time_secs'] = ship_leg_df_filtered.apply(lambda row: travel_wrapper(G, (row['source_lat'],row['source_long']), (row['dest_lat'],row['dest_long'])), axis=1)
    country_df_list.append(ship_leg_df_filtered)
   
leg_df_final = pd.concat(country_df_list, ignore_index=True)
print(f"Query time: {round(time.time() - start_time, 2)} seconds")
leg_df_final.to_csv('leg_travel_time.csv', index=False)