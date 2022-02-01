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
import json

ox.config(log_console=True, use_cache=True)
pd.options.mode.chained_assignment = None

distance_cache = {}
no_paths = set()

class route(object):
    def __init__(self, G, L ,source, destination, **kwargs):
        self.source = source
        self.destination = destination
        self.G = G
        self.origin_node = ox.get_nearest_node(self.G, self.source)
        self.destination_node = ox.get_nearest_node(self.G, self.destination)
        self.landmarks_from = L
        self.algorithm = kwargs["algorithm"]
    
    def catch(self, dictionary, *args, handle=lambda e : 0):
        try:
            return {(args[0],args[1]):dictionary[args[0]][args[1]]}
        except Exception as e:
            return handle(e)
    
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
        origin_from_landmarks = [self.catch(self.landmarks_from[0],landmark,a) for landmark in self.landmarks_from[0].keys()]
        destination_from_landmarks = [self.catch(self.landmarks_from[0],landmark,b) for landmark in self.landmarks_from[0].keys()]
        try:
            best_landmark = max(origin_from_landmarks[0], key=origin_from_landmarks[0].get)[0]
            self.origin_to_destination = {(a,b): origin_from_landmarks[0][(best_landmark,a)] - destination_from_landmarks[0][(best_landmark,b)]}
            return self.origin_to_destination[(a,b)]
        except Exception:
            return self.haversine(a, b)

    def calculate_travel_time(self, **kwargs):
        """Calculate shortest path by travel time 
        Search algorithm: Bi-directional Dijkstra (default) / A*
        Heuristic options (A*): Haversine (default)/Landmark/Euclidean
        """
        
        if (self.origin_node,self.destination_node) in distance_cache.keys():
            hours = distance_cache[(self.origin_node,self.destination_node)]//3600
            if hours >= 8:
                #compensating for driver rest time
                distance_cache[(self.origin_node,self.destination_node)] *= 1.375
            print(f"Estimated travel time: {time.strftime('%H:%M:%S', time.gmtime(distance_cache[(self.origin_node,self.destination_node)]))}")
            return distance_cache[(self.origin_node,self.destination_node)]
        else:
            try:
                if (self.algorithm == "astar"):
                    self.travel_time = nx.astar_path_length(self.G, self.origin_node, \
                                                            self.destination_node, \
                                                                heuristic=self.haversine, \
                                                                    weight='travel_time')
                
                elif (self.algorithm == "dijkstra"):
                    self.travel_time = nx.bidirectional_dijkstra(self.G, self.origin_node, \
                                                            self.destination_node, \
                                                                weight='travel_time')[0]    
                    
                hours = self.travel_time//3600
                if hours >= 8:
                    #compensating for driver rest time
                    self.travel_time *= 1.375
                print(f"Estimated travel time: {time.strftime('%H:%M:%S', time.gmtime(self.travel_time))}")
                distance_cache[(self.origin_node,self.destination_node)] = self.travel_time
                return self.travel_time
            except nx.NetworkXNoPath:
                no_paths.add((self.origin_node,self.destination_node))
                logging.warning(f"There is no path between the two nodes: {self.origin_node},{self.destination_node}. Finding Haversine distance instead!")
                distance = self.haversine(self.origin_node, self.destination_node)
                hours = (distance/65)
                self.travel_time = hours*3600
                if hours >= 8:
                    #compensating for driver rest time
                    self.travel_time *= 1.375
                print(f"Estimated travel time: {time.strftime('%H:%M:%S', time.gmtime(self.travel_time))}")
                distance_cache[(self.origin_node,self.destination_node)] = self.travel_time
                return self.travel_time
        
def travel_wrapper(G, L, source, destination):
    route_instance = route(G, L, source, destination, algorithm="dijkstra")
    travel_time = route_instance.calculate_travel_time()
    return travel_time

def keystoint(x):
        return {int(k): v for k, v in x.items()}

if __name__ == "__main__":
    ship_leg_df = pd.read_csv("sample.csv")
    graphml_map = {'KOR':'south_korea_highways_all.graphml', \
                                    'CN':'china_highways.graphml', \
                                        'JP': 'japan_highways_all.graphml', \
                                           'KR':'south_korea_highways_all.graphml'}
    landmarks_from_map = {'KOR':'landmark_files/south_korea_landmarks_from.json', \
                                    'CN':'landmark_files/china_landmarks_from.json', \
                                        'JP': 'landmark_files/japan_landmarks_from.json', \
                                           'KR':'landmark_files/south_korea_landmarks_from.json'}
    country_list = list(set(ship_leg_df['dest_country'].values))
    country_df_list =[]
    print("Estimated time calculated at ave. speed = free flow speed - 20kmph")
    print("Driver rest time factored at 3h of rest every 8h of driving")
    
    pathfinding_time = 0
    for country in country_list:
        network_file = graphml_map[country]
        landmark_file = landmarks_from_map[country]
        G = ox.load_graphml(network_file)
        f = open(landmark_file)
        landmarks_from = json.load(f, object_hook=keystoint)
        f.close()
        ship_leg_df_filtered = ship_leg_df.loc[ship_leg_df.dest_country == country]
        start_time = time.time()
        ship_leg_df_filtered['travel_time_secs'] = ship_leg_df_filtered.apply(lambda row: travel_wrapper(G, landmarks_from, (row['source_lat'],row['source_long']), (row['dest_lat'],row['dest_long'])), axis=1)
        pathfinding_time += round(time.time() - start_time, 2)
        country_df_list.append(ship_leg_df_filtered)
       
    print(f"Pathfinding time: {time.strftime('%H:%M:%S', time.gmtime(pathfinding_time))}")
    leg_df_final = pd.concat(country_df_list, ignore_index=True)
    leg_df_final.to_csv('leg_travel_time_astar.csv', index=False)