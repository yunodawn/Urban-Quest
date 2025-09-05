#include "m1.h"
#include "m2.h"
#include "m3.h"

#include "StreetsDatabaseAPI.h"
#include "OSMDatabaseAPI.h"

#include <queue>
#include <unordered_map>
#include <limits>
#include <vector>
#include <algorithm>
#include <cmath>

struct Node {
    IntersectionIdx intersection;
    double g_cost; //cost so far
    double f_cost; //g_cost + heuristic
    StreetSegmentIdx segment; //street segment used to reach this node
    
    bool operator>(const Node& other) const {
        return f_cost > other.f_cost;
    }
};  

double heuristic(IntersectionIdx current, IntersectionIdx goal);
double heuristic(IntersectionIdx current, IntersectionIdx goal) {
    //retrieve positions of the intersections
    LatLon curPos = getIntersectionPosition(current);
    LatLon goalPos = getIntersectionPosition(goal);
    double dist = findDistanceBetweenTwoPoints(curPos, goalPos);
    
    //assume MAX_SPEED is the fastest possible speed (m/s) on any segment (100kph)
    const double MAX_SPEED = 100/3.6;
    return dist / MAX_SPEED;
}

//helper function to compute the total travel time of a given path
double computePathTravelTime(const double turn_penalty, const std::vector<StreetSegmentIdx>& path) {
    if (path.empty()) return 0.0;
    
    double total_time = 0.0;
    for (size_t i = 0; i < path.size(); ++i) {
        total_time += findStreetSegmentTravelTime(path[i]);
        
        //apply turn penalty if the current segment is on a different street than the previous one
        if (i > 0) {
            StreetIdx prev_street = getStreetSegmentInfo(path[i - 1]).streetID;
            StreetIdx curr_street = getStreetSegmentInfo(path[i]).streetID;
            if (prev_street != curr_street) {
                total_time += turn_penalty;
            }
        }
    }
    return total_time;
}

//A* algorithm for finding the path between two intersections
std::vector<StreetSegmentIdx> findPathBetweenIntersections(
    const double turn_penalty,
    const std::pair<IntersectionIdx, IntersectionIdx> intersect_ids) {
    
    IntersectionIdx start = intersect_ids.first;
    IntersectionIdx goal = intersect_ids.second;
    
    if (start == goal)
        return {}; //no travel needed
    
    //priority queue (min-heap) where the lowest f_cost is at the top
    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open_set;
    
    //stores the best (lowest) g_cost (travel time so far) to reach each intersection
    std::unordered_map<IntersectionIdx, double> best_time;
    //these maps help reconstruct the path
    std::unordered_map<IntersectionIdx, StreetSegmentIdx> came_from;
    std::unordered_map<IntersectionIdx, IntersectionIdx> parent;
    
    double h = heuristic(start, goal);
    open_set.push({start, 0.0, h, -1});
    best_time[start] = 0.0;
    
    while (!open_set.empty()) {
        Node current = open_set.top();
        open_set.pop();
        
        if (current.intersection == goal)
            break;
        
        IntersectionIdx cur_intersection = current.intersection;
        
        //expand neighbors (adjacent street segments)
        for (StreetSegmentIdx seg : findStreetSegmentsOfIntersection(cur_intersection)) {
            StreetSegmentInfo seg_info = getStreetSegmentInfo(seg);
            IntersectionIdx next_intersection = (seg_info.from == cur_intersection) ? seg_info.to : seg_info.from;
            
            //respect one-way constraints
            if (seg_info.oneWay && seg_info.from != cur_intersection)
                continue;
            
            //compute new g_cost (travel time so far)
            double new_g = current.g_cost + findStreetSegmentTravelTime(seg);
            if (current.segment != -1 && getStreetSegmentInfo(current.segment).streetID != seg_info.streetID) {
                new_g += turn_penalty;
            }
            
            //relaxation: if this path is better, update and add neighbor to the open set
            if (best_time.find(next_intersection) == best_time.end() || new_g < best_time[next_intersection]) {
                best_time[next_intersection] = new_g;
                double new_f = new_g + heuristic(next_intersection, goal);
                open_set.push({next_intersection, new_g, new_f, seg});
                came_from[next_intersection] = seg;
                parent[next_intersection] = cur_intersection;
            }
        }
    }
    
    //if the goal was never reached, return an empty path
    if (came_from.find(goal) == came_from.end())
        return {};
    
    //reconstruct the path by backtracking from the goal
    std::vector<StreetSegmentIdx> path;
    for (IntersectionIdx at = goal; at != start; at = parent[at]) {
        path.push_back(came_from[at]);
    }
    std::reverse(path.begin(), path.end());
    return path;
}
