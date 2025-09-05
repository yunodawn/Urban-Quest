#include "StreetsDatabaseAPI.h"
#include <thread>
#include <mutex>
#include <future>
#include <atomic>
#include <functional>
#include <unordered_set>
#include <queue>
#include <chrono>
#include <omp.h>
#include <cstdlib>
#include <ctime>
#include <vector>
#include <unordered_map>
#include <set>
#include <limits>
#include <cmath>
#include "m4.h"
#include "m3.h"
#include "m1.h"

//helper delcarations
double estimateTravelTime(IntersectionIdx from, IntersectionIdx to);
IntersectionIdx findClosestDepotToPickup(
    const std::vector<IntersectionIdx>& depots,
    const std::vector<DeliveryInf>& deliveries,
    float turn_penalty);

IntersectionIdx findClosestDepot(
    IntersectionIdx from,
    const std::vector<IntersectionIdx>& depots,
    float turn_penalty);

IntersectionIdx findClosestDepot(
    IntersectionIdx from,
    const std::vector<IntersectionIdx>& depots,
    const std::unordered_map<IntersectionIdx, std::unordered_map<IntersectionIdx, double>>& distance_matrix); //dont delete this its an overload

double computeTotalPathTime(const std::vector<CourierSubPath>& path, float turn_penalty);
std::vector<CourierSubPath> runGreedyCourier(
    IntersectionIdx start_depot,
    const float turn_penalty,
    const std::vector<DeliveryInf>& deliveries,
    const std::vector<IntersectionIdx>& depots);

std::vector<CourierSubPath> runGreedyLookaheadCourier(
    IntersectionIdx start_depot,
    const float turn_penalty,
    const std::vector<DeliveryInf>& deliveries,
    const std::vector<IntersectionIdx>& depots,
    int lookahead_depth);

void findBestSequence(
    IntersectionIdx current_location,
    std::set<int> picked_up,
    std::set<int> delivered,
    std::vector<bool> is_picked,
    std::vector<bool> is_delivered,
    const std::unordered_map<IntersectionIdx, std::unordered_map<IntersectionIdx, double>>& distance_matrix,
    const std::unordered_map<IntersectionIdx, std::vector<int>>& pickup_map,
    const std::unordered_map<IntersectionIdx, std::vector<int>>& dropoff_map,
    int total_deliveries,
    int current_depth,
    int max_depth,
    double current_cost,
    std::vector<IntersectionIdx> current_sequence,
    double& best_sequence_cost,
    std::vector<IntersectionIdx>& best_sequence);

std::unordered_map<IntersectionIdx, std::unordered_map<IntersectionIdx, double>>
computeAllLineDistances(const std::vector<IntersectionIdx>& sources, 
                                const std::vector<IntersectionIdx>& destinations);

//////////////////////////////////////////////////////////////////////////////////////

std::unordered_map<IntersectionIdx, std::unordered_map<IntersectionIdx, double>>
computeAllLineDistances(const std::vector<IntersectionIdx>& sources, 
                                const std::vector<IntersectionIdx>& destinations){
    
    // distance matrix - stores straight-line distances instead of paths
    std::unordered_map<IntersectionIdx, std::unordered_map<IntersectionIdx, double>> all_distances;
    
    // for each source compute straight-line distances to all destinations
    for (IntersectionIdx source : sources) {
        std::unordered_map<IntersectionIdx, double> distances_from_source;
        
        //get coords 
        LatLon source_coords = getIntersectionPosition(source);
        
        for (IntersectionIdx dest : destinations) {
            if (source == dest) {
                distances_from_source[dest] = 0.0;
                continue;
            }

            LatLon dest_coords = getIntersectionPosition(dest);

            double distance = findDistanceBetweenTwoPoints(source_coords, dest_coords);

            distances_from_source[dest] = distance;
        }

        all_distances[source] = distances_from_source;
    }
    
    return all_distances;
}
std::vector<IntersectionIdx> apply2OptSwap(const std::vector<IntersectionIdx>& path, size_t i, size_t k);

bool isLegalPath(const std::vector<IntersectionIdx>& path,
                 const std::vector<DeliveryInf>& deliveries);

//main function
std::vector<CourierSubPath> travelingCourier(
    const float turn_penalty,
    const std::vector<DeliveryInf>& deliveries,
    const std::vector<IntersectionIdx>& depots)
{
    //ensure the test cases dont time out
    double time_limit_sec = 49.0;
    auto start_time = std::chrono::high_resolution_clock::now();
    auto now = std::chrono::high_resolution_clock::now();
    double elapsed = std::chrono::duration<double>(now - start_time).count();

    std::vector<CourierSubPath> best_path;
    double best_time = std::numeric_limits<double>::max();

    // local thread storing
    std::vector<std::vector<CourierSubPath>> thread_path_storing(depots.size());
    std::vector<double> thread_time_storing(depots.size(), std::numeric_limits<double>::max());
    
//    #pragma omp parallel for
//    for (int i = 0; i < depots.size(); i++){
//    //for (IntersectionIdx depot : depots) {
//        auto depot = depots[i];
//    }
    std::vector<IntersectionIdx> all_intersections;
    for (const auto& depot : depots){
        all_intersections.push_back(depot);
    }
    
    for (const auto& delivery : deliveries){
        all_intersections.push_back(delivery.pickUp);
        all_intersections.push_back(delivery.dropOff);
    }
    
    //get distance matrix 
    std::unordered_map<IntersectionIdx, std::unordered_map<IntersectionIdx, double>> 
        distance_matrix = computeAllLineDistances(all_intersections, all_intersections);

    //heuristic: multistart
    #pragma omp parallel for
    for (int i = 0; i < depots.size(); i++){
    //for (IntersectionIdx depot : depots) {
        auto depot = depots[i];
        auto path = runGreedyCourier(depot, turn_penalty, deliveries, depots);
        if (!path.empty()) {
            double travel_time = computeTotalPathTime(path, turn_penalty);
//           if (travel_time < best_time) {
//               best_time = travel_time;
//               best_path = path;
//           }
            thread_path_storing[i] = path;
            thread_time_storing[i] = travel_time;
        }
    }

    for (int i = 0; i < depots.size(); ++i) {
        if (thread_time_storing[i] < best_time) {
            best_time = thread_time_storing[i];
            best_path = thread_path_storing[i];
        }
    }

    if (best_path.empty()) return best_path;

    // Step 1: Convert to intersection path
    std::vector<IntersectionIdx> node_path;
    node_path.push_back(best_path.front().intersections.first);
    for (const auto& sub : best_path) {
        node_path.push_back(sub.intersections.second);
    }

    // Step 2: Cache all subpaths between intersections
    std::unordered_map<IntersectionIdx, std::unordered_map<IntersectionIdx, std::vector<StreetSegmentIdx>>> all_paths;
    for (size_t i = 0; i < node_path.size() - 1; ++i) {
        all_paths[node_path[i]][node_path[i + 1]] = findPathBetweenIntersections(turn_penalty, {node_path[i], node_path[i + 1]});
    }

    double best_cost = computeTotalPathTime(best_path, turn_penalty);

    // Step 3: 2-opt improvement loop
    bool improvement = true;
    
    while (improvement) {
        
        //std::cout << "[INFO] Running 2opt...\n";
        improvement = false;

        now = std::chrono::high_resolution_clock::now();
        elapsed = std::chrono::duration<double>(now - start_time).count();
        if (elapsed > time_limit_sec) break;

    #pragma omp parallel for
        for (size_t i = 1; i < node_path.size() - 2; ++i) {
            for (size_t k = i + 1; k < node_path.size() - 1; ++k) {

                now = std::chrono::high_resolution_clock::now();
                elapsed = std::chrono::duration<double>(now - start_time).count();
                if (elapsed > time_limit_sec) continue;

                auto new_path = apply2OptSwap(node_path, i, k);
                if (!isLegalPath(new_path, deliveries)) continue;

                double new_cost = 0.0;
                bool valid = true;
                for (size_t j = 0; j < new_path.size() - 1; ++j) {
                    IntersectionIdx a = new_path[j];
                    IntersectionIdx b = new_path[j + 1];

                    if (all_paths[a][b].empty()) {
#pragma omp critical
                        if (all_paths[a][b].empty()) {
                        all_paths[a][b] = findPathBetweenIntersections(turn_penalty, {a, b});
                        }
                    }

                    const auto& segment = all_paths[a][b];
                    if (segment.empty()) {
                        valid = false;
                        continue;
                    }

                    new_cost += computePathTravelTime(turn_penalty, segment);
                }
                
                if (valid && new_cost < best_cost) {
#pragma omp critical
                    if (valid && new_cost < best_cost) {
                    //std::cout << "2Opt QoR improved: " << best_cost << " ? " << new_cost << "\n";
                    node_path = new_path;
                    best_cost = new_cost;
                    improvement = true;
                                    }
                }
            }
            if (improvement) continue;
        }
    }

    //Step 4: Simulated Annealing
    srand(time(NULL));
    double T = 1000.0;
    double alpha = 0.9999;

    std::vector<IntersectionIdx> current_path = node_path;
    double current_cost = best_cost;

    while (true) {
        now = std::chrono::high_resolution_clock::now();
        elapsed = std::chrono::duration<double>(now - start_time).count();
        if (elapsed > time_limit_sec || T < 1e-9) break;

        if (current_path.size() < 4) break;
        size_t max_index = current_path.size() - 2;
        if (max_index <= 2) {
            break;
        }

        size_t i = 1 + (rand() % (max_index - 1));
        size_t k = i + 1 + (rand() % (max_index - i));
        if (k < i) std::swap(i, k);

        auto new_path = apply2OptSwap(current_path, i, k);
        if (!isLegalPath(new_path, deliveries)) {
            continue;
        }

        double new_cost = 0.0;
        bool valid = true;
        for (size_t j = 0; j < new_path.size() - 1; ++j) {
            IntersectionIdx a = new_path[j];
            IntersectionIdx b = new_path[j + 1];

            // If path isn't already cached, compute and cache it
            if (all_paths[a][b].empty()) {
                all_paths[a][b] = findPathBetweenIntersections(turn_penalty, {a, b});
            }

            const auto& seg = all_paths[a][b];

            if (seg.empty()) {
                valid = false;
                break;
            }
            new_cost += computePathTravelTime(turn_penalty, seg);
        }

        if (!valid) {
            continue;
        }
        double delta = new_cost - current_cost;
        if (delta < 0) {
            current_path = new_path;
            current_cost = new_cost;
            if (new_cost < best_cost) {
                best_cost = new_cost;
                node_path = new_path;
            }
        } else {
            double accept_prob = std::exp(-delta / T);
            double rand_val = static_cast<double>(rand()) / RAND_MAX;
            if (rand_val < accept_prob) {
                current_path = new_path;
                current_cost = new_cost;
            }
        }

        T *= alpha;
    }

    // Step 5: Convert back to CourierSubPath
    std::vector<CourierSubPath> improved_path;
    for (size_t i = 0; i < node_path.size() - 1; ++i) {
        CourierSubPath sub;
        sub.intersections = {node_path[i], node_path[i + 1]};
        sub.subpath = all_paths[node_path[i]][node_path[i + 1]];
        improved_path.push_back(sub);
    }

    return improved_path;
}


//greedy route from 1 depot
// std::vector<CourierSubPath> runGreedyCourier(
//     IntersectionIdx start_depot,
//     const float turn_penalty,
//     const std::vector<DeliveryInf>& deliveries,
//     const std::vector<IntersectionIdx>& depots)
// {
//     std::set<int> picked_up, delivered;
//     std::vector<bool> is_picked(deliveries.size(), false), is_delivered(deliveries.size(), false);
//     std::vector<CourierSubPath> final_path;

//     std::unordered_map<IntersectionIdx, std::vector<int>> pickup_map, dropoff_map;
//     for (int i = 0; i < deliveries.size(); i++) {
//         pickup_map[deliveries[i].pickUp].push_back(i);
//         dropoff_map[deliveries[i].dropOff].push_back(i);
//     }

//     IntersectionIdx current = start_depot;

//     while (picked_up.size() < deliveries.size() || delivered.size() < deliveries.size()) {
//         IntersectionIdx next = -1;
//         double best_cost = std::numeric_limits<double>::max();
//         bool is_next_dropoff = false;

//         for (const auto& [pickup, idxs] : pickup_map) {
//             if (!is_picked[idxs[0]]) {
//                 double est_cost = estimateTravelTime(current, pickup);
//                 if (est_cost < best_cost) {
//                     best_cost = est_cost;
//                     next = pickup;
//                     is_next_dropoff = false;
//                 }
//             }
//         }

//         for (const auto& [dropoff, idxs] : dropoff_map) {
//             for (int i : idxs) {
//                 if (is_picked[i] && !is_delivered[i]) {
//                     double est_cost = estimateTravelTime(current, dropoff);
//                     if (est_cost < best_cost) {
//                         best_cost = est_cost;
//                         next = dropoff;
//                         is_next_dropoff = true;
//                     }
//                 }
//             }
//         }

//         if (next == -1) return {}; // Failed

//         auto path = findPathBetweenIntersections(turn_penalty, {current, next});
//         if (path.empty()) return {};

//         CourierSubPath sub;
//         sub.intersections = {current, next};
//         sub.subpath = path;

//         if (!is_next_dropoff) {
//             for (int i : pickup_map[next]) {
//                 is_picked[i] = true;
//                 picked_up.insert(i);
//             }
//         } else {
//             for (int i : dropoff_map[next]) {
//                 if (is_picked[i]) {
//                     is_delivered[i] = true;
//                     delivered.insert(i);
//                 }
//             }
//         }

//         final_path.push_back(sub);
//         current = next;
//     }

//     IntersectionIdx best_end_depot = findClosestDepot(current, depots, turn_penalty);
//     auto end_path = findPathBetweenIntersections(turn_penalty, {current, best_end_depot});
//     if (!end_path.empty()) {
//         CourierSubPath final_leg;
//         final_leg.intersections = {current, best_end_depot};
//         final_leg.subpath = end_path;
//         final_path.push_back(final_leg);
//     }

//     return final_path;
// }

std::vector<CourierSubPath> runGreedyLookaheadCourier(
    IntersectionIdx start_depot,
    const float turn_penalty,
    const std::vector<DeliveryInf>& deliveries,
    const std::vector<IntersectionIdx>& depots,
    int lookahead_depth)  
{
    //intersections for compute line distance call 
    std::vector<IntersectionIdx> all_intersections;
    all_intersections.push_back(start_depot);
    
    for (const auto& delivery : deliveries){
        all_intersections.push_back(delivery.pickUp);
        all_intersections.push_back(delivery.dropOff);
    }
    
    for (const auto& depot : depots){
        all_intersections.push_back(depot);
    }

    std::unordered_map<IntersectionIdx, std::unordered_map<IntersectionIdx, double>> 
        distance_matrix = computeAllLineDistances(all_intersections, all_intersections);
    
    std::vector<bool> is_picked(deliveries.size(), false), is_delivered(deliveries.size(), false);
    std::vector<CourierSubPath> final_path;

    std::unordered_map<IntersectionIdx, std::vector<int>> pickup_map, dropoff_map;
    for (int i = 0; i < deliveries.size(); i++){
        pickup_map[deliveries[i].pickUp].push_back(i);
        dropoff_map[deliveries[i].dropOff].push_back(i);
    }

    IntersectionIdx current = start_depot;
    std::set<int> picked_up, delivered;

    //routing until all packages are delivered
    while (delivered.size() < deliveries.size()){
        //find sequence of n intersections to visit 
        std::vector<IntersectionIdx> best_sequence;
        double best_sequence_cost = std::numeric_limits<double>::infinity();
        
        //current state for simulating sequences
        std::set<int> sim_picked = picked_up;
        std::set<int> sim_delivered = delivered;
        std::vector<bool> sim_is_picked = is_picked;
        std::vector<bool> sim_is_delivered = is_delivered;
        
        //find the best sequence using recursive exploration
        findBestSequence(
            current,
            sim_picked,
            sim_delivered,
            sim_is_picked,
            sim_is_delivered,
            distance_matrix,
            pickup_map,
            dropoff_map,
            deliveries.size(),
            0,  // current depth
            lookahead_depth,
            0.0,  // current sequence cost
            {},  // current sequence
            best_sequence_cost,
            best_sequence
        );
        
        if (best_sequence.empty()) {
            //find the best depot to end at
            if (delivered.size() == deliveries.size()) {
                IntersectionIdx best_end_depot = findClosestDepot(current, depots, distance_matrix);
                auto end_path = findPathBetweenIntersections(turn_penalty, {current, best_end_depot});
                if (!end_path.empty()) {
                    CourierSubPath final_leg;
                    final_leg.intersections = {current, best_end_depot};
                    final_leg.subpath = end_path;
                    final_path.push_back(final_leg);
                }
                break;
            } 
            
            else{
                return {};
            }
        }

        IntersectionIdx next = best_sequence[0];
        
        //calculate actual path to the next location
        auto path = findPathBetweenIntersections(turn_penalty, {current, next});
        if (path.empty()) return {};
        
        CourierSubPath sub;
        sub.intersections = {current, next};
        sub.subpath = path;
        final_path.push_back(sub);

        //bool is_pickup_node = false;
        
        //this is a pickup location
        if (pickup_map.find(next) != pickup_map.end()) {
            for (int i : pickup_map[next]) {
                if (!is_picked[i]) {
                    is_picked[i] = true;
                    picked_up.insert(i);
                    //is_pickup_node = true;
                }
            }
        }
        
        //this is a dropoff location
        if (dropoff_map.find(next) != dropoff_map.end()) {
            for (int i : dropoff_map[next]) {
                if (is_picked[i] && !is_delivered[i]) {
                    is_delivered[i] = true;
                    delivered.insert(i);
                }
            }
        }

        current = next;
    }

    IntersectionIdx best_end_depot = findClosestDepot(current, depots, distance_matrix);
    auto end_path = findPathBetweenIntersections(turn_penalty, {current, best_end_depot});
    if (!end_path.empty()) {
        CourierSubPath final_leg;
        final_leg.intersections = {current, best_end_depot};
        final_leg.subpath = end_path;
        final_path.push_back(final_leg);
    }

    return final_path;
}

std::vector<CourierSubPath> runGreedyCourier( //old fn 
    IntersectionIdx start_depot,
    const float turn_penalty,
    const std::vector<DeliveryInf>& deliveries,
    const std::vector<IntersectionIdx>& depots)
{
    //collect intersections for compute line distance call 
    std::vector<IntersectionIdx> all_intersections;
    all_intersections.push_back(start_depot);
    
    for (const auto& delivery : deliveries){
        all_intersections.push_back(delivery.pickUp);
        all_intersections.push_back(delivery.dropOff);
    }
    
    for (const auto& depot : depots){
        all_intersections.push_back(depot);
    }

    std::unordered_map<IntersectionIdx, std::unordered_map<IntersectionIdx, double>> 
        distance_matrix = computeAllLineDistances(all_intersections, all_intersections);
    
    std::set<int> picked_up, delivered;
    std::vector<bool> is_picked(deliveries.size(), false), is_delivered(deliveries.size(), false);
    std::vector<CourierSubPath> final_path;

    std::unordered_map<IntersectionIdx, std::vector<int>> pickup_map, dropoff_map;
    for (int i = 0; i < deliveries.size(); i++) {
        pickup_map[deliveries[i].pickUp].push_back(i);
        dropoff_map[deliveries[i].dropOff].push_back(i);
    }

    IntersectionIdx current = start_depot;

    while (picked_up.size() < deliveries.size() || delivered.size() < deliveries.size()) {
        IntersectionIdx next = -1;
        double best_cost = std::numeric_limits<double>::max();
        bool is_next_dropoff = false;

        //use precomputed distance matrix instead of calculating on the spot 
        for (const auto& [pickup, idxs] : pickup_map) {
            if (!is_picked[idxs[0]]) {
                double est_cost = distance_matrix[current][pickup] / 25.0; 
                if (est_cost < best_cost) {
                    best_cost = est_cost;
                    next = pickup;
                    is_next_dropoff = false;
                }
            }
        }

        for (const auto& [dropoff, idxs] : dropoff_map) {
            for (int i : idxs) {
                if (is_picked[i] && !is_delivered[i]) {
                    double est_cost = distance_matrix[current][dropoff] / 25.0; 
                    if (est_cost < best_cost) {
                        best_cost = est_cost;
                        next = dropoff;
                        is_next_dropoff = true;
                    }
                }
            }
        }

        if (next == -1) return {}; 

        //after choosing the best next location, compute the actual path
        auto path = findPathBetweenIntersections(turn_penalty, {current, next});
        if (path.empty()) return {};

        CourierSubPath sub;
        sub.intersections = {current, next};
        sub.subpath = path;

        if (!is_next_dropoff) {
            for (int i : pickup_map[next]) {
                is_picked[i] = true;
                picked_up.insert(i);
            }
        } 
        
        else{
            for (int i : dropoff_map[next]){
                if (is_picked[i]){
                    is_delivered[i] = true;
                    delivered.insert(i);
                }
            }
        }

        final_path.push_back(sub);
        current = next;
    }

    //get the best end depot using precomputed distances
    IntersectionIdx best_end_depot = depots[0];
    double best_dist = std::numeric_limits<double>::max();
    
    for (IntersectionIdx depot : depots) {
        double dist = distance_matrix[current][depot];
        if (dist < best_dist){
            best_dist = dist;
            best_end_depot = depot;
        }
    }
    
    //compute the actual path to the best end depot
    auto end_path = findPathBetweenIntersections(turn_penalty, {current, best_end_depot});
    if (!end_path.empty()){
        CourierSubPath final_leg;
        final_leg.intersections = {current, best_end_depot};
        final_leg.subpath = end_path;
        final_path.push_back(final_leg);
    }

    return final_path;
}

//helpers
double estimateTravelTime(IntersectionIdx from, IntersectionIdx to) {
    LatLon pos1 = getIntersectionPosition(from);
    LatLon pos2 = getIntersectionPosition(to);
    double distance = findDistanceBetweenTwoPoints(pos1, pos2);
    return distance / 25.0;
}

IntersectionIdx findClosestDepot(
    IntersectionIdx from,
    const std::vector<IntersectionIdx>& depots,
    float turn_penalty)
{
    IntersectionIdx best_depot = depots[0];
    double best_time = std::numeric_limits<double>::max();

    for (IntersectionIdx depot : depots) {
        auto path = findPathBetweenIntersections(turn_penalty, {from, depot});
        if (!path.empty()) {
            double time = computePathTravelTime(turn_penalty, path);
            if (time < best_time) {
                best_time = time;
                best_depot = depot;
            }
        }
    }
    return best_depot;
}

double computeTotalPathTime(const std::vector<CourierSubPath>& path, float turn_penalty) {
    double total_time = 0;
    for (const auto& sub : path) {
        total_time += computePathTravelTime(turn_penalty, sub.subpath);
    }
    return total_time;
}

IntersectionIdx findClosestDepot(
    IntersectionIdx from,
    const std::vector<IntersectionIdx>& depots,
    const std::unordered_map<IntersectionIdx, std::unordered_map<IntersectionIdx, double>>& distance_matrix)
{
    IntersectionIdx best_depot = depots[0];
    double best_time = std::numeric_limits<double>::max();

    for (IntersectionIdx depot : depots) {
        double time = distance_matrix.at(from).at(depot) / 25.0;  // Use straight-line speed
        if (time < best_time) {
            best_time = time;
            best_depot = depot;
        }
    }

    return best_depot;
}

void findBestSequence(
    IntersectionIdx current_location,
    std::set<int> picked_up,
    std::set<int> delivered,
    std::vector<bool> is_picked,
    std::vector<bool> is_delivered,
    const std::unordered_map<IntersectionIdx, std::unordered_map<IntersectionIdx, double>>& distance_matrix,
    const std::unordered_map<IntersectionIdx, std::vector<int>>& pickup_map,
    const std::unordered_map<IntersectionIdx, std::vector<int>>& dropoff_map,
    int total_deliveries,
    int current_depth,
    int max_depth,
    double current_cost,
    std::vector<IntersectionIdx> current_sequence,
    double& best_sequence_cost,
    std::vector<IntersectionIdx>& best_sequence)
{
    // Base case: we've reached our lookahead depth or delivered everything
    if (current_depth >= max_depth || delivered.size() == total_deliveries) {
        // Check if this sequence is better than our current best
        if (current_cost < best_sequence_cost && !current_sequence.empty()) {
            best_sequence_cost = current_cost;
            best_sequence = current_sequence;
        }
        return;
    }
    
    // Find all valid next locations
    std::vector<std::pair<IntersectionIdx, double>> next_candidates;
    
    // Check pickup locations
    for (const auto& [pickup, idxs] : pickup_map) {
        for (int i : idxs) {
            if (!is_picked[i]) {
                double dist = distance_matrix.at(current_location).at(pickup) / 25.0;
                next_candidates.push_back({pickup, dist});
                break;  // Only need to consider each pickup location once
            }
        }
    }
    
    // Check dropoff locations for packages we've already picked up
    for (const auto& [dropoff, idxs] : dropoff_map) {
        for (int i : idxs) {
            if (is_picked[i] && !is_delivered[i]) {
                double dist = distance_matrix.at(current_location).at(dropoff) / 25.0;
                next_candidates.push_back({dropoff, dist});
                break;  // Only need to consider each dropoff location once
            }
        }
    }
    
    // If no valid moves, return
    if (next_candidates.empty()) {
        return;
    }
    
    // Sort candidates by distance (optional optimization)
    std::sort(next_candidates.begin(), next_candidates.end(), 
              [](const auto& a, const auto& b) { return a.second < b.second; });
              
    // Limit the branching factor (optional optimization)
    const int max_branches = 5;  // Consider only the closest 5 options
    if (next_candidates.size() > max_branches) {
        next_candidates.resize(max_branches);
    }
    
    // Try each candidate
    for (const auto& [next_location, dist] : next_candidates) {
        // Create new state for this branch
        std::set<int> new_picked = picked_up;
        std::set<int> new_delivered = delivered;
        std::vector<bool> new_is_picked = is_picked;
        std::vector<bool> new_is_delivered = is_delivered;
        
        // Update state for pickup
        if (pickup_map.find(next_location) != pickup_map.end()) {
            for (int i : pickup_map.at(next_location)) {
                if (!new_is_picked[i]) {
                    new_is_picked[i] = true;
                    new_picked.insert(i);
                }
            }
        }
        
        // Update state for dropoff
        if (dropoff_map.find(next_location) != dropoff_map.end()) {
            for (int i : dropoff_map.at(next_location)) {
                if (new_is_picked[i] && !new_is_delivered[i]) {
                    new_is_delivered[i] = true;
                    new_delivered.insert(i);
                }
            }
        }
        
        // Create new sequence for this branch
        std::vector<IntersectionIdx> new_sequence = current_sequence;
        new_sequence.push_back(next_location);
        
        // Recursively explore this branch
        findBestSequence(
            next_location,
            new_picked,
            new_delivered,
            new_is_picked,
            new_is_delivered,
            distance_matrix,
            pickup_map,
            dropoff_map,
            total_deliveries,
            current_depth + 1,
            max_depth,
            current_cost + dist,
            new_sequence,
            best_sequence_cost,
            best_sequence
        );
    }
}

std::vector<IntersectionIdx> apply2OptSwap(const std::vector<IntersectionIdx>& path, size_t i, size_t k) {
    std::vector<IntersectionIdx> new_path = path;
    std::reverse(new_path.begin() + i, new_path.begin() + k + 1);
    return new_path;
}

bool isLegalPath(const std::vector<IntersectionIdx>& path,
                 const std::vector<DeliveryInf>& deliveries){

    std::unordered_set<int> picked_up;
    std::unordered_map<IntersectionIdx, std::vector<int>> pickups_map;
    std::unordered_map<IntersectionIdx, std::vector<int>> dropoffs_map;

    for (size_t i = 0; i < deliveries.size(); ++i) {
        pickups_map[deliveries[i].pickUp].push_back(i);
        dropoffs_map[deliveries[i].dropOff].push_back(i);
    }

    for (IntersectionIdx loc : path) {
        for (int i : pickups_map[loc]) {
            picked_up.insert(i);
        }
        for (int i : dropoffs_map[loc]) {
            if (picked_up.find(i) == picked_up.end()) {
                return false; // trying to drop off without picking up
            }
        }
    }
    return true;
}