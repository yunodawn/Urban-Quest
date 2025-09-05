/* 
 * Copyright 2025 University of Toronto
 *
 * Permission is hereby granted, to use this software and associated 
 * documentation files (the "Software") in course work at the University 
 * of Toronto, or for personal use. Other uses are prohibited, in 
 * particular the distribution of the Software either publicly or to third 
 * parties.
 *
 * The above copyright notice and this permission notice shall be included in 
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR 
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE 
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

// #include <vector>
// #include "StreetsDatabaseAPI.h"
// #include "m4.h"
// #include "m3.h"
// #include "m1.h"
// #include <thread>
// #include <mutex>
// #include <future>
// #include <atomic>
// #include <functional>
// #include <unordered_set>
// #include <unordered_map>
// #include <queue>
// #include <chrono>
// #include <omp.h>
// #include <cstdlib>
// #include <ctime>
// #include <cmath>
// #include <limits>
// #include <set>

// struct Location{
//     IntersectionIdx id;
//     bool is_pickup;
//     int delivery_idx;
// };

// //create a struct to keep track of our current state during the search
// struct State{
//     IntersectionIdx current_location;
//     std::vector<bool> pickups_done;
//     std::vector<bool> dropoffs_done;
//     double total_cost;
//     std::vector<IntersectionIdx> path;
// };

// bool isLegalPath(const std::vector<IntersectionIdx>& path,
//                  const std::vector<DeliveryInf>& deliveries);

// std::vector<IntersectionIdx> apply2OptSwap(const std::vector<IntersectionIdx>& path, size_t i, size_t k);

// std::unordered_map<IntersectionIdx, std::unordered_map<IntersectionIdx, std::vector<StreetSegmentIdx>>>
// computeAllPaths(const float turn_penalty, 
//                 const std::vector<DeliveryInf>& deliveries,
//                 const std::vector<IntersectionIdx>& depots);

// double evaluateGreedyScore(
//     const Location& loc,
//     const State& current_state,
//     const std::vector<Location>& all_locations,
//     const std::unordered_map<IntersectionIdx, std::unordered_map<IntersectionIdx, std::vector<StreetSegmentIdx>>>& all_paths,
//     const double turn_penalty,
//     int& evaluation_count,
//     const int total_paths
// );

// //actual functions BELOW ///////////////////////////////////////////////////////////////////////////////

// std::unordered_map<IntersectionIdx, std::unordered_map<IntersectionIdx, std::vector<StreetSegmentIdx>>>
// computeAllPaths(const float turn_penalty,
//                 const std::vector<DeliveryInf>& deliveries,
//                 const std::vector<IntersectionIdx>& depots){

//     struct Node{
//         IntersectionIdx intersection;
//         double g_cost;
//         StreetSegmentIdx segment;
//         bool operator>(const Node& other) const{
//             return g_cost > other.g_cost;
//         }
//     };

//     std::unordered_set<IntersectionIdx> all_intersections;
//     std::unordered_map<IntersectionIdx, std::unordered_set<IntersectionIdx>> required_targets;

//     //collect all relevant intersections by going through
//     //the delivery requests in the deliveries vector: depots, pickups, dropoffs
    
//     for (const auto& delivery : deliveries){
//         all_intersections.insert(delivery.pickUp);
//         all_intersections.insert(delivery.dropOff);

//         //from the pickup point, we must5 be able to reach the dropOff point 
//         required_targets[delivery.pickUp].insert(delivery.dropOff);

//         // loop thru every depot-from depots to pickup, and dropoff to depot
//         for (const auto& depot : depots){
//             all_intersections.insert(depot);
//             required_targets[depot].insert(delivery.pickUp);
//             required_targets[delivery.dropOff].insert(depot);
//             //depot connected to pickup which is connected to its dropoffs which is connected back to the depots
//         }
//     }

//     //add all pairwise pickup/dropoff paths that might be chained
//     std::vector<IntersectionIdx> nodes(all_intersections.begin(), all_intersections.end());
    
//     //#pragma omp parallel for
//     //slows downf or smaller maps (easy toronto)

//     //loop through each SOURCE intersection
//     for (size_t i = 0; i < nodes.size(); i++){
//         //loop through each DEST intersection
//         for (size_t j = 0; j < nodes.size(); j++) {
//             if (nodes[i] != nodes[j]) {
//                 //no self paths- builds a list of paths to every other path
//                 required_targets[nodes[i]].insert(nodes[j]);
//             }
//         }
//     }
//     std::unordered_map<IntersectionIdx, std::unordered_map<IntersectionIdx, std::vector<StreetSegmentIdx>>> all_paths;

//     //used ChatGPT to help us identify the issue, it recognized pramga omp parallel only worked for for loops of format (int i = 0; i...) and (x : y) was not valid
//     //April 5th, 2025 https://chatgpt.com/share/67f1dce4-56a0-800f-b3da-b5e4a76846bf
//     //gave it this function and asked why compilation was failing, and told me it failed due to formatting of for loop which then I fixed
//     std::vector<std::pair<IntersectionIdx, std::unordered_set<IntersectionIdx>>> entries(required_targets.begin(), required_targets.end());

//     //GPT also helped with idea of local paths, all from same GPT covnersation as above so please refer to that citation
//     //Helped with idea of local thread merged to one and helped with code
//     std::vector<std::unordered_map<IntersectionIdx, std::unordered_map<IntersectionIdx, std::vector<StreetSegmentIdx>>>> thread_local_paths(omp_get_max_threads());

//     //run Djikstra from each source to its needed destinations
//     #pragma omp parallel for
//     for (int i = 0; i < entries.size(); i++) {
//         const auto& [source, targets] = entries[i];
//         int threadID = omp_get_thread_num();
//         auto& local_paths = thread_local_paths[threadID];
        
//         std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open_set;
//         std::unordered_map<IntersectionIdx, double> best_time;
//         std::unordered_map<IntersectionIdx, StreetSegmentIdx> came_from;
//         std::unordered_map<IntersectionIdx, IntersectionIdx> parent;
//         //reconstruct the paths after reaching a target 
//         std::unordered_set<IntersectionIdx> remaining_targets = targets;

//         open_set.push({source, 0.0, -1});
//         best_time[source] = 0.0;

//         while (!open_set.empty() && !remaining_targets.empty()) {
//             Node current = open_set.top();
//             open_set.pop();

//             IntersectionIdx current_intersection = current.intersection;

//             //if we already visited with a better time- skip to save time
//             if (current.g_cost > best_time[current_intersection]) continue;

//             //if we’ve reached a needed destination, reconstruct the path
//             if (remaining_targets.count(current_intersection)){
//                 std::vector<StreetSegmentIdx> path;
//                 for (IntersectionIdx at = current_intersection; at != source; at = parent[at]){
//                     StreetSegmentIdx seg = came_from[at];
//                     const auto& info = getStreetSegmentInfo(seg);
                    
//                     if (info.oneWay && info.from != parent[at]) {
                        
//                         path = findPathBetweenIntersections(turn_penalty, {source, current_intersection});
//                         continue; //illegal due to one ways- use the A star here then 
//                     }
                    
//                     path.push_back(seg);
//                 }
                
//                 //storing the legal path
//                 std::reverse(path.begin(), path.end());
//                 local_paths[source][current_intersection] = std::move(path);
//                 remaining_targets.erase(current_intersection);
//             }

//             //expanding the neighbours 
//             for (StreetSegmentIdx seg : findStreetSegmentsOfIntersection(current_intersection)){
//                 StreetSegmentInfo seg_info = getStreetSegmentInfo(seg);

//                 if (seg_info.oneWay && seg_info.from != current_intersection) {continue;}

//                 IntersectionIdx next = (seg_info.from == current_intersection) ? seg_info.to : seg_info.from;
//                 double travel_time = findStreetSegmentTravelTime(seg);

//                 double new_cost = current.g_cost + travel_time;
//                 if (current.segment != -1 &&
//                     getStreetSegmentInfo(current.segment).streetID != seg_info.streetID) {
//                     new_cost += turn_penalty;
//                 }

//                 if (!best_time.count(next) || new_cost < best_time[next]) {
//                     best_time[next] = new_cost;
//                     came_from[next] = seg;
//                     parent[next] = current_intersection;
//                     open_set.push({next, new_cost, seg});
//                 }
//             }
//         }

//         // for (const auto& target : targets) {
//         //     if (local_paths[source].find(target) == local_paths[source].end() || 
//         //         local_paths[source][target].empty()) {
//         //         // Try a more exhaustive search for this specific target if needed
//         //         std::vector<StreetSegmentIdx> fallback_path = findPathBetweenIntersections(
//         //             turn_penalty, {source, target});

//         //         local_paths[source][target] = fallback_path;
//         //     }
//         // }
  
//     }

//     for (const auto& thread_map : thread_local_paths) {
//         for (const auto& [source, target_map] : thread_map) {
//             for (const auto& [target, path] : target_map) {
//                 all_paths[source][target] = path;
//             }
//         }
//     }   

//     return all_paths;
// }

// std::vector<CourierSubPath> travelingCourier(
//                             const float turn_penalty,
//                             const std::vector<DeliveryInf>& deliveries,
//                             const std::vector<IntersectionIdx>& depots){

//     //limit the computation to 45 secs
//     auto start_time = std::chrono::high_resolution_clock::now();
//     auto now = std::chrono::high_resolution_clock::now();
//     double elapsed = std::chrono::duration<double>(now - start_time).count();
//     const double time_limit_sec = 49.5;
    
//     //part 1- get all of the intersections we need (e.g depots, pickups, dropoffs)
//     std::vector<IntersectionIdx> important_intersections = depots;
//     std::unordered_set<IntersectionIdx> intersection_set(depots.begin(), depots.end());
    
//     for (const auto& delivery : deliveries){
//         if (intersection_set.find(delivery.pickUp) == intersection_set.end()){
            
//             important_intersections.push_back(delivery.pickUp);
//             intersection_set.insert(delivery.pickUp);

//         }
        
//         if (intersection_set.find(delivery.dropOff) == intersection_set.end()){
           
//            important_intersections.push_back(delivery.dropOff);
//             intersection_set.insert(delivery.dropOff);
//         }
//     }
    
//     // part 2- precompute all paths between important intersections using multi-dest djiksra function
//     auto all_paths = computeAllPaths(turn_penalty, deliveries, depots);
    
//     //part 3- use a greedy algorithm to calculate the actual path 
    
//     std::vector<Location> all_locations;
//     for (size_t i = 0; i < deliveries.size(); i++){
        
//         all_locations.push_back({deliveries[i].pickUp, true, static_cast<int>(i)});
//         all_locations.push_back({deliveries[i].dropOff, false, static_cast<int>(i)});

//     }
    
//     int total_paths = all_locations.size();

//     //find the best route using a greedy approach
//     double best_cost = std::numeric_limits<double>::infinity();
//     std::vector<IntersectionIdx> best_path;
    
//     int totalThreads = omp_get_max_threads();
//     std::vector<double> local_cost(totalThreads, std::numeric_limits<double>::infinity());
//     std::vector<std::vector<IntersectionIdx>> local_path(totalThreads);
    
//     //HEURISTIC: MULTISTART
//     //change the starting depot to find the fastest one
//     #pragma omp parallel for
//     //for (IntersectionIdx start_depot : depots){
//     for (int j = 0; j < depots.size(); j++) {
//         IntersectionIdx start_depot = depots[j];

//         //ensure we dont surpass time limit for test case
//         auto current = std::chrono::high_resolution_clock::now();
//         auto timeElapsed = std::chrono::duration<double>(current - start_time).count();
//         // if (timeElapsed > time_limit_sec){
//         //     //break;
//         //     //omp cant have breaks so i dont know if this will ruin it but needs to be continue
//         //     continue;
//         // }

//         int evaluation_count = 0;

//         State initial_state;
//         initial_state.current_location = start_depot;
//         initial_state.pickups_done.resize(deliveries.size(), false);
//         initial_state.dropoffs_done.resize(deliveries.size(), false);
//         initial_state.total_cost = 0.0;
//         initial_state.path.push_back(start_depot);
        
//         //always pick the closest valid next location no matter what
//         State current_state = initial_state;
//         bool can_move = true;
        
//         while (can_move){
//             can_move = false;
//             double best_next_cost = std::numeric_limits<double>::infinity();
//             IntersectionIdx best_next_location = -1;
//             bool is_next_pickup = false;
//             int next_delivery_idx = -1;
            
//             //find the best next location
//             for (const auto& loc : all_locations){
//                 if (loc.is_pickup && !current_state.pickups_done[loc.delivery_idx]){
//                     //if this is a vaalid pickup location
//                     double cost = computePathTravelTime(turn_penalty, all_paths[current_state.current_location][loc.id]);

//                     if (cost < best_next_cost && !all_paths[current_state.current_location][loc.id].empty()){
//                         best_next_cost = cost;
//                         best_next_location = loc.id;
//                         is_next_pickup = true;
//                         next_delivery_idx = loc.delivery_idx;
//                         can_move = true;
//                     }
//                 } 
                
//                 else if (!loc.is_pickup && current_state.pickups_done[loc.delivery_idx] && !current_state.dropoffs_done[loc.delivery_idx]){
                    
//                     // if this is a valid dropoff (meaning package has been picked up already)
//                     double cost = computePathTravelTime(turn_penalty, all_paths[current_state.current_location][loc.id]);
//                     if (cost < best_next_cost && !all_paths[current_state.current_location][loc.id].empty()){
//                         best_next_cost = cost;
//                         best_next_location = loc.id;
//                         is_next_pickup = false;
//                         next_delivery_idx = loc.delivery_idx;
//                         can_move = true;
//                     }

//                 }
//             }
            
//             //if we have a valid next location then update the state
//             if (can_move){
//                 current_state.total_cost += best_next_cost;
//                 current_state.current_location = best_next_location;
//                 current_state.path.push_back(best_next_location);
                
//                 if (is_next_pickup){
//                     current_state.pickups_done[next_delivery_idx] = true;
//                 } 
//                 else{
//                     current_state.dropoffs_done[next_delivery_idx] = true;
//                 }
//             }
//         }
        
//         bool all_completed = true;
//         for (size_t i = 0; i < deliveries.size(); i++) {
//             if (!current_state.dropoffs_done[i]) {
//                 all_completed = false;
//                 break;
//             }
//         }
        
//         //if all deliveries are done then we can find the best depot to return to
//         if (all_completed){
//             double best_return_cost = std::numeric_limits<double>::infinity();
//             IntersectionIdx best_depot = -1;
            
//             //check all the end depots 
//             for (IntersectionIdx end_depot : depots){
//                 double return_cost = computePathTravelTime(turn_penalty, all_paths[current_state.current_location][end_depot]);
//                 if (return_cost < best_return_cost && !all_paths[current_state.current_location][end_depot].empty()){
//                     best_return_cost = return_cost;
//                     best_depot = end_depot;
//                 }
//             }
            
//             if (best_depot != -1){
//                 current_state.total_cost += best_return_cost;
//                 current_state.path.push_back(best_depot);
                
//                 //change the best solution if we found a better one 
// //                if (current_state.total_cost < best_cost){
// //                    local_cost[omp_get_thread_num()] = current_state.total_cost;
// //                    local_path[omp_get_thread_num()] = current_state.path;
// //                }
//                 local_cost[omp_get_thread_num()] = current_state.total_cost;
//                 local_path[omp_get_thread_num()] = current_state.path;
//             }
//         }
//     }
        
//     for (int i = 0; i < omp_get_max_threads(); i++) {
//         if (local_cost[i] < best_cost) {
//             best_cost = local_cost[i];
//             best_path = local_path[i];
//         }
//     }
    
//     //part 3.5: local improvement with 2-opt perburbation 
//     bool improvement = true;
//     while (improvement) {
//         improvement = false;

//         //check if time limit has passed before each pass
//         now = std::chrono::high_resolution_clock::now();
//         elapsed = std::chrono::duration<double>(now - start_time).count();
//         // if (elapsed > time_limit_sec) {
//         //     break; //exit if over 45 seconds
//         // }

//         for (size_t i = 1; i < best_path.size() - 2; ++i) {
//             for (size_t k = i + 1; k < best_path.size() - 1; ++k) {
                
//                 now = std::chrono::high_resolution_clock::now();
//                 elapsed = std::chrono::duration<double>(now - start_time).count();
//                 // if (elapsed > time_limit_sec) {
//                 //     break; //early termination
//                 // }

//                 //apply 2-opt perturbation and ensure path is legal
//                 std::vector<IntersectionIdx> new_path = apply2OptSwap(best_path, i, k);
//                 if (!isLegalPath(new_path, deliveries)) continue;

//                 double new_cost = 0.0;
//                 bool valid_path = true;
//                 for (size_t j = 0; j < new_path.size() - 1; ++j) {
//                     const auto& segment = all_paths.at(new_path[j]).at(new_path[j + 1]);
//                     if (segment.empty()) {
//                         valid_path = false;
//                         break;
//                     }
//                     new_cost += computePathTravelTime(turn_penalty, segment);
//                 }

//                 if (valid_path && new_cost < best_cost) {
//                     best_path = new_path;
//                     best_cost = new_cost;
//                     improvement = true;
//                     break; //accept the first valid improvement (greedy)
//                 }
//             }
//             if (improvement) break;
//         }
//     }

    
//     //part 3.6 (simulated annealing)
//     srand(time(NULL));

//     double T = 1000.0; //initial temp
//     double alpha = 0.995; //cooling rate

//     //current solution starts as the best found so far
//     std::vector<IntersectionIdx> current_path = best_path;
//     double current_cost = best_cost;

//     while (true){
//         now = std::chrono::high_resolution_clock::now();
//         elapsed = std::chrono::duration<double>(now - start_time).count();
//         //stop if we exceed time or T is effectively zero
//         if (elapsed > time_limit_sec || T < 1e-9) {
//             break;
//         }

//         //randomly choose a segment [i..k] for 2-opt
//         if (current_path.size() < 4) {
//             //too short to 2-opt, just break
//             break;
//         }
//         size_t max_index = current_path.size() - 2; // Exclude last depot
//         if (max_index <= 2) break; // Nothing to improve
//         size_t i = 1 + (rand() % (max_index - 1)); // i in [1, max_index - 1]
//         size_t k = i + 1 + (rand() % (max_index - i)); // k in [i+1, max_index]
//         if (k < i){
//             std::swap(i, k);
//         }

//         //apply 2-opt
//         std::vector<IntersectionIdx> new_path = apply2OptSwap(current_path, i, k);

//         //check if new path is legal
//         if (isLegalPath(new_path, deliveries) == 0) {
//             //if it's not legal, skip
//             continue;
//         }

//         //compute new path cost
//         double new_cost = 0.0;
//         bool valid_path = true;
//         for (size_t j = 0; j < new_path.size() - 1; ++j) {
//             IntersectionIdx a = new_path[j];
//             IntersectionIdx b = new_path[j + 1];
//             if (all_paths.find(a) == all_paths.end() ||
//                 all_paths[a].find(b) == all_paths[a].end() ||
//                 all_paths[a][b].empty()) {
//                 valid_path = false;
//                 break;
//             }
//             new_cost += computePathTravelTime(turn_penalty, all_paths[a][b]);
//         }
        
//         if (!valid_path){
//             continue;
//         }

//         double delta = new_cost - current_cost;
//         if (delta < 0) {
//             //better solution => accept unconditionally
//             current_path = new_path;
//             current_cost = new_cost;
//             //update global best
//             if (new_cost < best_cost){
//                 best_cost = new_cost;
//                 best_path = new_path;
//             }
//         } 
        
//         else{
//             //worse solution => accept with probability e^(-delta / T)
//             double accept_prob = std::exp(-delta / T);
//             double rand_val = static_cast<double>(rand()) / RAND_MAX;
//             if (rand_val < accept_prob) {
//                 current_path = new_path;
//                 current_cost = new_cost;
//             }
//         }

//         //cool down
//         T = T * alpha;
//     }

//     // part 4- build the final courier subpaths from the best path
//     std::vector<CourierSubPath> result;
    
//     //if there was no valid path 
//     if (best_path.size() <= 1){
//         return {};
//     }
    
//     //bulding the subpaths
//     for (size_t i = 0; i < best_path.size() - 1; i++){
//         CourierSubPath subpath;
//         subpath.intersections = {best_path[i], best_path[i+1 ]};
//         subpath.subpath = all_paths[best_path[i]][best_path[i+1]];
//         result.push_back(subpath);
//     }
    
//     return result;
// }


// double evaluateGreedyScore(const Location& loc, const State& current_state,
//     const std::vector<Location>& all_locations, 
//     const std::unordered_map<IntersectionIdx, std::unordered_map<IntersectionIdx, std::vector<StreetSegmentIdx>>>& all_paths,
//     const double turn_penalty, int& evaluation_count, const int total_paths){

//     // README: the important stuff we want to mess with is the THRESHOLD variable and the if statement on line 580

//     //counting the number of paths we have
//     evaluation_count++;

//     auto source_it = all_paths.find(current_state.current_location);
//     if (source_it == all_paths.end()){
//         return std::numeric_limits<double>::infinity();
//     }

//     auto target_it = source_it->second.find(loc.id);
//     if (target_it == source_it->second.end() || target_it->second.empty()){
//         return std::numeric_limits<double>::infinity();
//     }

//     double cost_to_loc = computePathTravelTime(turn_penalty, target_it->second);

//     double threshold = total_paths * 0.37;
//     if (evaluation_count <= threshold) {
//         State simulated = current_state;
//         simulated.current_location = loc.id;
        
//         if (loc.is_pickup) {simulated.pickups_done[loc.delivery_idx] = true;}
//         else {simulated.dropoffs_done[loc.delivery_idx] = true;}

//         double min_next_cost = std::numeric_limits<double>::infinity();
//         auto next_source_it = all_paths.find(loc.id);
        
//         if (next_source_it != all_paths.end()){
//             for (const auto& loc2 : all_locations){
//                 if (loc2.is_pickup && simulated.pickups_done[loc2.delivery_idx]){
//                     continue;
//                 }
                
//                 if (!loc2.is_pickup && (!simulated.pickups_done[loc2.delivery_idx] || simulated.dropoffs_done[loc2.delivery_idx])) {
//                     continue;
//                 }
                
//                 auto next_target_it = next_source_it->second.find(loc2.id);
//                 if (next_target_it != next_source_it->second.end() && !next_target_it->second.empty()) {
//                     double cost = computePathTravelTime(turn_penalty, next_target_it->second);
//                     min_next_cost = std::min(min_next_cost, cost);
//                 }
//             }
//         }
        
//         if (min_next_cost == std::numeric_limits<double>::infinity()) {min_next_cost = 0;}
//         return cost_to_loc + (0.57 * min_next_cost);
//     } 
    
//     else {return cost_to_loc;}
// }

// std::vector<IntersectionIdx> apply2OptSwap(const std::vector<IntersectionIdx>& path, size_t i, size_t k) {
//     std::vector<IntersectionIdx> new_path = path;
//     std::reverse(new_path.begin() + i, new_path.begin() + k + 1);
//     return new_path;
// };

// bool isLegalPath(const std::vector<IntersectionIdx>& path,
//                  const std::vector<DeliveryInf>& deliveries){

//     std::unordered_set<int> picked_up;
//     std::unordered_map<IntersectionIdx, std::vector<int>> pickups_map;
//     std::unordered_map<IntersectionIdx, std::vector<int>> dropoffs_map;

//     for (size_t i = 0; i < deliveries.size(); ++i) {
//         pickups_map[deliveries[i].pickUp].push_back(i);
//         dropoffs_map[deliveries[i].dropOff].push_back(i);
//     }

//     for (IntersectionIdx loc : path) {
//         for (int i : pickups_map[loc]) {
//             picked_up.insert(i);
//         }
//         for (int i : dropoffs_map[loc]) {
//             if (picked_up.find(i) == picked_up.end()) {
//                 return false; // trying to drop off without picking up
//             }
//         }
//     }
//     return true;
// }
