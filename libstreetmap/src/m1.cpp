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
#include <iostream>
#include "m1.h"
#include "StreetsDatabaseAPI.h"
#include <cmath>
#include "OSMDatabaseAPI.h"
#include <limits>
#include <vector>
#include <map>
#include <utility>
#include <unordered_set>

#include "OSMEntity.h"
#include "OSMNode.h"
#include "OSMWay.h"
#include "OSMRelation.h"

const double EPSILON = 1e-6;

//forward declaration for helper function
double findStreetSegmentLength_Helper(StreetSegmentIdx street_segment_id);

//global variable for loadmap
std::vector<double> all_street_segment_lengths;
std::vector<double> all_street_lengths;
std::vector<std::vector<IntersectionIdx>> all_street_intersections;
std::vector<std::vector<IntersectionIdx>> all_adjacent_intersections;

std::vector<double> all_street_segment_traveltimes;
std::vector<std::vector<StreetSegmentIdx>> intersection_street_segments;

std::map<OSMID, const OSMWay*> OSMWayList;
std::map<OSMID, double> OSMWayLength;
std::map<OSMID, const OSMNode*> OSMNodeList;
std::map<OSMID, std::map<std::string, std::string>> OSMNodeTags;

std::multimap<std::string, StreetIdx> SimpleStreet; 
// loadMap will be called with the name of the file that stores the "layer-2"
// map data accessed through StreetsDatabaseAPI: the street and intersection 
// data that is higher-level than the raw OSM data). 
// This file name will always end in ".streets.bin" and you 
// can call loadStreetsDatabaseBIN with this filename to initialize the
// layer 2 (StreetsDatabase) API.
// If you need data from the lower level, layer 1, API that provides raw OSM
// data (nodes, ways, etc.) you will also need to initialize the layer 1 
// OSMDatabaseAPI by calling loadOSMDatabaseBIN. That function needs the 
// name of the ".osm.bin" file that matches your map -- just change 
// ".streets" to ".osm" in the map_streets_database_filename to get the proper
// name.
bool loadMap(std::string map_streets_database_filename) {
    bool load_successful = false; //Indicates whether the map has loaded 
                                  //successfully

    std::cout << "loadMap: " << map_streets_database_filename << std::endl;
    
    //clearing globals
    
    //findstreetsegmentintersection
    intersection_street_segments.clear();

    //findstreetlength
    all_street_lengths.clear();
    
    //findstreetsegmentlength
    all_street_segment_lengths.clear();
    
    //findstreetsegmenttraveltime
    all_street_segment_traveltimes.clear();
    
    //findintersectionsofstreet
    all_street_intersections.clear();
    
    //waylength
    OSMWayList.clear();
    OSMWayLength.clear();
    
    //osmnodetags
    OSMNodeList.clear();
    OSMNodeTags.clear();
    
    //adjacentintersections
    all_adjacent_intersections.clear();
    
    //partialidstreetid
    SimpleStreet.clear();

    const std::string street_bin = ".streets.bin";
    const std::string osm_bin = ".osm.bin";

    //loading the databases 
    load_successful = loadStreetsDatabaseBIN(map_streets_database_filename);
    if(!load_successful){return false;} 

    // //changing the name of the osm file so we can load it 
    //toronto.streets.bin -> toronto.osm.bin
    std::string map_osm_database_filename = map_streets_database_filename;
    map_osm_database_filename.replace(map_osm_database_filename.size() - street_bin.size(), street_bin.size(), osm_bin);

    load_successful = loadOSMDatabaseBIN(map_osm_database_filename);
    if(!load_successful){return false;}


    //loading map related data structures here
    //load for all high speed requirement functions

    //loading for find street segments of intersection and for get distance between two points 
    intersection_street_segments.resize(getNumIntersections());
    all_adjacent_intersections.resize(getNumIntersections());

    for (int i = 0; i < getNumIntersections(); i++) {
        std::vector<StreetSegmentIdx>& street_segments = intersection_street_segments[i];

        //for all adjacent intersections
        std::unordered_set<IntersectionIdx> adjInterSet;
            
        for(int j = 0; j < getNumIntersectionStreetSegment(i); j++) {
            int ss_id = getIntersectionStreetSegment(i, j);
            street_segments.push_back(ss_id);

            //for all adjacent intersections
            StreetSegmentInfo segment_info = getStreetSegmentInfo(street_segments[j]);
            if (segment_info.oneWay){
                if (segment_info.from == i) {
                    adjInterSet.insert(segment_info.to);}
            }

            else{
                if (segment_info.from == i){
                    adjInterSet.insert(segment_info.to);} 
                else{
                    adjInterSet.insert(segment_info.from);}
            }
        }
        //intersection_street_segments[i] = street_segments; 
        all_adjacent_intersections[i] = std::vector<IntersectionIdx>(adjInterSet.begin(), adjInterSet.end());       
    }

    //loading for find intersection of street + find street length together
    //+ street segment length + segment travel time
    
    all_street_lengths.resize(getNumStreets(), 0.0);
    all_street_segment_lengths.resize(getNumStreetSegments(), 0.0);
    all_street_segment_traveltimes.resize(getNumStreetSegments(), 0.0);
    all_street_intersections.resize(getNumStreets(), std::vector<int>());

    for(int i = 0; i < getNumStreetSegments(); i++){
        StreetSegmentInfo segment_info = getStreetSegmentInfo(i);

        //this part is for find street segment length
        double segment_length = findStreetSegmentLength_Helper(i);
        all_street_segment_lengths[i] = segment_length;

        //this part is for find street segment travel time
        all_street_segment_traveltimes[i] = all_street_segment_lengths[i] / segment_info.speedLimit;
        
        //this part is for find street length
        all_street_lengths[segment_info.streetID] += segment_length; 

        //this part is for find intersections of street
        //format: std::find(start value, end value, value ur looking for)
        //note: std find will return the end of the vector if we dont find the value we are looking for in that vector
        //for every segment we loop through, check the to and from intersections!
        //if they dont find it in the all street intersections vector, then add it in!! 
        if (std::find(all_street_intersections[segment_info.streetID].begin(), all_street_intersections[segment_info.streetID].end(), segment_info.from) == all_street_intersections[segment_info.streetID].end()) {
        all_street_intersections[segment_info.streetID].push_back(segment_info.from);
        }
        if (std::find(all_street_intersections[segment_info.streetID].begin(), all_street_intersections[segment_info.streetID].end(), segment_info.to) == all_street_intersections[segment_info.streetID].end()) {
        all_street_intersections[segment_info.streetID].push_back(segment_info.to);
        }
    }

    //loading for get osm node tag value
    for (unsigned i = 0; i < getNumberOfNodes(); i++) {
        const OSMNode* node = getNodeByIndex(i);
        OSMNodeList[node->id()] = node;
        
        
        std::map<std::string, std::string> TagMap;
        
        for (int j = 0; j < getTagCount(node); j++) {
            std::pair<std::string, std::string> tag = getTagPair(node, j);
            TagMap[tag.first] = tag.second;
        }
        OSMNodeTags[node->id()] = TagMap;
    }
    
    //loading for find way length
    double totalLength = 0;    
    for (unsigned i = 0; i < getNumberOfWays(); i++) {
        const OSMWay* way = getWayByIndex(i);
        OSMWayList[way->id()] = way;
        //get the list of node IDs in the way
        totalLength = 0;
        const std::vector<OSMID>& node_ids = getWayMembers(way);

        //get the coordinates of the first node
        auto prevNode = OSMNodeList.find(node_ids[0]);
        LatLon prevCoords = getNodeCoords(prevNode->second);

        //iterate through the remaining nodes and calculate the distance between consecutive nodes
        for(size_t j = 1; j < node_ids.size(); j++){
            auto currNode = OSMNodeList.find(node_ids[j]);
            LatLon currCoords = getNodeCoords(currNode->second);

            //calculate the distance between the previous node and the current node
            totalLength += findDistanceBetweenTwoPoints(prevCoords, currCoords);

            //update the previous node coordinates
            prevCoords = currCoords;
        }
        OSMWayLength[way->id()] = totalLength;
    }
    
    //loading for find street id of partial street names
    for (int i = 0; i < getNumStreets(); i++) {
       std::string new_str = getStreetName(i);
       
       new_str.erase(std::remove_if(new_str.begin(), new_str.end(), isspace), new_str.end());       
       std::transform(new_str.begin(), new_str.end(), new_str.begin(), ::tolower);
       
       SimpleStreet.insert(std::make_pair(new_str, i));
    }
    //returning load successful from the databases
    return load_successful;
}

void closeMap() {
    //Clean-up your map related data structures here
    
    //clears all global variables
    intersection_street_segments.clear();
    all_street_lengths.clear();
    all_street_segment_lengths.clear();
    all_street_segment_traveltimes.clear();
    all_street_intersections.clear();    
    OSMWayList.clear();
    OSMWayLength.clear();
    OSMNodeList.clear();
    OSMNodeTags.clear();
    all_adjacent_intersections.clear();
    SimpleStreet.clear();
    closeStreetDatabase();
    closeOSMDatabase();
}

std::vector<double> convertLatLonToXY(LatLon, LatLon);
//helper function for convert LatLon to XY Coordinates
std::vector<double> convertLatLonToXY(LatLon point_1, LatLon point_2){
    //convert latitude and longitude in degrees to radians
    double lat1 = point_1.latitude() * kDegreeToRadian;
    double lon1 = point_1.longitude() * kDegreeToRadian;

    double lat2 = point_2.latitude() * kDegreeToRadian;
    double lon2 = point_2.longitude() * kDegreeToRadian;

    //convert from latitude and longitude to x and y
    //(x,y) = (R*lon*cos(lat_avg), R*lat)
    double latAvg = (lat1 + lat2) / 2;

    double xCoord1 = kEarthRadiusInMeters * lon1 * std::cos(latAvg);
    double yCoord1 = kEarthRadiusInMeters * lat1;

    double xCoord2 = kEarthRadiusInMeters * lon2 * std::cos(latAvg);
    double yCoord2 = kEarthRadiusInMeters * lat2;

    std::vector<double> xyCoords = {xCoord1, yCoord1, xCoord2, yCoord2};

    return xyCoords;
}

double findDistanceBetweenTwoPoints(LatLon point_1, LatLon point_2){

    //convert latitude and longitude in degrees to radians
    double lat1 = point_1.latitude() * kDegreeToRadian;
    double lon1 = point_1.longitude() * kDegreeToRadian;

    double lat2 = point_2.latitude() * kDegreeToRadian;
    double lon2 = point_2.longitude() * kDegreeToRadian;

    //convert from latitude and longitude to x and y
    //(x,y) = (R*lon*cos(lat_avg), R*lat)
    double latAvg = (lat1 + lat2) / 2;

    double xCoord1 = kEarthRadiusInMeters * lon1 * std::cos(latAvg);
    double yCoord1 = kEarthRadiusInMeters * lat1;

    double xCoord2 = kEarthRadiusInMeters * lon2 * std::cos(latAvg);
    double yCoord2 = kEarthRadiusInMeters * lat2;

    //std::vector<double> xyCoords = {xCoord1, yCoord1, xCoord2, yCoord2};
    return std::sqrt((xCoord2 - xCoord1) * (xCoord2 - xCoord1) + (yCoord2 - yCoord1) * (yCoord2 - yCoord1));
}

double findStreetSegmentLength_Helper(StreetSegmentIdx street_segment_id){

    if (street_segment_id < 0 || street_segment_id >= getNumStreetSegments()){
        return 0.0;
    }

    StreetSegmentInfo segment_info = getStreetSegmentInfo(street_segment_id);

    //getting the start and end position of the street segment
    LatLon starting_point = getIntersectionPosition(segment_info.from);
    LatLon end_point = getIntersectionPosition(segment_info.to);

    //initialize a total length
    double total_length = 0.0;

    //getting the total length using the previous function ->for no curve points
    if(segment_info.numCurvePoints == 0){
        total_length = findDistanceBetweenTwoPoints(starting_point, end_point);
        return total_length;
    }

    //for curve points
    else{
        LatLon last_pt = starting_point; //set the last point to the starting point to start

        //basically we are gonna go in a loop around the curve points 
        //and get the distance of the curve by repeatedly adding the distance between the curve points
        for(int i = 0; i < segment_info.numCurvePoints; i++){
            LatLon curve_pt = getStreetSegmentCurvePoint(street_segment_id, i);
            total_length += findDistanceBetweenTwoPoints(last_pt, curve_pt);
            last_pt = curve_pt;
        }
        //at the end, add length from the last curve pt to the end of the straight
        total_length += findDistanceBetweenTwoPoints(last_pt, end_point);
    }
    return total_length;
    //potential case: what if the street is straight and then curves and then is straight and then curves again?
}

double findStreetSegmentLength(StreetSegmentIdx street_segment_id){
    return all_street_segment_lengths[street_segment_id];
}

double findStreetSegmentTravelTime(StreetSegmentIdx street_segment_id){
    return all_street_segment_traveltimes[street_segment_id];
}

double findStreetSegmentTurnAngle(StreetSegmentIdx src_street_segment_id, StreetSegmentIdx dst_street_segment_id){
    
    //initialize doubles for angle calculation
    double theta = NO_ANGLE;
    double cosTheta;

    double srcDist, dstDist, segDist = 0;

    //get the latitude and longitude of the from and to of each street segment
    StreetSegmentInfo srcInfo = getStreetSegmentInfo(src_street_segment_id);
    StreetSegmentInfo dstInfo = getStreetSegmentInfo(dst_street_segment_id);

    LatLon srcFrom = getIntersectionPosition(srcInfo.from);
    LatLon srcTo = getIntersectionPosition(srcInfo.to);
    LatLon dstFrom = getIntersectionPosition(dstInfo.from);
    LatLon dstTo = getIntersectionPosition(dstInfo.to);

    //check their latlon dont match (street segments connect)
    if (!( ((srcTo == dstTo)) ||
           ((srcTo == dstFrom)) ||
           ((srcFrom == dstTo)) ||
           ((srcFrom == dstFrom)) ) ){
        return theta;
    }

    else{
        //if they intersect, consider curves. (this assumes srcTo connects to dstTo)
        if((srcTo == dstTo)) {
            if (dstInfo.oneWay){
                return theta;
            }
            //gives the LatLon of the closest point of the segment to the "to"
            if (srcInfo.numCurvePoints > 0){
                srcFrom = getStreetSegmentCurvePoint(src_street_segment_id, (srcInfo.numCurvePoints)-1);
            }
            if (dstInfo.numCurvePoints > 0){
                dstFrom = getStreetSegmentCurvePoint(dst_street_segment_id, (dstInfo.numCurvePoints)-1);
            }

            segDist = findDistanceBetweenTwoPoints(dstFrom, srcFrom);

        }
        //srcto to dstfrom
        else if ((srcTo == dstFrom)){
            //gives the LatLon of the closest point of the segment to the "to"
            if (srcInfo.numCurvePoints > 0){
                srcFrom = getStreetSegmentCurvePoint(src_street_segment_id, (srcInfo.numCurvePoints)-1);
            }
            if (dstInfo.numCurvePoints > 0){
                dstTo = getStreetSegmentCurvePoint(dst_street_segment_id, 0);
            }
            
            segDist = findDistanceBetweenTwoPoints(srcFrom, dstTo);
        }
        //srcfrom to dstto
        else if ((srcFrom == dstTo)){
            if (srcInfo.oneWay){
                return theta;
            }
            if (dstInfo.oneWay){
                return theta;
            }
            //gives the LatLon of the closest point of the segment to the "to"
            if (srcInfo.numCurvePoints > 0){
                srcTo = getStreetSegmentCurvePoint(src_street_segment_id, 0);
            }
            if (dstInfo.numCurvePoints > 0){
                dstFrom = getStreetSegmentCurvePoint(dst_street_segment_id, (dstInfo.numCurvePoints)-1);
            }
            
            segDist = findDistanceBetweenTwoPoints(srcTo, dstFrom);
        }
        //srcfrom to dstfrom
        else if ((srcFrom == dstFrom)){
            if (srcInfo.oneWay){
                return theta;
            }
            //gives the LatLon of the closest point of the segment to the "to"
            if (srcInfo.numCurvePoints > 0){
                srcTo = getStreetSegmentCurvePoint(src_street_segment_id, 0);
            }
            if (dstInfo.numCurvePoints > 0){
                dstTo = getStreetSegmentCurvePoint(dst_street_segment_id, 0);
            }
            
            segDist = findDistanceBetweenTwoPoints(srcTo, dstTo);
        }

        //use laws of cosine to find angle
        srcDist = findDistanceBetweenTwoPoints(srcFrom, srcTo);
        dstDist = findDistanceBetweenTwoPoints(dstFrom, dstTo);
        
        //to avoid division by zero
        if (srcDist == 0 || dstDist == 0 || segDist == 0){
            return theta;
        }

        cosTheta = std::clamp(((pow(srcDist,2) + pow(dstDist, 2) - pow(segDist, 2)) / (2*srcDist*dstDist)), -1.0, 1.0);

    }

    return M_PI - acos(cosTheta);
}

std::vector<IntersectionIdx> findAdjacentIntersections(IntersectionIdx intersection_id){
    return all_adjacent_intersections[intersection_id];
}

IntersectionIdx findClosestIntersection(LatLon my_position){
    if(getNumIntersections() == 0){ //edge case where there are no intersections?
        return -1;
    }

    IntersectionIdx closest_intersection = 0; //initialize closest intersection to 0 
    double min_distance = std::numeric_limits<double>::infinity(); // use 1e30 if doesnt work

    //now we loop through ALL the intersections until we find one with the closest coords to the position 
    for(IntersectionIdx i = 0; i < getNumIntersections(); i++){ 
        LatLon intersection_position = getIntersectionPosition(i); //find current intersection position
        double current_distance = findDistanceBetweenTwoPoints(my_position, intersection_position); //get distance between intersection and us

        if (intersection_position == my_position) {
            return i; // return if we are already in an intersection
        }

        if(current_distance < min_distance){
            min_distance = current_distance;
            closest_intersection = i;
        }
    }
    return closest_intersection;
}

std::vector<StreetSegmentIdx> findStreetSegmentsOfIntersection(IntersectionIdx intersection_id){
    return intersection_street_segments[intersection_id];
    //code here is taken from the tutorial and we are given permission to use it, in Shuran's words during the tutorial
}

// Returns all intersections along the given street.
// There should be no duplicate intersections in the returned vector.
// Speed Requirement --> high
std::vector<IntersectionIdx> findIntersectionsOfStreet(StreetIdx street_id){
    return all_street_intersections[street_id];
}

//expanded from intersectionofstreet
std::vector<IntersectionIdx> findIntersectionsOfTwoStreets(std::pair<StreetIdx, StreetIdx> street_ids){
    
    //getting the vectors of each intersection
    std::vector<IntersectionIdx> intersections_of_street1 = findIntersectionsOfStreet(street_ids.first);
    std::vector<IntersectionIdx> intersections_of_street2 = findIntersectionsOfStreet(street_ids.second);

    //sorting the vectors of each intersection
    //https://en.cppreference.com/w/cpp/algorithm/sort
    std::sort(intersections_of_street1.begin(), intersections_of_street1.end());
    std::sort(intersections_of_street2.begin(), intersections_of_street2.end());

    //create the return vector
    std::vector<IntersectionIdx> same_intersection;

    //use set intersetcion function to automatically sort
    //review this in case it doesnt meet time requirements 
    //https://en.cppreference.com/w/cpp/iterator/back_inserter
    std::set_intersection(intersections_of_street1.begin(), intersections_of_street1.end(), 
    intersections_of_street2.begin(), intersections_of_street2.end(), std::back_inserter(same_intersection));

    //delete duplicates 
    auto last = std::unique(same_intersection.begin(), same_intersection.end());
    same_intersection.erase(last, same_intersection.end());
    
    return same_intersection; 
}

// Returns all street ids corresponding to street names that start with the
// given prefix.
// The function should be case-insensitive to the street prefix.
// The function should ignore spaces.
// For example, both "bloor " and "BloOrst" are prefixes to
// "Bloor Street East".
// If no street names match the given prefix, this routine returns an empty
// (length 0) vector.
// You can choose what to return if the street prefix passed in is an empty
// (length 0) string, but your program must not crash if street_prefix is a
// length 0 string.
// Speed Requirement --> high

//cycles through names of streets of global variable and checks until first time a street with a greater value than prefix then cycles past and checks all numbers of streets with the prefix
std::vector<StreetIdx> findStreetIdsFromPartialStreetName(std::string street_prefix){
    std::vector<StreetIdx> s_ids;
    if (street_prefix == "") {
        return s_ids;
    }
    std::string new_pre = street_prefix;
    //code generated from ChatGPT to help change the strings of the prefix and the street names as they included uppercases and spaces
    //ChatGPT, https://chatgpt.com/share/67a692c4-7280-800f-bac3-0d9bc99c3e6f, February 7 2025, Only the code for changing the strings to simpler versions without uppercases and spaces was used
    //prompt below:
    //i want to compare two strings in c++, but there might be spaces and uppercases and one of the strings might be longer than the other. what is the best way to compare these two strings, ignoring capitals and spaces, only making sure that the longer string first characters include the second strng
    new_pre.erase(std::remove_if(new_pre.begin(), new_pre.end(), isspace), new_pre.end());       
    std::transform(new_pre.begin(), new_pre.end(), new_pre.begin(), ::tolower);
    
    auto it = SimpleStreet.lower_bound(new_pre);

    while (it != SimpleStreet.end()) {
       if ((it->first).compare(0,new_pre.length(), new_pre) == 0) {
           s_ids.push_back(it->second);
           it++;
       }
       else {
           break;
       }
    }
    return s_ids;
}

double findStreetLength(StreetIdx street_id){
    return all_street_lengths[street_id];
}

LatLonBounds findStreetBoundingBox (StreetIdx street_id){
    //set min lat and long to max value 
    
    double min_latitude = std::numeric_limits<double>::infinity();
    double min_longitude = std::numeric_limits<double>::infinity();

    //set max lat and long to min values
    double max_latitude = std::numeric_limits<double>::lowest();
    double max_longitude = std::numeric_limits<double>::lowest();
    bool found_point = false;

    //go through all street segments until you find one on our street
    for(int i = 0; i < getNumStreetSegments(); i++){
        StreetSegmentInfo segment_info = getStreetSegmentInfo(i);

        if(segment_info.streetID == street_id){ //when you find one on our street get the position
            found_point = true;
            LatLon from_position = getIntersectionPosition(segment_info.from);

            //compare latitude() and longitude for the from intersection
            if(from_position.latitude() < min_latitude){min_latitude = from_position.latitude();}
            if(from_position.longitude() < min_longitude){min_longitude = from_position.longitude();}
            if(from_position.latitude() > max_latitude){max_latitude = from_position.latitude();}
            if(from_position.longitude() > max_longitude){max_longitude = from_position.longitude();}

            //compare latitude and longitude for the to intersection
            LatLon to_position = getIntersectionPosition(segment_info.to);
         
            if(to_position.latitude() < min_latitude){min_latitude = to_position.latitude();}
            if(to_position.longitude() < min_longitude){min_longitude = to_position.longitude();}
            if(to_position.latitude() > max_latitude){max_latitude = to_position.latitude();}
            if(to_position.longitude() > max_longitude){max_longitude = to_position.longitude();}

            //compare latitude and longitude for the curve points
            for(int j = 0; j < segment_info.numCurvePoints; j++){
                LatLon curve_position = getStreetSegmentCurvePoint(i, j);

                if(curve_position.latitude() < min_latitude){min_latitude = curve_position.latitude();}
                if(curve_position.longitude() < min_longitude){min_longitude = curve_position.longitude();}
                if(curve_position.latitude() > max_latitude){max_latitude = curve_position.latitude();}
                if(curve_position.longitude() > max_longitude){max_longitude = curve_position.longitude();}
            }
        }
    }

    if(!found_point){
        return {};
    }

    LatLon min_corner(min_latitude, min_longitude);
    LatLon max_corner(max_latitude, max_longitude);

    // return vector made of created objects
    return {min_corner, max_corner};

    //return {{min_latitude, min_longitude}, {max_latitude, max_longitude}};
    //return{};
}

POIIdx findClosestPOI(LatLon my_position, std::string poi_type){
    double distance = 0.0;
    //initialize smallest distance to a large number
    double smallestDistance = std::numeric_limits<double>::max();
    //set index of smallest distance to invalid number
    POIIdx indexOfSmallest = -1;

    //loop through all other POIs to compare distances
    for (int POIIndex = 0; POIIndex < getNumPointsOfInterest(); POIIndex++){

        //if POI type matches, compare distance
        if (getPOIType(POIIndex) == poi_type){
            distance = findDistanceBetweenTwoPoints(my_position, getPOIPosition(POIIndex));

            //if current distance is less than smallest distance, replace and store index
            if (distance < smallestDistance){
                smallestDistance = distance;
                indexOfSmallest = POIIndex; 
            }
        }
    }

    return indexOfSmallest;

}

std::vector<double> FeatureArea_Helper(LatLon);
//helper function for SINGULAR CONVERSION
std::vector<double> FeatureArea_Helper(LatLon point){
    double lat = point.latitude() * kDegreeToRadian;
    double lon = point.longitude() * kDegreeToRadian;

    double x = kEarthRadiusInMeters * (lon) * cos(lat);
    double y = kEarthRadiusInMeters * (lat);

    std::vector<double> xy = {x, y};
    
    return xy;
}
double findFeatureArea(FeatureIdx feature_id){
    //get number of points of the feature
    int num_points = getNumFeaturePoints(feature_id);

    LatLon first_point = getFeaturePoint(feature_id, 0);
    LatLon last_point = getFeaturePoint(feature_id, num_points - 1);

    //if the number of points is less than 3 then its not a polygon
    //also if the first point != last point then its not a closed polygon
    if(num_points < 3 || first_point.latitude() != last_point.latitude() || first_point.longitude() != last_point.longitude()){ 
        return 0.0;
    }

    //initialize area
    double area = 0.0;

    std::vector<std::pair<double, double>> xyPoints;

    for(int i = 0; i < num_points - 1; i++){
        LatLon point_1 = getFeaturePoint(feature_id, i);
        LatLon point_2 = getFeaturePoint(feature_id, i + 1);

        std::vector<double> xy_1 = FeatureArea_Helper(point_1);
        std::vector<double> xy_2 = FeatureArea_Helper(point_2);

        //area using shoelace formula
        //https://alexkritchevsky.com/2018/08/06/oriented-area.html
        area += xy_1[0] * xy_2[1] - xy_1[1] * xy_2[0];
        //std::vector<double> xyCoords = {xCoord1, yCoord1, xCoord2, yCoord2};
    }
    area = std::abs(area) / 2.0;
    return area;
}

//checks wayid exists and if it does output length from a map created in load map
double findWayLength(OSMID way_id) {
    if (OSMWayList.empty()) {
        return 0.0;
    }
        
    auto it = OSMWayList.find(way_id);
    if (it == OSMWayList.end() ) {
        return 0.0;
    }
    
    return OSMWayLength[way_id];
}

// Return the value associated with this key on the specified OSMNode.
// If this OSMNode does not exist in the current map, or the specified key is
// not set on the specified OSMNode, return an empty string.
// Speed Requirement --> high

//Found osmnodes and checked if they existed,then pulled a map of their keys and values and if their key exists output the value
//https://en.cppreference.com/w/cpp/algorithm/find
std::string getOSMNodeTagValue(OSMID osm_id, std::string key) {
    auto it = OSMNodeTags.find(osm_id);
    if (it != OSMNodeTags.end()) {
        auto itt = it->second.find(key);
        if (itt != it->second.end()) {
            return itt->second;
        }
    }
    
    return "";
}
