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

#include "m1.h"
#include "m2.h"
#include "m3.h"
#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>
#include <vector>
#include "ezgl/application.hpp"
#include "ezgl/graphics.hpp"
#include "ezgl/color.hpp"
#include <sstream>
#include "StreetsDatabaseAPI.h"
#include <utility>
#include <curl/curl.h>
#include "OSMDatabaseAPI.h"
#include <unordered_map>
#include <map>
#include <set>

//structs and classes defined here
struct Intersection_data{
  LatLon position;
  ezgl::point2d xy_loc = {0,0};
  std::string name;
  bool highlight = false;
};

struct POI_data{
  LatLon position;
  ezgl::point2d xy_loc = {0, 0};
  std::string name;
  std::string type;
  bool highlight = false;
};

struct Feature_data{
  //include polygon points in xy coords
    std::vector<ezgl::point2d> points; 
    std::string name;
    ezgl::point2d xy_loc = {0,0};
    FeatureType type;
    ezgl::rectangle bounding_box;
    bool highlight = false;
};

struct MapBounds{
  double min_lat;
  double max_lat;
  double min_lon;
  double max_lon;
};

struct Street_data{
  //xy points representing the street segment
  std::vector<ezgl::point2d> polyline;
  std::string street_name;
  float speedLimit;
  bool oneWay;
  ezgl::rectangle bbox;
};

struct Street_label{
  std::string name;
  ezgl::point2d label_point;
  float speedLimit;
};

struct SubwayStation{
  std::string name;
  LatLon position;
  ezgl::point2d xy_loc;
};

struct SubwayLine{
  std::string line_name;
  std::vector<ezgl::point2d> polyline;
  ezgl::rectangle bbox;  
};

struct BusRoute{
  std::vector<ezgl::point2d> polyline;
};

//declare helper functions here

//main drawing functions
void draw_main_canvas(ezgl::renderer *g);
void initial_setup(ezgl::application *application, bool new_window);
void drawPOI(ezgl::renderer *g, size_t poi_id);

void draw_feature_areas(ezgl::renderer *g, double world_width, const ezgl::rectangle& visible_world);
void draw_streets(ezgl::renderer *g, double world_width, const ezgl::rectangle& visible_world);
void draw_street_names(ezgl::renderer *g, double world_width, const ezgl::rectangle& visible_world, ezgl::canvas *c, std::vector<ezgl::rectangle> &global_label_boxes);
void draw_intersections(ezgl::renderer *g, double world_width, const ezgl::rectangle& visible_world);
void draw_subway_map(ezgl::renderer *g, double world_width, const ezgl::rectangle& visible_world, ezgl::canvas *c, std::vector<ezgl::rectangle> &global_label_boxes);
void draw_bus_map(ezgl::renderer *g, double world_width);
void draw_scale(ezgl::renderer *g, double world_width, ezgl::canvas *c);
void draw_path(ezgl::renderer *g, const std::vector<StreetSegmentIdx> &path);
void print_path_directions(const std::vector<StreetSegmentIdx>& path, IntersectionIdx start_intersection);

//precomputing helper functions
void precomputeIntersections();
void precomputeFeatures();
void precomputePOI();
void precomputeStreets();
void precomputeStreetLabels();
void precomputeSubwayStations();
void precomputeSubwayLines();
void precomputeBusRoutes();
ezgl::rectangle calculateFeatureBoundingBox(const std::vector<ezgl::point2d>& points);

//colouring helper functions
void selectFeatureColour(ezgl::renderer *g, FeatureType type);
void highlightIntersection(ezgl::application *application, double x, double y);
void unhighlightAll();

//coordinate and button functios
MapBounds calculateMinMaxLatLon();
bool shouldDrawFeature(double world_width, FeatureType type);
bool isPointInView(const ezgl::point2d& point, const ezgl::rectangle& view);
bool isObjectInView(const ezgl::rectangle& object_bbox, const ezgl::rectangle& view);
bool rectangles_intersect(const ezgl::rectangle &r1, const ezgl::rectangle &r2);
ezgl::rectangle make_label_bbox(const ezgl::point2d &base_point, const std::string &text,
                                double world_units_per_pixel,
                                double char_width_pixels,
                                double text_height_pixels);
bool is_overlapping(const ezgl::rectangle &box, const std::vector<ezgl::rectangle> &existing_boxes);
const OSMNode* findNodeByOSMID(const OSMID &target_id);

//zoom functions
void zoomFitToIntersection(ezgl::application *app, IntersectionIdx intersection_id);
void zoomFitToPath(ezgl::application *app, const std::vector<StreetSegmentIdx> &path);

//general helper functions
float x_from_lon(float lon);
float y_from_lat(float lat);
float lon_from_x(float x);
float lat_from_y(float y);
void draw_one_way_arrow(ezgl::renderer *g, const ezgl::point2d &start, const ezgl::point2d &end);

POIIdx findClosestPOIHelper(LatLon my_position);
std::string leftOrRight(StreetSegmentIdx, StreetSegmentIdx);
std::string computeBearingDirection(LatLon from, LatLon to);

//text helper functions
std::pair<std::string, std::string> splitStreetNames(const char* text, int length);
std::string getMissingStreetInputsMessage(const std::string &s1, const std::string &s2, const std::string &s3, const std::string &s4);
std::string getInvalidStreetNamesMessage(const std::vector<StreetIdx>& s1_ids, const std::vector<StreetIdx>& s2_ids,
                                          const std::vector<StreetIdx>& s3_ids, const std::vector<StreetIdx>& s4_ids);
std::string getInvalidIntersectionSelectionMessage(const std::vector<IntersectionIdx>& start_intersections, const std::vector<IntersectionIdx>& end_intersections);
void set_font_with_fallback(ezgl::renderer *g, const std::string &text, double size);

// UI callback functions
void find_intersection_button_cbk(GtkWidget */*widget*/, ezgl::application *application);
void toggle_colorful (GtkWidget* /*widget*/, ezgl::application* application);
void checklist_open (GtkWidget* /*widget*/, ezgl::application* application);
gboolean subway_switch(GtkSwitch* /*SubwaySwitch*/, gboolean switch_state, ezgl::application* application);
gboolean bus_switch(GtkSwitch* /*BusSwitch*/, gboolean switch_state, ezgl::application* application);
void change_maps (GtkWidget* /*widget*/, ezgl::application* application);
void help_button (GtkWidget* /*widget*/, ezgl::application* application);
void find_path_button_cbk(GtkWidget */*widget*/, ezgl::application *application);
void unhighlight_all_cbk(GtkWidget */*widget*/, ezgl::application *application);
void show_directions_window(const std::vector<std::pair<std::string, std::string>>& instructions);

//event callback functions
void act_on_mouse_press(ezgl::application *application, GdkEventButton *event, double x, double y);
void act_on_mouse_move(ezgl::application *application, GdkEventButton *event, double x, double y);
void act_on_key_press(ezgl::application *application, GdkEventKey *event, char *key_name);

//global variables defined here
float avg_lat = 0.0;
const float kmh_to_ms = 3.6; 
std::vector<Intersection_data> intersections;
std::vector<POI_data> points_of_interest;
std::vector<Feature_data> features;
std::vector<Street_data> streets;
std::vector<Street_label> street_labels;
std::vector<SubwayStation> subway_stations;
std::vector<SubwayLine> subway_lines;
std::vector<BusRoute> bus_routes;
std::unordered_map<std::string, ezgl::surface*> poi_surfaces;
double initialSize;
ezgl::canvas *main_canvas = nullptr;
const double MAX_ZOOM_OUT_RATIO = 2.0;
bool max_zoom_reached = false;
bool colorful = false;
ezgl::color newColor = ezgl::PINK;
bool drawSubway = false;
bool drawBus = false;
std::map<std::string, std::string> cityFile = {
    {"Toronto", "toronto_canada"}, {"Boston", "boston_usa"}, {"Beijing", "beijing_china"}, {"Tehran", "tehran_iran"}, {"Iceland", "iceland"}, {"Rio de Janeiro", "rio-de-janeiro_brazil"}, {"Golden Horseshoe", "golden-horseshoe_canada"}, {"New Delhi", "new-delhi_india"}, {"New York", "new-york_usa"}, {"London", "london_england"}, {"Beirut", "beirut_lebanon"}, {"Berlin", "berlin_germany"}, {"Cape Town", "cape-town_south-africa"}, {"Hamilton", "hamilton_canada"}, {"Hong Kong", "hong-kong_china"}, {"Interlaken", "interlaken_switzerland"}, {"Saint Helena", "saint-helena"}, {"Singapore", "singapore"}, {"Tokyo", "tokyo_japan"}
};
std::vector<StreetSegmentIdx> current_path;
const double turn_penalty = 15.0;
IntersectionIdx clicked_start = -1;
IntersectionIdx clicked_end = -1;
IntersectionIdx path_start = -1;
IntersectionIdx path_end = -1;
GtkWidget* directions_window = nullptr;

std::vector<ezgl::color> available_colours = {
  {0, 128, 0},      //green
  {255, 255, 0},    //yellow
  {128, 0, 128},    //purple
  {0, 0, 255},      //blue
  {255, 0, 0},      //red
  {255, 165, 0},    //orange
  {255, 105, 180},  //pink
  {0, 255, 255},    //cyan
  {255, 0, 255},    //magenta
  {0, 128, 128},    //teal
  {50, 205, 50},    //lime
  {139, 69, 19},    //brown
  {128, 128, 128},  //grey
  {255, 215, 0},    //gold
  {128, 0, 0},      //maroon
  {0, 0, 128},      //navy
  {128, 128, 0},    //olive
  {64, 224, 208},   //turquoise
  {238, 130, 238},  //violet
  {220, 20, 60},    //crimson
  {75, 0, 130},     //indigo
  {255, 191, 0},    //amber
  {245, 245, 220},  //beige
  {255, 127, 80},   //coral
  {230, 230, 250},  //lavender
  {54, 69, 79},     //charcoal
  {250, 128, 114},  //salmon
  {224, 17, 95},    //ruby
  {135, 206, 235},  //sky blue
  {0, 255, 127},    //spring green
  {255, 0, 127},    //rose
  {255, 218, 185},  //peach
  {127, 255, 212},  //aquamarine
  {160, 82, 45},    //sienna
  {70, 130, 180},   //steel blue
  {229, 228, 226}   //platinum
};

//map to store poi images

void drawMap() {
  //CURLcode res = curl_global_init(CURL_GLOBAL_ALL);
  ezgl::application::settings settings;
  settings.main_ui_resource = "libstreetmap/resources/main.ui";
  settings.window_identifier = "MainWindow";
  settings.canvas_identifier = "MainCanvas";

  ezgl::application application(settings);
  
  precomputeIntersections();
  precomputePOI();
  precomputeFeatures();
  precomputeStreets();
  precomputeStreetLabels();
  // precomputeSubwayStations();
  // precomputeSubwayLines();
  // precomputeBusRoutes();

  MapBounds bounds = calculateMinMaxLatLon();
  ezgl::rectangle initial_world({x_from_lon(bounds.min_lon), y_from_lat(bounds.min_lat)}, 
                                {x_from_lon(bounds.max_lon), y_from_lat(bounds.max_lat)});
  
  initialSize = initial_world.top_right().x - initial_world.top_left().x;
     
  application.add_canvas("MainCanvas", draw_main_canvas, initial_world);

  main_canvas = application.get_canvas("MainCanvas");

  //do NOT write code after this line
  application.run(initial_setup, act_on_mouse_press, 
                  act_on_mouse_move, act_on_key_press);
}

void draw_main_canvas (ezgl::renderer *g){
  g->draw_rectangle({0, 0}, {1000, 1000});
  ezgl::rectangle visible_world = g->get_visible_world();
  std::vector<ezgl::rectangle> global_label_boxes;
  
  //colorful button part
  if (colorful) {
      g->set_color(newColor);
      g->fill_rectangle(visible_world);
  }
  
  //get current visible width and height 
  double world_width = visible_world.top_right().x - visible_world.top_left().x;

  draw_feature_areas(g, world_width, visible_world);
  draw_streets(g, world_width, visible_world);

  if (!current_path.empty()) {
    draw_path(g, current_path);
  }

  draw_intersections(g, world_width, visible_world);
  draw_street_names(g, world_width, visible_world, main_canvas, global_label_boxes);

  //draw the path flags on top of the intersections and street names
  if (!current_path.empty()) {
    ezgl::point2d start_pos = intersections[path_start].xy_loc;
    ezgl::surface *start_flag = ezgl::renderer::load_png("libstreetmap/resources/start_flag.png");
    g->draw_surface(start_flag, start_pos);
    ezgl::renderer::free_surface(start_flag);

    ezgl::point2d end_pos = intersections[path_end].xy_loc;
    ezgl::surface *finish_flag = ezgl::renderer::load_png("libstreetmap/resources/finish_flag.png");
    g->draw_surface(finish_flag, end_pos);
    ezgl::renderer::free_surface(finish_flag);
  }

  //draw scale (size is constant for now)
  if (main_canvas != nullptr) {
    draw_scale(g, world_width, main_canvas);
  }

  //drawing the poi as images - please leave this loop here
  for (size_t poi_id = 0; poi_id < points_of_interest.size(); ++poi_id){
    if (isPointInView(points_of_interest[poi_id].xy_loc, visible_world)){
      if(world_width / initialSize < 0.009){
        drawPOI(g, poi_id);
      }
    }
  }
  draw_subway_map(g, world_width, visible_world, main_canvas, global_label_boxes);
  draw_bus_map(g, world_width);
}

void initial_setup(ezgl::application *application, bool /*new_window*/)
{
  // status bar message
  application->update_message("EZGL Application");
  application->create_button("Find Intersections", 7, find_intersection_button_cbk);

  //colorful button for bonus feature
  application->create_button ("Color", 10, toggle_colorful);  
  
  GObject* SubwaySwitch = application->get_object("SubwaySwitch");
  g_signal_connect (SubwaySwitch, "state-set", G_CALLBACK(subway_switch), application);
  
  GObject* BusSwitch = application->get_object("BusSwitch");
  g_signal_connect (BusSwitch, "state-set", G_CALLBACK(bus_switch), application);
  
  application->create_button ("Change Maps", 14, change_maps);  

  application->create_button ("Help", 16, help_button); 
  
  application->create_button("Find Fastest Path", 17, find_path_button_cbk);

  application->create_button("Unhighlight All", 20, unhighlight_all_cbk);

}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//drawing functions!!

//drawing the feature areas
void draw_feature_areas(ezgl::renderer *g, double world_width, const ezgl::rectangle& visible_world){
  //drawing the feature areas
  for (size_t feature_id = 0; feature_id < features.size(); ++feature_id){
    //check to see if the feature is in the current frame
    if (isObjectInView(features[feature_id].bounding_box, visible_world)){
      selectFeatureColour(g, features[feature_id].type);
    
      //check to see if its a line or a point - not a polygon
      if (features[feature_id].points.size() == 2) {
          if(shouldDrawFeature(world_width, features[feature_id].type)){
            double feature_line_width = 7;
            g->set_line_width(feature_line_width);
            g->draw_line(features[feature_id].points[0], features[feature_id].points[1]);
          }
      }

      else if (features[feature_id].points.size() == 1) {
          if(shouldDrawFeature(world_width, features[feature_id].type)){
            float radius = 5;
            ezgl::point2d center = features[feature_id].xy_loc;
            g->fill_arc(center, radius, 0, 360);
          }
      }

      //draw the polygon
      else{
        if(shouldDrawFeature(world_width, features[feature_id].type)){
          g->fill_poly(features[feature_id].points);
        }
      }
    }
  }
}

void draw_streets(ezgl::renderer *g, double world_width, const ezgl::rectangle& visible_world){
  //drawing streets and one-way arrows

  //determine line width for roads based on zoom level
  double minor_line_width = 2;
  double major_line_width = 0;
  
  //zoomed out a lot
  if ((world_width / initialSize) >= 1) {major_line_width = 2;}
  
  //zoomed out a little 
  else if ((world_width / initialSize) >= 0.5) {major_line_width = 3;} 
  
  //zoomed in
  else {major_line_width = 5;}
  
  //iterate over each precomputed street.
  for (size_t i = 0; i < streets.size(); ++i) {
    const Street_data &s = streets[i];

    if (!rectangles_intersect(visible_world, s.bbox)) {
      continue;
    }

    //major street have speedlimit over 70km/h = 70/3.6 = 19.44m/s
    if (s.speedLimit >= (70 / kmh_to_ms)) {
      g->set_color(246, 207, 101);
      g->set_line_width(major_line_width);
    } 
    //medium roads have speed limit btwn 50 and 70 kmh
    else if((s.speedLimit >= (50 / kmh_to_ms)) && (s.speedLimit < (70 / kmh_to_ms ))){
      if ((world_width / initialSize) >= 0.4) {
        continue;
      }
      g->set_color(211, 211, 211);
      g->set_line_width(minor_line_width);
    }
    else{
      //dont draw minor street if zoomed out
      if ((world_width / initialSize) >= 0.1) {
        continue;
      }
      g->set_color(211, 211, 211);
      g->set_line_width(minor_line_width);
    }

    for (size_t j = 0; j < s.polyline.size() - 1; ++j){
      g->draw_line(s.polyline[j], s.polyline[j+1]);
    }
  
    //draw arrows for one-way streets when zoomed in
    if ((world_width / initialSize) <= 0.02){
      if (s.oneWay && s.polyline.size() >= 2) {
        //use first two polyline points to find arrow direction
        draw_one_way_arrow(g, s.polyline.front(), s.polyline[1]);
      }
    }
  }
}

void draw_street_names(ezgl::renderer *g, double world_width, const ezgl::rectangle &visible_world, ezgl::canvas *c, std::vector<ezgl::rectangle> &global_label_boxes) {
    const int canvas_width = c->width();
    double world_units_per_pixel = world_width / static_cast<double>(canvas_width);
    
    //estimated text dimensions in pixels
    const double char_width_pixels = 8;
    const double text_height_pixels = 14;
    
    for (size_t i = 0; i < street_labels.size(); ++i) {
        const auto &sl = street_labels[i];
        
        //draw if shown in window
        if (!visible_world.contains(sl.label_point)){
          continue;
        }
        
        ezgl::rectangle label_box = make_label_bbox(sl.label_point, sl.name,
                                                    world_units_per_pixel, 
                                                    char_width_pixels, 
                                                    text_height_pixels);
                                                    
        if (is_overlapping(label_box, global_label_boxes)){
            continue;
        }

        //draw label based on zoom threshold and speed limit so reduce clutter
        if((sl.speedLimit > (85 / kmh_to_ms)) && (sl.speedLimit <= (100 / kmh_to_ms))){
          if ((world_width / initialSize) >= 0.61) {
            continue;
          }
        }
        if((sl.speedLimit > (70 / kmh_to_ms)) && (sl.speedLimit <= (85 / kmh_to_ms))){
          if ((world_width / initialSize) >= 0.37) {
            continue;
          }
        }
        if((sl.speedLimit > (60 / kmh_to_ms)) && (sl.speedLimit <= (70 / kmh_to_ms))){
          if ((world_width / initialSize) >= 0.08) {
            continue;
          }
        }
        if((sl.speedLimit > (40 / kmh_to_ms)) && (sl.speedLimit <= (60 / kmh_to_ms))){
          if ((world_width / initialSize) >= 0.05) {
            continue;
          }
        }
        if (sl.speedLimit <= (40 / kmh_to_ms)){
          if ((world_width / initialSize) >= 0.02) {
            continue;
          }
        }

        //set name font and colour (format font for non-latin characters)
        set_font_with_fallback(g, sl.name, 10.0);
        g->set_color(61,77,98);
        
        // Draw the street name at the label position in WORLD coordinates.
        g->draw_text(sl.label_point, sl.name);
        
        // Save the label's bounding box so that future labels can avoid overlapping.
        global_label_boxes.push_back(label_box);
    }
}

void draw_subway_map(ezgl::renderer *g, double world_width, const ezgl::rectangle& visible_world, ezgl::canvas *c, std::vector<ezgl::rectangle> &global_label_boxes) {
    if (drawSubway) {
        g->set_line_width(3);
        g->set_color(ezgl::BLACK);
        size_t line_count = subway_lines.size();

        //draw subway lines
        for (size_t i = 0; i < subway_lines.size(); i++) {
          const SubwayLine &line = subway_lines[i];
          if (line.polyline.size() < 2){
            continue;
          }

          //dont draw if out of frame
          if (!rectangles_intersect(visible_world, line.bbox)){
            continue;
          }

          //assign a color for this line based solely on its index. pattern goes like this: 1,2,3,4,4,3,2,1. this accounts for both directions of line
          size_t colour_index = 0;
          if (i < line_count / 2) {
            colour_index = i % available_colours.size();
          } 
          //assign colour for lines going the other direction
          else {
            colour_index = (line_count - 1 - i) % available_colours.size();
          }

          g->set_color(available_colours[colour_index]);

          //for each subway line, draw each segment of the polyline
          for (size_t j = 0; j < line.polyline.size() - 1; j++) {
            g->draw_line(line.polyline[j], line.polyline[j+1]);
          }
        }

        const int canvas_width = c->width();
        double world_units_per_pixel = world_width / static_cast<double>(canvas_width);
        const double char_width_pixels = 8;
        const double text_height_pixels = 14;

        //draw subway stations
        for (size_t i = 0; i < subway_stations.size(); i++) {
          const SubwayStation &station = subway_stations[i];
          if (!visible_world.contains(station.xy_loc)){
            continue;
          }
          //draw orange circle for station
          g->set_color(255, 165, 0);
          g->fill_arc(station.xy_loc, 5, 0, 360);

          ezgl::point2d label_position = station.xy_loc + ezgl::point2d{world_units_per_pixel, world_units_per_pixel};

          // Create bounding box for this label.
          ezgl::rectangle label_box = make_label_bbox(label_position, station.name, world_units_per_pixel, char_width_pixels, text_height_pixels);

          // Check for overlap with previously drawn labels.
          if (is_overlapping(label_box, global_label_boxes)){
              continue;
          }

          //draw station name
          set_font_with_fallback(g, station.name, 10.0);
          g->set_color(ezgl::BLACK);
          g->draw_text(label_position, station.name);
          global_label_boxes.push_back(label_box);
        }
    }
}   

void draw_bus_map(ezgl::renderer *g, double world_width) {
    if (drawBus) {
        g->set_line_width(2);

        for (size_t i = 0; i < bus_routes.size(); ++i) {
          const BusRoute &route = bus_routes[i];

          //show if zoomed in enough
          if ((world_width / initialSize) >= 0.16) {
            continue;
          }

          //draw if there are at least two points
          if (route.polyline.size() < 2){
            continue;
          }

          size_t colour_index = 0;
          colour_index = i % available_colours.size();
          g->set_color(available_colours[colour_index]);

          for (size_t j = 0; j < route.polyline.size() - 1; ++j) {
            g->draw_line(route.polyline[j], route.polyline[j+1]);
          }
        }
    }
}

void draw_intersections(ezgl::renderer *g, double world_width, const ezgl::rectangle& visible_world){
  //drawing the intersections and highlighting them if necessary
  for (size_t inter_id = 0; inter_id < intersections.size(); ++inter_id){
    if (isPointInView(intersections[inter_id].xy_loc, visible_world)){
      if(intersections[inter_id].highlight){g->set_color(ezgl::RED);}
      else{g->set_color(99, 99, 99);}

      //set size 
      float width = 4;
      float height = width;
      ezgl::point2d inter_loc = intersections[inter_id].xy_loc -
                                ezgl::point2d{width / 2, height / 2};
      
      if(world_width / initialSize < 0.01){
        g->fill_rectangle(inter_loc, width, height);
      }
    }
  }
}

void drawPOI(ezgl::renderer *g, size_t poi_id){
  //create the surface 
  ezgl::point2d poi_location = points_of_interest[poi_id].xy_loc;
  
  //load image depending on type
  if(points_of_interest[poi_id].type == "restaurant"){
    ezgl::surface *png_surface = ezgl::renderer::load_png("libstreetmap/resources/restaurant_resize.png");
    g->draw_surface(png_surface, poi_location);
    ezgl::renderer::free_surface(png_surface);
  }

  else if(points_of_interest[poi_id].type == "doctors" ||
        points_of_interest[poi_id].type == "hospital" ||
        points_of_interest[poi_id].type == "emergency_room" ){
    ezgl::surface *png_surface = ezgl::renderer::load_png("libstreetmap/resources/hospital_resize.png");
    g->draw_surface(png_surface, poi_location);
    ezgl::renderer::free_surface(png_surface);
  }

  else if(points_of_interest[poi_id].type == "bicycle_rental"){
    ezgl::surface *png_surface = ezgl::renderer::load_png("libstreetmap/resources/bicycle2_resize.png");
    g->draw_surface(png_surface, poi_location);
    ezgl::renderer::free_surface(png_surface);
  }

  else if(points_of_interest[poi_id].type == "vending_machine"){
    ezgl::surface *png_surface = ezgl::renderer::load_png("libstreetmap/resources/snack.png");
    g->draw_surface(png_surface, poi_location);
    ezgl::renderer::free_surface(png_surface);
  }

  else if(points_of_interest[poi_id].type == "bus_station"){
    ezgl::surface *png_surface = ezgl::renderer::load_png("libstreetmap/resources/bus.png");
    g->draw_surface(png_surface, poi_location);
    ezgl::renderer::free_surface(png_surface);
  }

  else if(points_of_interest[poi_id].type == "internet_cafe"){
    ezgl::surface *png_surface = ezgl::renderer::load_png("libstreetmap/resources/gaming.png");
    g->draw_surface(png_surface, poi_location);
    ezgl::renderer::free_surface(png_surface);
  }

  else{
    g->set_color(ezgl::color(206, 151, 240));
    float radius = 4;
    ezgl::point2d center = points_of_interest[poi_id].xy_loc;
    g->fill_arc(center, radius, 0, 360);
  }
}

void draw_scale(ezgl::renderer *g, double world_width, ezgl::canvas *c) {
    //switch to SCREEN coordinates so that the scale bar is fixed
    g->set_coordinate_system(ezgl::SCREEN);

    const int margin = 20;
    const int scale_bar_length_pixels = 100;
    
    //get the current canvas width
    double canvas_width = c->width();
    double canvas_height = c->height();
    
    //compute the conversion factor (world units per pixel)
    double world_units_per_pixel = world_width / static_cast<double>(canvas_width);
    
    //calculate the world distance corresponding to our fixed pixel scale bar
    double scale_distance = scale_bar_length_pixels * world_units_per_pixel;
    
    //define the start and end points of the scale bar in SCREEN coordinates
    ezgl::point2d scale_end = { canvas_width - margin, canvas_height - margin };
    ezgl::point2d scale_start = { (canvas_width - margin) - scale_bar_length_pixels, canvas_height - margin };
    
    //draw the scale
    g->set_color(ezgl::BLACK);
    g->set_line_width(2);
    g->draw_line(scale_start, scale_end);
    
    //create label for scale
    std::ostringstream oss;
    oss << scale_distance << " m";
    ezgl::point2d text_position = { (canvas_width - margin) - (scale_bar_length_pixels / 2.0), (canvas_height - margin) + 10 };
    g->set_font_size(12);
    g->draw_text(text_position, oss.str());
    
    //switch back to WORLD coordinates.
    g->set_coordinate_system(ezgl::WORLD);
}

void draw_path(ezgl::renderer *g, const std::vector<StreetSegmentIdx> &path) {
  g->set_color(0, 0, 255, 200); //set highlight colour r,g,b,a (transparency goes from 0 to 255)
  g->set_line_width(4); //make the path not completely cover street

  for (size_t i = 0; i < path.size(); ++i) {
    StreetSegmentInfo seg_info = getStreetSegmentInfo(path[i]);

    //get the intersections at both ends of the segment
    LatLon from_pos = getIntersectionPosition(seg_info.from);
    LatLon to_pos   = getIntersectionPosition(seg_info.to);

    //convert lat/lon to screen coordinates
    ezgl::point2d from_xy = { x_from_lon(from_pos.longitude()), y_from_lat(from_pos.latitude()) };
    ezgl::point2d to_xy   = { x_from_lon(to_pos.longitude()), y_from_lat(to_pos.latitude()) };

    //draw curve points if any
    if (seg_info.numCurvePoints > 0) {
      ezgl::point2d prev_xy = from_xy;
      for (int j = 0; j < seg_info.numCurvePoints; ++j) {
        LatLon curve_pos = getStreetSegmentCurvePoint(path[i], j);
        ezgl::point2d curve_xy = { x_from_lon(curve_pos.longitude()), y_from_lat(curve_pos.latitude()) };
        g->draw_line(prev_xy, curve_xy);
        prev_xy = curve_xy;
      }
      g->draw_line(prev_xy, to_xy); //draws a line from the final curve point to to
    }
    else{
      g->draw_line(from_xy, to_xy);
    }
  }
}

void zoomFitToPath(ezgl::application *app, const std::vector<StreetSegmentIdx> &path){
  if (path.empty()) {return;}

  double leftmost_x = std::numeric_limits<double>::max();
  double rightmost_x = std::numeric_limits<double>::lowest();
  double min_y = std::numeric_limits<double>::max();
  double max_y = std::numeric_limits<double>::lowest();

  for (const auto & seg_id : path){
    StreetSegmentInfo seg_info = getStreetSegmentInfo(seg_id);

    LatLon from_pos = getIntersectionPosition(seg_info.from);
    LatLon to_pos = getIntersectionPosition(seg_info.to);

    double from_x = x_from_lon(from_pos.longitude());
    double from_y = y_from_lat(from_pos.latitude());
    double to_x = x_from_lon(to_pos.longitude());
    double to_y = y_from_lat(to_pos.latitude());

    leftmost_x = std::min(leftmost_x, std::min(from_x, to_x));
    rightmost_x = std::max(rightmost_x, std::max(from_x, to_x));
    min_y = std::min(min_y, std::min(from_y, to_y));
    max_y = std::max(max_y, std::max(from_y, to_y));
  }

  ezgl::canvas *canvas = app->get_canvas("MainCanvas");
  ezgl::rectangle world = canvas->get_camera().get_world();
  double canvas_aspect_ratio = world.width() / world.height(); 

  double path_width = rightmost_x - leftmost_x;
  double path_height = max_y - min_y;
  double zoom_width = path_width;
  double zoom_height = path_height;

  if ((path_width / path_height) > canvas_aspect_ratio){
    zoom_height = path_width / canvas_aspect_ratio;
  } 
  else{
    zoom_width = path_height * canvas_aspect_ratio;
  }

  //10% padding to the zoom
  double padding_ratio = 0.1;
  double padding_x = zoom_width * padding_ratio;
  double padding_y = zoom_height * padding_ratio;

  zoom_width += 2 * padding_x;
  zoom_height += 2 * padding_y;

  double center_x = (leftmost_x + rightmost_x) / 2.0;
  double center_y = (min_y + max_y) / 2.0;

  ezgl::rectangle new_visible_world(
  ezgl::point2d(center_x - zoom_width / 2, center_y - zoom_height / 2),
  ezgl::point2d(center_x + zoom_width / 2, center_y + zoom_height / 2));

  canvas->get_camera().set_world(new_visible_world);
  app->refresh_drawing();
}

void print_path_directions(const std::vector<StreetSegmentIdx>& path, IntersectionIdx start_intersection){
  std::stringstream printPath;

  if (path.empty()){
    return;
  } 

  std::vector<std::pair<std::string, std::string>> instructions;

  //determine direction from start_intersection to the next intersection
  StreetSegmentInfo seg_info = getStreetSegmentInfo(path[0]);
  IntersectionIdx next_intersection;
  if (seg_info.from == start_intersection){
    next_intersection = seg_info.to;
  } 
  else{
    next_intersection = seg_info.from;
  }

  LatLon startLatLon = getIntersectionPosition(start_intersection);
  LatLon nextLatLon = getIntersectionPosition(next_intersection);
  
  std::string dir = computeBearingDirection(startLatLon, nextLatLon);

  double cumulative_distance = findStreetSegmentLength(path[0]);
  std::string current_street = getStreetName(seg_info.streetID);
  instructions.emplace_back("straight_arrow.png", "Start heading " + dir + " on " + current_street);

  for (size_t i = 1; i < path.size(); ++i) {
    StreetSegmentInfo prev_seg = getStreetSegmentInfo(path[i - 1]);
    StreetSegmentInfo curr_seg = getStreetSegmentInfo(path[i]);

    std::string prev_street = getStreetName(prev_seg.streetID);
    std::string curr_street = getStreetName(curr_seg.streetID);

    if (curr_street == prev_street) {
      //still on the same street, so add this segment's length.
      cumulative_distance += findStreetSegmentLength(path[i]);
    }
    //if we turned to different street
    if (curr_street != prev_street) {
      std::string distance_str;
      if (cumulative_distance < 1000.0) {
        distance_str = std::to_string((int)cumulative_distance) + "m";
      } 
      else {
        double km = cumulative_distance / 1000.0;
        std::stringstream ss;
        //std::fixed makes the precision apply to digits after the decimal, instead of total significant digits
        ss << std::fixed << std::setprecision(2) << km << "km";
        distance_str = ss.str();
      }

      std::string turn = leftOrRight(path[i - 1], path[i]);
      std::string icon;
      if (turn == "left") icon = "turn_left_arrow.png";
      else if (turn == "right") icon = "turn_right_arrow.png";
      else icon = "straight_arrow.png";

      instructions.emplace_back(icon, "Turn " + turn + " onto " + curr_street + " in " + distance_str);

      //reset culmuative distance for the new street block
      cumulative_distance = findStreetSegmentLength(path[i]);
      current_street = curr_street;
    }
  }

  std::string final_distance_str;
  if (cumulative_distance < 1000.0) {
    final_distance_str = std::to_string((int)cumulative_distance) + "m";
  } 
  else {
    double km = cumulative_distance / 1000.0;
    std::stringstream ss;
    ss << std::fixed << std::setprecision(2) << km << "km";
    final_distance_str = ss.str();
  }
  instructions.emplace_back("destination_flag.png", "Travel " + final_distance_str + " and you will have arrived at your destination.");

  show_directions_window(instructions);
}

//Bloor Street East & Ted Rogers Way
//Charles Street West & Balmuto Street 

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//button functions!

void unhighlight_all_cbk(GtkWidget */*widget*/, ezgl::application *application) {
  unhighlightAll();
  current_path.clear();
  application->refresh_drawing();
  clicked_start = -1;
  clicked_end = -1;
}

void find_path_button_cbk(GtkWidget */*widget*/, ezgl::application *application) {
  GObject* gtk_object1 = application->get_object("PointOne");
  GObject* gtk_object2 = application->get_object("PointTwo");

  GtkEntry* gtk_entry1 = GTK_ENTRY(gtk_object1);
  GtkEntry* gtk_entry2 = GTK_ENTRY(gtk_object2);

  const gchar* text1 = gtk_entry_get_text(gtk_entry1);
  const gchar* text2 = gtk_entry_get_text(gtk_entry2);
  const guint16 length1 = gtk_entry_get_text_length(gtk_entry1);
  const guint16 length2 = gtk_entry_get_text_length(gtk_entry2);

  //separate text into separate street names
  std::pair<std::string, std::string> split1 = splitStreetNames(text1, length1);
  std::pair<std::string, std::string> split2 = splitStreetNames(text2, length2);

  std::string street1 = split1.first;
  std::string street2 = split1.second;
  std::string street3 = split2.first;
  std::string street4 = split2.second;

  //if a street name is missing
  std::string missingMsg = getMissingStreetInputsMessage(street1, street2, street3, street4);
  if (!missingMsg.empty()) {
    application->update_message(missingMsg);
    return;
  }

  //retrieve street IDs from names
  std::vector<StreetIdx> street1_ids = findStreetIdsFromPartialStreetName(street1);
  std::vector<StreetIdx> street2_ids = findStreetIdsFromPartialStreetName(street2);
  std::vector<StreetIdx> street3_ids = findStreetIdsFromPartialStreetName(street3);
  std::vector<StreetIdx> street4_ids = findStreetIdsFromPartialStreetName(street4);

  //if street name is invalid
  std::string invalidNames = getInvalidStreetNamesMessage(street1_ids, street2_ids, street3_ids, street4_ids);
  if (!invalidNames.empty()) {
    application->update_message(invalidNames);
    return;
  }

  //select the first matching street ID (assuming no ambiguity)
  std::vector<IntersectionIdx> start_intersections = findIntersectionsOfTwoStreets({street1_ids[0], street2_ids[0]});
  std::vector<IntersectionIdx> end_intersections = findIntersectionsOfTwoStreets({street3_ids[0], street4_ids[0]});

  //if intersection is invalid
  std::string invalidInts = getInvalidIntersectionSelectionMessage(start_intersections, end_intersections);
  if (!invalidInts.empty()) {
    application->update_message(invalidInts);
    return;
  }
  
  if (start_intersections == end_intersections) {
      application->update_message("Identical intersections given");
      current_path.clear();
      application->refresh_drawing();
      return;
  }

  //select the first available intersections for now
  IntersectionIdx start_intersection = start_intersections[0];
  IntersectionIdx end_intersection = end_intersections[0];

  ezgl::point2d start_pos = intersections[start_intersection].xy_loc;
  ezgl::point2d end_pos = intersections[end_intersection].xy_loc;

  unhighlightAll();
  highlightIntersection(application, start_pos.x, start_pos.y);
  highlightIntersection(application, end_pos.x, end_pos.y);

  std::stringstream msgStream;
  msgStream << "Path selected: " << intersections[start_intersection].name 
            << " -> " << intersections[end_intersection].name;
  application->update_message(msgStream.str());

  //call A* pathfinding
  current_path = findPathBetweenIntersections(turn_penalty, {start_intersection, end_intersection});

  if (current_path.empty()) {
    application->update_message("No path found between selected intersections");
    return;
  }

  zoomFitToPath(application, current_path);
  print_path_directions(current_path, start_intersection);

  //store intersection idx for flags
  path_start = start_intersection;
  path_end = end_intersection;
  application->refresh_drawing();

  //if user clicked one intersection, then typed path, they need to re-click. (increases usability)
  clicked_start = -1;
  clicked_end = -1;

  return;
}

void help_button (GtkWidget* /*widget*/, ezgl::application* application) {
    std::stringstream title;
    std::stringstream help;

    title << "Guide:";
    
    help << "To find the intersections of two streets, put the street names into the search bar and press \"Find Intersections\"" 
            << "\n" << "To change the color of the map, type in the color you'd like to change to into the bar, and press \"Color\""
            << "\n" << "To display the transit lines, flip the switches of either the \"Subway Display\" or the \"Bus Display\""
            << "\n" << "To change the map, find the new location in the drop-down bar and press \"Change Maps\""
            << "\n" << "To find the fastest path between two intersections, type the street names in the format \"Street 1 & Street 2\" or \"Street 1 and Street 2\" into the boxes and press \"Find Fastest Path\"";
  
    application->create_popup_message(title.str().c_str(), help.str().c_str());
    application->refresh_drawing();
}

void find_intersection_button_cbk(GtkWidget */*widget*/, ezgl::application *application) {
  GObject* gtk_object1 = application->get_object("IntersectionOne");
  GObject* gtk_object2 = application->get_object("IntersectionTwo");

  GtkEntry* gtk_entry1 = GTK_ENTRY(gtk_object1);
  GtkEntry* gtk_entry2 = GTK_ENTRY(gtk_object2);
  
  const gchar* text1 = gtk_entry_get_text(gtk_entry1);
  const gchar* text2 = gtk_entry_get_text(gtk_entry2);
  const guint16 length1 = gtk_entry_get_text_length(gtk_entry1);
  const guint16 length2 = gtk_entry_get_text_length(gtk_entry2);
  
  std::string street1;
  std::string street2;
  
  for (int i = 0; i < length1; i++) {
      street1 = street1 + text1[i];
  }
  
  for (int i = 0; i < length2; i++) {
      street2 = street2 + text2[i];
  }
  if (street1 == "") {
      if (street2 == "") {
          application->update_message("No streets given");
          return;
      }
      std::vector<StreetIdx> s2O = findStreetIdsFromPartialStreetName(street2);
      if (s2O.size() == 0) {
          application->update_message("Streets do not exist/Non-unique street name");
          return;
      }
      StreetIdx s2 = s2O[0];
      std::vector<IntersectionIdx> intersectionOfS2 = findIntersectionsOfStreet(s2);
      std::string title = "Intersections: " + street2;
      std::stringstream coordinates;

      if (!intersectionOfS2.empty()) {
        zoomFitToIntersection(application, intersectionOfS2[0]);
      }
              
      for (int i = 0; i < intersectionOfS2.size(); i++) {
        ezgl::point2d intersection_pos = intersections[intersectionOfS2[i]].xy_loc;
        highlightIntersection(application, intersection_pos.x, intersection_pos.y);
        application->refresh_drawing();
      
        coordinates << "x: " << std::to_string(intersection_pos.x) << " y: " << std::to_string(intersection_pos.y) << "\n";
      }
      application->create_popup_message(title.c_str(), coordinates.str().c_str());
  
      application->refresh_drawing();
      return;
  }
  
  if (street2 == "") {
      std::vector<StreetIdx> s1O = findStreetIdsFromPartialStreetName(street1);
      
      if (s1O.size() == 0) {
          application->update_message("Streets do not exist/Non-unique street name");
          return;
      }
      
      StreetIdx s1 = s1O[0];
      std::vector<IntersectionIdx> intersectionOfS1 = findIntersectionsOfStreet(s1);
      std::string title = "Intersections: " + street1;
      std::stringstream coordinates;

      if (!intersectionOfS1.empty()) {
        zoomFitToIntersection(application, intersectionOfS1[0]);
      }
  
      for (int i = 0; i < intersectionOfS1.size(); i++) {
        ezgl::point2d intersection_pos = intersections[intersectionOfS1[i]].xy_loc;
        highlightIntersection(application, intersection_pos.x, intersection_pos.y);
        application->refresh_drawing();
      
        coordinates << "x: " << std::to_string(intersection_pos.x) << " y: " << std::to_string(intersection_pos.y) << "\n";
      }
      application->create_popup_message(title.c_str(), coordinates.str().c_str());
  
      application->refresh_drawing();
      return;
  }
  
  std::vector<StreetIdx> s1O = findStreetIdsFromPartialStreetName(street1);
  std::vector<StreetIdx> s2O = findStreetIdsFromPartialStreetName(street2);
      
  if (s1O.size() == 0 || s2O.size() == 0) {
    application->update_message("Streets do not exist/Non-unique street name");
    return;
  }
  StreetIdx s1 = s1O[0];
  StreetIdx s2 = s2O[0];

  auto namePair = std::make_pair(s1, s2);
  std::vector<IntersectionIdx> intersectionIndexes = findIntersectionsOfTwoStreets(namePair);
  std::string title = "Intersections: " + street1 + " & " + street2;
  std::stringstream coordinates;
  
  if (intersectionIndexes.empty()) {
    application->update_message("Streets do not exist/Non-unique street name");
    return;
  }
  
  zoomFitToIntersection(application, intersectionIndexes[0]);  
  unhighlightAll();

  for (int i = 0; i < intersectionIndexes.size(); i++) {
      ezgl::point2d intersection_pos = intersections[intersectionIndexes[i]].xy_loc;
      highlightIntersection(application, intersection_pos.x, intersection_pos.y);
      application->refresh_drawing();
      
      coordinates << "x: " << std::to_string(intersection_pos.x) << " y: " << std::to_string(intersection_pos.y) << "\n";
  }
  
  application->create_popup_message(title.c_str(), coordinates.str().c_str());
  application->refresh_drawing();
}

void show_directions_window(const std::vector<std::pair<std::string, std::string>>& instructions) {
  //close any pre-existing directions window
  if (GTK_IS_WIDGET(directions_window)) {
    gtk_widget_destroy(directions_window);
    directions_window = nullptr;
  }

  //create new window
  directions_window = gtk_window_new(GTK_WINDOW_TOPLEVEL);
  gtk_window_set_title(GTK_WINDOW(directions_window), "Path Directions");
  gtk_window_set_default_size(GTK_WINDOW(directions_window), 450, 300);

  //keeps the window always on top
  gtk_window_set_keep_above(GTK_WINDOW(directions_window), TRUE);

  //add scrollable container
  GtkWidget *scrolled_window = gtk_scrolled_window_new(NULL, NULL);
  gtk_container_add(GTK_CONTAINER(directions_window), scrolled_window);

  //box to hold instructions
  GtkWidget *vbox = gtk_box_new(GTK_ORIENTATION_VERTICAL, 5);
  gtk_container_add(GTK_CONTAINER(scrolled_window), vbox);

  for (size_t i = 0; i < instructions.size(); ++i) {
    const std::string &image_name = instructions[i].first;
    const std::string &text = instructions[i].second;

    GtkWidget *hbox = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 5);
    GtkWidget *image = gtk_image_new_from_file(("libstreetmap/resources/" + image_name).c_str());
    GtkWidget *label = gtk_label_new(text.c_str());
    gtk_label_set_xalign(GTK_LABEL(label), 0); //align text to left

    gtk_box_pack_start(GTK_BOX(hbox), image, FALSE, FALSE, 5);
    gtk_box_pack_start(GTK_BOX(hbox), label, TRUE, TRUE, 5); //allowed to expand horizontally
    gtk_box_pack_start(GTK_BOX(vbox), hbox, FALSE, FALSE, 5);
  }

  //display complete UI window
  gtk_widget_show_all(directions_window);
}

void zoomFitToIntersection(ezgl::application *app, IntersectionIdx intersection_id){
  LatLon position = getIntersectionPosition(intersection_id);

  double pos_x = x_from_lon(position.longitude());
  double pos_y = y_from_lat(position.latitude());

  double zoom_width = 100;
  double zoom_height = 77;

  ezgl::rectangle new_visible_world(
    ezgl::point2d(pos_x - zoom_width, pos_y - zoom_height), 
    ezgl::point2d(pos_x + zoom_width, pos_y + zoom_height));

  
  app->get_canvas("MainCanvas")->get_camera().set_world(new_visible_world);
  app->refresh_drawing();
}

void act_on_mouse_press(ezgl::application *application, GdkEventButton *event, double x, double y){
  application->update_message("Mouse Clicked");

  if ((event->state & GDK_CONTROL_MASK) && (event->state & GDK_SHIFT_MASK))
    std::cout << "with control and shift pressed ";
  else if (event->state & GDK_CONTROL_MASK)
    std::cout << "with control pressed ";
  else if (event->state & GDK_SHIFT_MASK)
    std::cout << "with shift pressed ";

  LatLon pos = LatLon(lat_from_y(y), lon_from_x(x));
  
  //for clicking on intersection
  int inter_id = findClosestIntersection(pos);
  ezgl::point2d intersection_pos = intersections[inter_id].xy_loc;
  float width = 4;
  float height = width;

  //check to see if you actually clicked on the rectangle that the intersection is on 
  if (x >= intersection_pos.x - width / 2 && x <= intersection_pos.x + width / 2 &&
      y >= intersection_pos.y - height / 2 && y <= intersection_pos.y + height / 2){
    

    if (clicked_start == -1 && !intersections[inter_id].highlight){
      //only highlight the first intersection clicked
      clicked_start = inter_id;
      unhighlightAll();
      highlightIntersection(application, intersections[clicked_start].xy_loc.x, intersections[clicked_start].xy_loc.y);
      application->update_message("First intersection selected: " + intersections[inter_id].name);
    } 
    else if (clicked_start != -1){
      clicked_end = inter_id;

      //if same intersection is pressed again, give error and clear old path
      if (clicked_start == clicked_end) {
        application->update_message("Identical intersections given");

        current_path.clear();
        application->refresh_drawing();

        clicked_start = -1;
        clicked_end = -1;
        return;
      }

      //only highlight the 2 intersections used for the path
      unhighlightAll();
      highlightIntersection(application, intersections[clicked_start].xy_loc.x, intersections[clicked_start].xy_loc.y);
      highlightIntersection(application, intersections[clicked_end].xy_loc.x, intersections[clicked_end].xy_loc.y);
      
      std::stringstream msgStream;
      msgStream << "Path selected: " << intersections[clicked_start].name 
                << " -> " << intersections[clicked_end].name;
      application->update_message(msgStream.str());

      //compute and store the path
      current_path = findPathBetweenIntersections(turn_penalty, {clicked_start, clicked_end});

      if (current_path.empty()) {
        application->update_message("No path found between selected intersections");
        clicked_start = -1;
        clicked_end = -1;
        return;
      }

      zoomFitToPath(application, current_path);
      print_path_directions(current_path, clicked_start);

      //store path start and end index for drawing flags
      path_start = clicked_start;
      path_end = clicked_end;
      //reset selection
      clicked_start = -1;
      clicked_end = -1;
    }
    else{
      //toggles intersection to be unhighlighted
      highlightIntersection(application, x, y);
    }
  }
  
  //for clicking on a poi
  POIIdx poi_id = findClosestPOIHelper(pos);
  ezgl::point2d poi_pos = points_of_interest[poi_id].xy_loc;
  //check to see if you actually clicked on the rectangle that the poi is on 
  if (x >= poi_pos.x - width / 2 && x <= poi_pos.x + width / 2 &&
      y >= poi_pos.y - height / 2 && y <= poi_pos.y + height / 2){
      std::string poi_output = "POI Name: " + points_of_interest[poi_id].name + ", Type: " + points_of_interest[poi_id].type;
      application->update_message(poi_output.c_str());
  }

  //refresh at the end to see instant results
  application->refresh_drawing();
}

void act_on_mouse_move(ezgl::application */*application*/, GdkEventButton */*event*/, double x, double y){
  //std::cout << "Mouse move at coordinates (" << x << "," << y << ") "<< std::endl;
  (void)x;
  (void)y;
}

void act_on_key_press(ezgl::application *application, GdkEventKey */*event*/, char *key_name){
  application->update_message("Key Pressed");
  std::cout << key_name <<" key is pressed" << std::endl;
}

gboolean subway_switch(GtkSwitch* /*SubwaySwitch*/, gboolean switch_state, ezgl::application* application) {
    drawSubway = switch_state;
    application->refresh_drawing();
    
    return false;
}

gboolean bus_switch(GtkSwitch* /*BusSwitch*/, gboolean switch_state, ezgl::application* application) {
    drawBus = switch_state;
    application->refresh_drawing();
    
    return false;
}

void change_maps (GtkWidget* /*widget*/, ezgl::application* application) {
  GObject* city = application->get_object("cities");
  if (gtk_combo_box_text_get_active_text (GTK_COMBO_BOX_TEXT(city)) == NULL) {
      application->update_message("Please enter a city");
      return;
  }
  std::string currentCity = gtk_combo_box_text_get_active_text (GTK_COMBO_BOX_TEXT(city));

  std::string fileCity = cityFile[currentCity];
  std::string newFile = "/cad2/ece297s/public/maps/" + fileCity + ".streets.bin";
  application->update_message("Loading " + currentCity + "...");

  //close directions window
  if (GTK_IS_WIDGET(directions_window)) {
    gtk_widget_destroy(directions_window);
    directions_window = nullptr;
  }

  closeMap();
  intersections.clear();
  points_of_interest.clear();
  features.clear();
  streets.clear();
  street_labels.clear();
  subway_stations.clear();
  subway_lines.clear();
  bus_routes.clear();
  poi_surfaces.clear();
  current_path.clear();
  
  bool load_success = loadMap(newFile);
  if (!load_success) {
      application->update_message("Failed to load map for " + currentCity);
      return;
  }

  precomputeIntersections();
  precomputePOI();
  precomputeFeatures();
  precomputeStreets();
  precomputeStreetLabels();
  // precomputeSubwayStations();
  // precomputeSubwayLines();
  // precomputeBusRoutes();

  MapBounds bounds = calculateMinMaxLatLon();
  ezgl::rectangle new_world({x_from_lon(bounds.min_lon), y_from_lat(bounds.min_lat)}, 
                              {x_from_lon(bounds.max_lon), y_from_lat(bounds.max_lat)});
  
  initialSize = new_world.top_right().x - new_world.top_left().x;
  
  application->get_renderer()->set_visible_world(new_world);
  
  application->refresh_drawing();
  application->update_message(currentCity + " loaded successfully!");
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// helper functions: e.g coordinate conversion

std::string leftOrRight(StreetSegmentIdx from_seg, StreetSegmentIdx to_seg) {
  //get from and to points for the 'from' segment
  StreetSegmentInfo from_info = getStreetSegmentInfo(from_seg);
  ezgl::point2d from_start = {x_from_lon(getIntersectionPosition(from_info.from).longitude()),
                              y_from_lat(getIntersectionPosition(from_info.from).latitude())};
  ezgl::point2d from_end = {x_from_lon(getIntersectionPosition(from_info.to).longitude()),
                            y_from_lat(getIntersectionPosition(from_info.to).latitude())};

  //get from and to points for the 'to' segment
  StreetSegmentInfo to_info = getStreetSegmentInfo(to_seg);
  ezgl::point2d to_start = {x_from_lon(getIntersectionPosition(to_info.from).longitude()),
                            y_from_lat(getIntersectionPosition(to_info.from).latitude())};
  ezgl::point2d to_end = {x_from_lon(getIntersectionPosition(to_info.to).longitude()),
                          y_from_lat(getIntersectionPosition(to_info.to).latitude())};

  //define common pivot intersection point
  ezgl::point2d pivot;
  if (from_info.to == to_info.from || from_info.to == to_info.to) {
    pivot = from_end;
    if (from_info.to == to_info.to){
      std::swap(to_start, to_end);
    }
  } 
  else {
    pivot = from_start;
    if (from_info.from == to_info.from){
      std::swap(to_start, to_end);
    }
  }

  //create vectors from pivot to the other points
  double vec1_x = from_end.x - from_start.x;
  double vec1_y = from_end.y - from_start.y;
  double vec2_x = to_end.x - to_start.x;
  double vec2_y = to_end.y - to_start.y;

  //compute cross product
  double cross = vec1_x * vec2_y - vec1_y * vec2_x;

  //determine turn direction
  if (cross > 0) {
    return "left";
  } 
  else {
    return "right";
  }
}

std::string computeBearingDirection(LatLon from, LatLon to) {
  double lat1 = from.latitude() * kDegreeToRadian;
  double lat2 = to.latitude() * kDegreeToRadian;
  double delta_lon = (to.longitude() - from.longitude()) * kDegreeToRadian;

  double y = sin(delta_lon) * cos(lat2);
  double x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(delta_lon);

  double bearing = atan2(y, x) * 180.0 / M_PI;
  bearing = fmod((bearing + 360.0), 360.0); //normalize to 0-360 degrees

  if (bearing >= 337.5 || bearing < 22.5) return "north";
  else if (bearing >= 22.5 && bearing < 67.5) return "northeast";
  else if (bearing >= 67.5 && bearing < 112.5) return "east";
  else if (bearing >= 112.5 && bearing < 157.5) return "southeast";
  else if (bearing >= 157.5 && bearing < 202.5) return "south";
  else if (bearing >= 202.5 && bearing < 247.5) return "southwest";
  else if (bearing >= 247.5 && bearing < 292.5) return "west";
  else return "northwest";
}

void draw_one_way_arrow(ezgl::renderer *g, const ezgl::point2d &start, const ezgl::point2d &end) {
  //calculate the direction of segment
  double dx = end.x - start.x;
  double dy = end.y - start.y;
  double angle = std::atan2(dy, dx);
  
  //parameters for arrow
  const double body_length = 20.0;
  const double head_length = 10.0;
  const double head_width = 5.0;
  
  //where arrow tip is
  ezgl::point2d tip = {start.x + body_length * std::cos(angle),
                       start.y + body_length * std::sin(angle)};
  
  //draw arrow body
  g->set_line_width(2);
  g->set_color(ezgl::RED);
  g->draw_line(start, tip);
  
  //where base of arrow head is
  ezgl::point2d base_center = {tip.x - head_length * std::cos(angle),
                               tip.y - head_length * std::sin(angle)};
  
  //find perpendicular unit vector for the base
  double perp_x = -std::sin(angle);
  double perp_y = std::cos(angle);
  double half_width = head_width / 2.0;
  
  //find the base corners of the arrowhead
  ezgl::point2d base_left = {base_center.x + half_width * perp_x,
                             base_center.y + half_width * perp_y};
  ezgl::point2d base_right = {base_center.x - half_width * perp_x,
                              base_center.y - half_width * perp_y};
  
  //create and draw the arrowhead triangle.
  std::vector<ezgl::point2d> arrow_head = {tip, base_left, base_right};
  g->fill_poly(arrow_head);

}

//highlight, colouring, and draw based on zoom functions

void selectFeatureColour(ezgl::renderer *g, FeatureType type){
  //select the colour of the feature based on feature type

    //normal green
    if (type == GREENSPACE){g->set_color(195, 236, 178);}
    
    //light green
    else if(type == PARK) {g->set_color(195, 236, 178);}
    
    //dark green
    else if (type == GOLFCOURSE) {g->set_color(195, 236, 178);} 
    
    //lightish blue
    else if (type == LAKE) {g->set_color(110, 185, 250);} 

    //darker blue
    else if (type == RIVER) {g->set_color(0, 97, 194);}

    //light blue
    else if (type == STREAM) {g->set_color(195, 236, 178);} 

    //super light blue
    else if (type == GLACIER) {g->set_color(158, 234, 255);}

    //purplely
    else if (type == BUILDING) {g->set_color(232, 232, 232);}

    //orangey-yellow
    else if (type == BEACH) {g->set_color(250, 231, 187);}

    //light beige
    else if (type == ISLAND) {g->set_color(195, 236, 178);}
    
    //unknown -> make it grey 
    else {g->set_color(88, 85, 89);}
}

bool shouldDrawFeature(double world_width, FeatureType type){
  if((world_width / initialSize) < 0.08 
    && type == BUILDING){return true;}
  
  else if((world_width / initialSize) <= 2 
    && type == LAKE){return true;}
  
  else if((world_width / initialSize) <= 2 
    && type == BEACH){return true;}

  else if((world_width / initialSize) <= 2 
    && type == ISLAND){return true;}
  
  else if((world_width / initialSize) <= 2 
    && type == GLACIER){return true;}
  
  else if((world_width / initialSize) <= 2 
    && type == STREAM){return true;}
  
  else if((world_width / initialSize) <= 2 
    && type == RIVER){return true;}
  
  else if((world_width / initialSize) <= 2 
    && type == GOLFCOURSE){return true;}
  
  else if((world_width / initialSize) <= 2 
    && type == PARK){return true;}
  
  else if((world_width / initialSize) <= 2 
    && type == GREENSPACE){return true;}
  
  else{return false;}
}

void highlightIntersection(ezgl::application *application, double x, double y){
    //convert coords to latlon
    LatLon pos = LatLon(lat_from_y(y), lon_from_x(x));
    int inter_id = findClosestIntersection(pos);

    //keep highlight dimensions the same
    float width = 4;
    float height = width;
    
    //render the highlight
    ezgl::renderer *g = application->get_renderer();
    g->set_color(ezgl::RED);
    g->fill_rectangle(intersections[inter_id].xy_loc, width, height);

    //toggle highlight state
    intersections[inter_id].highlight = !intersections[inter_id].highlight;

    std::stringstream ss;
    ss<<"Intersection: " << intersections[inter_id].name;
  
  application->update_message(ss.str());
}

void unhighlightAll() {
    for (int i = 0; i < getNumIntersections(); i++) {
        intersections[i].highlight = false;
    }
}

//precomputing functions

void precomputeIntersections(){
  //for drawing intersections
  intersections.resize(getNumIntersections());

  for (int id = 0; id < getNumIntersections(); ++id){
    intersections[id].position = getIntersectionPosition(id);
    intersections[id].name = getIntersectionName(id);
  }
  
  //get lat lon positions
  MapBounds bounds = calculateMinMaxLatLon();
  avg_lat = (bounds.min_lat + bounds.max_lat) / 2;
  
  for (int id = 0; id<getNumIntersections(); ++id){
    intersections[id].xy_loc.x = x_from_lon(intersections[id].position.longitude());
    intersections[id].xy_loc.y = y_from_lat(intersections[id].position.latitude());
  }
}

void precomputePOI(){
  points_of_interest.resize(getNumPointsOfInterest());
  for (int id = 0; id < getNumPointsOfInterest(); ++id){
    points_of_interest[id].position = getPOIPosition(id);
    points_of_interest[id].type = getPOIType(id);
    points_of_interest[id].name = getPOIName(id);
    points_of_interest[id].xy_loc.x = x_from_lon(points_of_interest[id].position.longitude());
    points_of_interest[id].xy_loc.y = y_from_lat(points_of_interest[id].position.latitude());
  }
}


void precomputeFeatures(){
  features.resize(getNumFeatures());

  for (int id = 0; id < getNumFeatures(); ++id){
    features[id].name = getFeatureName(id);
    features[id].type = getFeatureType(id);

    //convert LatLon points of each polygon to ezgl::point2d
    int numPoints = getNumFeaturePoints(id);
    //reserve eliminates reallocations, each copying hundreeds of elements
    features[id].points.reserve(numPoints);

    for (int pointNum = 0; pointNum < numPoints; ++pointNum){
      LatLon point = getFeaturePoint(id, pointNum);
      float x = x_from_lon(point.longitude());
      float y = y_from_lat(point.latitude());
      //include the x y coordinates in the feature data
      features[id].points.push_back({x, y});
    }

    //precompute bounding box
    features[id].bounding_box = calculateFeatureBoundingBox(features[id].points);
  }
}

void precomputeStreets() {
  streets.clear();
  streets.reserve(getNumStreetSegments());

  for (StreetSegmentIdx seg_id = 0; seg_id < getNumStreetSegments(); ++seg_id) {

    StreetSegmentInfo seg_info = getStreetSegmentInfo(seg_id);
    Street_data s;
    s.street_name = getStreetName(seg_info.streetID);
    s.speedLimit = seg_info.speedLimit;
    s.oneWay = seg_info.oneWay;

    LatLon from_pos = getIntersectionPosition(seg_info.from);
    LatLon to_pos   = getIntersectionPosition(seg_info.to);
    double from_x = x_from_lon(from_pos.longitude());
    double from_y = y_from_lat(from_pos.latitude());
    double to_x   = x_from_lon(to_pos.longitude());
    double to_y   = y_from_lat(to_pos.latitude());
    
    //reserve space for the polyline (start, curve points, end)
    s.polyline.reserve(seg_info.numCurvePoints + 2);

    //build polyline for segment (polyline is the part of the streetseg from 1 curvepoint to the next)
    s.polyline.push_back({from_x, from_y});
    
    for (int i = 0; i < seg_info.numCurvePoints; ++i) {
      LatLon curve_pt = getStreetSegmentCurvePoint(seg_id, i);
      double curve_x = x_from_lon(curve_pt.longitude());
      double curve_y = y_from_lat(curve_pt.latitude());
      s.polyline.push_back({curve_x, curve_y});
    }
    s.polyline.push_back({to_x, to_y});

    //precompute the bounding box for the polyline.
    double min_x = std::numeric_limits<double>::max();
    double max_x = std::numeric_limits<double>::lowest();
    double min_y = std::numeric_limits<double>::max();
    double max_y = std::numeric_limits<double>::lowest();
    for (size_t i = 0; i < s.polyline.size(); ++i) {
      double x = s.polyline[i].x;
      double y = s.polyline[i].y;
      if (x < min_x) { min_x = x; }
      if (x > max_x) { max_x = x; }
      if (y < min_y) { min_y = y; }
      if (y > max_y) { max_y = y; }
    }
    s.bbox = ezgl::rectangle({min_x, min_y}, {max_x, max_y});
    
    streets.push_back(s);
  }
}

void precomputeStreetLabels() {
  //use maps to group label points and speed limits by street name.
  std::unordered_map<std::string, std::vector<ezgl::point2d>> points_by_street;
  std::unordered_map<std::string, float> speed_by_street;

  //iterate over each precomputed street seg
  for (size_t i = 0; i < streets.size(); ++i) {
    const auto &s = streets[i];

    if (s.street_name.empty() || s.street_name == "<unknown>" || s.polyline.size() < 2) {
      continue;
    }
    
    ezgl::point2d midpoint = s.polyline[s.polyline.size() / 2];

    //group the midpoint by street name.
    points_by_street[s.street_name].push_back(midpoint);

    //update speed limit for the street (using the maximum value (some streets have multiple speed limits))
    if (speed_by_street.find(s.street_name) == speed_by_street.end()) {
      speed_by_street[s.street_name] = s.speedLimit;
    } 
    else {
      if (s.speedLimit > speed_by_street[s.street_name]){
        speed_by_street[s.street_name] = s.speedLimit;
      }
    }
  }

  street_labels.clear();

  //decide where to draw street name
  auto it = points_by_street.begin();
  for (size_t i = 0; i < points_by_street.size(); ++i, ++it) {
    const auto &entry = *it;
    const std::string &street_name = entry.first;
    const std::vector<ezgl::point2d> &points = entry.second;
    
    //use the middle street seg of the street to draw from
    size_t idx = points.size() / 2;
    ezgl::point2d label_point = points[idx];
    
    Street_label sl;
    sl.name = street_name;
    sl.label_point = label_point;
    sl.speedLimit = speed_by_street[street_name];
    
    street_labels.push_back(sl);
  }
}

void precomputeSubwayStations() {
  subway_stations.clear();
  int num_nodes = getNumberOfNodes();
  
  subway_stations.reserve(num_nodes / 1000);

  for (int i = 0; i < num_nodes; ++i) {
    const OSMNode* node = getNodeByIndex(i);
    if (!node) continue;
    
    bool is_subway_station = false;
    std::string station_name = "";
    int tagCount = getTagCount(node);

    for (int j = 0; j < tagCount; ++j) {
        std::pair<std::string, std::string> tagPair = getTagPair(node, j);
        if ((tagPair.first == "station" && tagPair.second == "subway")) {
            is_subway_station = true;
        }
        if (tagPair.first == "name") {
            station_name = tagPair.second;
        }
    }
    
    if (is_subway_station){
      SubwayStation s;
      if (station_name.empty()){
        s.name = "Unnamed Station";
      } 
      else{
        s.name = station_name;
      }
      s.position = getNodeCoords(node);
      s.xy_loc = { x_from_lon(s.position.longitude()),
                    y_from_lat(s.position.latitude()) };
      subway_stations.push_back(s);
      
    }
  }
}

void precomputeSubwayLines() {
  subway_lines.clear();
  for (int i = 0; i < getNumberOfRelations(); ++i) {
    const OSMRelation* relation = getRelationByIndex(i);
    if (!relation){
      continue;
    }
    bool is_subway_line = false;
    std::string line_name = "";
    int tagCount = getTagCount(relation);

    for (int j = 0; j < tagCount; ++j) {
      std::pair<std::string, std::string> tagPair = getTagPair(relation, j);
      //subway lines osm pair is route=subway.
      if (tagPair.first == "route" && tagPair.second == "subway") {
        is_subway_line = true;
      }
      if (tagPair.first == "name") {
        line_name = tagPair.second;
      }
    }
    
    if (!is_subway_line){
      continue;
    }
            
    SubwayLine line;
    if (line_name.empty()) {
      line.line_name = "Unnamed Line";
    } 
    else {
      line.line_name = line_name;
    }
    //get members of the relation
    std::vector<TypedOSMID> members = getRelationMembers(relation);
    line.polyline.reserve(members.size());

    for (size_t k = 0; k < members.size(); ++k) {
      //check if node
      if (members[k].type() == TypedOSMID::Node) {
        //use the helper function to find the node by OSMID
        const OSMNode* node = findNodeByOSMID(static_cast<OSMID>(members[k]));
        if (node) {
          LatLon pos = getNodeCoords(node);
          ezgl::point2d xy = { x_from_lon(pos.longitude()),
                                y_from_lat(pos.latitude()) };
          line.polyline.push_back(xy);
        }
      }
    }
    //add line if there's at least two points
    if (line.polyline.size() < 2) {
      continue;
    }

    //precompute the bounding box using an index-based loop.
    double min_x = std::numeric_limits<double>::max();
    double max_x = std::numeric_limits<double>::lowest();
    double min_y = std::numeric_limits<double>::max();
    double max_y = std::numeric_limits<double>::lowest();
    
    for (size_t k = 0; k < line.polyline.size(); k++) {
      double x = line.polyline[k].x;
      double y = line.polyline[k].y;
      if (x < min_x) { min_x = x; }
      if (x > max_x) { max_x = x; }
      if (y < min_y) { min_y = y; }
      if (y > max_y) { max_y = y; }
    }
    line.bbox = ezgl::rectangle({min_x, min_y}, {max_x, max_y});

    subway_lines.push_back(line);

  }
}

void precomputeBusRoutes() {
  bus_routes.clear();
  int num_relations = getNumberOfRelations();

  for (int i = 0; i < num_relations; ++i) {
    const OSMRelation* relation = getRelationByIndex(i);
    if (!relation) continue;

    bool is_bus_route = false;
    int tagCount = getTagCount(relation);

    for (int j = 0; j < tagCount; ++j) {
      std::pair<std::string, std::string> tagPair = getTagPair(relation, j);
      if (tagPair.first == "route" && tagPair.second == "bus") {
        is_bus_route = true;
      }
    }

    if (!is_bus_route) {
      continue;
    }

    BusRoute route;

    //get the members of the relation
    std::vector<TypedOSMID> members = getRelationMembers(relation);
    route.polyline.reserve(members.size());

    for (size_t k = 0; k < members.size(); ++k) {
      //check if the member is a node.
      if (members[k].type() == TypedOSMID::Node) {
        const OSMNode* node = findNodeByOSMID(static_cast<OSMID>(members[k]));
        if (node) {
          LatLon pos = getNodeCoords(node);
          ezgl::point2d xy = { x_from_lon(pos.longitude()),
                                y_from_lat(pos.latitude()) };
          route.polyline.push_back(xy);
        }
      }
    }
    //add routes with at least two points
    if (route.polyline.size() >= 2) {
        bus_routes.push_back(route);
    }
  }
}

//coordinate and bounds functions - also draw based on coordinates (whether its in the current frame)

float x_from_lon(float lon){
  return (lon * kDegreeToRadian * kEarthRadiusInMeters * std::cos (avg_lat * kDegreeToRadian));
}

float y_from_lat(float lat){
  return (lat * kDegreeToRadian * kEarthRadiusInMeters);
}

float lon_from_x(float x){
  return (x/(kDegreeToRadian * kEarthRadiusInMeters * std::cos (avg_lat * kDegreeToRadian)));
}

float lat_from_y(float y){
  return (y/(kDegreeToRadian * kEarthRadiusInMeters));
}

MapBounds calculateMinMaxLatLon(){
  //get the bounds of the map using intersection positions.
  double max_lat = getIntersectionPosition(0).latitude();
  double min_lat = max_lat;
  double max_lon = getIntersectionPosition(0).longitude();
  double min_lon = max_lon;
  
  //get lat lon positions
  for (int id = 0; id<getNumIntersections(); ++id){
    intersections[id].position = getIntersectionPosition(id);

    max_lat = std::max(max_lat, intersections[id].position.latitude());
    min_lat = std::min(min_lat, intersections[id].position.latitude());
    max_lon = std::max(max_lon, intersections[id].position.longitude());
    min_lon = std::min(min_lon, intersections[id].position.longitude());
  }

  return {min_lat, max_lat, min_lon, max_lon};
}

ezgl::rectangle calculateFeatureBoundingBox(const std::vector<ezgl::point2d>& points){
  if (points.empty()){
      return ezgl::rectangle({0, 0}, {0, 0});
  }
  
  //set min and max to first point
  double min_x = points[0].x;
  double max_x = points[0].x;
  double min_y = points[0].y;
  double max_y = points[0].y;
  
  //find min and max points
  for (const auto& point : points){
      min_x = std::min(min_x, point.x);
      max_x = std::max(max_x, point.x);
      min_y = std::min(min_y, point.y);
      max_y = std::max(max_y, point.y);
  }

  //return bounding box
  return ezgl::rectangle({min_x, min_y}, {max_x, max_y});
}

bool isPointInView(const ezgl::point2d& point, const ezgl::rectangle& view){
  return (point.x >= view.left() && point.x <= view.right() &&
          point.y >= view.bottom() && point.y <= view.top());
}

bool isObjectInView(const ezgl::rectangle& object_bbox, const ezgl::rectangle& view){
  return !(object_bbox.right() < view.left() || object_bbox.left() > view.right() ||
           object_bbox.top() < view.bottom() || object_bbox.bottom() > view.top());
}

bool rectangles_intersect(const ezgl::rectangle &r1, const ezgl::rectangle &r2){
  ezgl::point2d r1_bl = r1.bottom_left();
  ezgl::point2d r1_tr = r1.top_right();
  ezgl::point2d r2_bl = r2.bottom_left();
  ezgl::point2d r2_tr = r2.top_right();

  if (r1_tr.x < r2_bl.x || r1_bl.x > r2_tr.x ||
      r1_tr.y < r2_bl.y || r1_bl.y > r2_tr.y){
    return false;
  }
  return true;
}

ezgl::rectangle make_label_bbox(const ezgl::point2d &base_point, const std::string &text,
                                double world_units_per_pixel,
                                double char_width_pixels,
                                double text_height_pixels) {

  double width_world = (char_width_pixels * text.length()) * world_units_per_pixel;
  double height_world = text_height_pixels * world_units_per_pixel;

  return ezgl::rectangle(base_point, {base_point.x + width_world, base_point.y + height_world});
}

bool is_overlapping(const ezgl::rectangle &box, const std::vector<ezgl::rectangle> &existing_boxes) {
  for (size_t i = 0; i < existing_boxes.size(); ++i) {
    if (rectangles_intersect(box, existing_boxes[i])) {
      return true;
    }
  }
  return false;
}

const OSMNode* findNodeByOSMID(const OSMID &target_id) {
    int numNodes = getNumberOfNodes();
    for (int i = 0; i < numNodes; i++) {
        const OSMNode* node = getNodeByIndex(i);
        if (node && node->id() == target_id) { 
            return node;
        }
    }
    return nullptr; 
}

POIIdx findClosestPOIHelper(LatLon my_position){
    double distance = 0.0;
    //initialize smallest distance to a large number
    double smallestDistance = std::numeric_limits<double>::max();
    //set index of smallest distance to invalid number
    POIIdx indexOfSmallest = -1;

    //loop through all other POIs to compare distances
    for (int POIIndex = 0; POIIndex < getNumPointsOfInterest(); POIIndex++){
        distance = findDistanceBetweenTwoPoints(my_position, getPOIPosition(POIIndex));

            //if current distance is less than smallest distance, replace and store index
        if (distance < smallestDistance){
            smallestDistance = distance;
            indexOfSmallest = POIIndex; 
        }
    }

    return indexOfSmallest;

}

void toggle_colorful (GtkWidget* /*widget*/, ezgl::application* application) {
    GObject* gtk_object1 = application->get_object("ColorText");
  
    GtkEntry* gtk_entry1 = GTK_ENTRY(gtk_object1);
  
    const gchar* text1 = gtk_entry_get_text(gtk_entry1);
    const guint16 length1 = gtk_entry_get_text_length(gtk_entry1);
  
    std::string text;
  
    for (int i = 0; i < length1; i++) {
          text = text + text1[i];
    }
    
    if (text == "Black" || text == "black") {
        newColor = ezgl::BLACK;
        colorful = !colorful;
    }
    
    else if (text == "Grey" || text == "grey") {
        newColor = ezgl::GREY_55;
        colorful = !colorful;
    }

    else if (text == "Red" || text == "red") {
        newColor = ezgl::color(156, 64, 64);
        colorful = !colorful;
    }
    
    else if (text == "Orange" || text == "orange") {
        newColor = ezgl::color(242, 165, 102);
        colorful = !colorful;
    }
    
    else if (text == "Yellow" || text == "yellow") {
        newColor = ezgl::color(252, 226, 119);
        colorful = !colorful;
    }

    else if (text == "Green" || text == "green") {
        newColor = ezgl::color(153, 237, 111);
        colorful = !colorful;
    }
    
    else if (text == "Cyan" || text == "cyan") {
        newColor = ezgl::color(105, 245, 189);
        colorful = !colorful;
    }
    
    else if (text == "Blue" || text == "blue") {
        newColor = ezgl::color(105, 184, 245);
        colorful = !colorful;
    }
    
    else if (text == "Purple" || text == "purple") {
        newColor = ezgl::color(182, 148, 255);
        colorful = !colorful;
    }
    
    else if (text == "Pink" || text == "pink") {
        newColor = ezgl::color(240, 151, 210);
        colorful = !colorful;
    }
    
    else {
        colorful = !colorful;
    }
    
    application->refresh_drawing();
}

//text helper functions
std::pair<std::string, std::string> splitStreetNames(const char* text, int length) {
  //indicates whether we've passed the delimiter
  bool passed = false; 
  std::string first;
  std::string second;

  // Use an index-based loop
  for (int i = 0; i < length; i++) {
    //check for delimiter " & "
    if (i + 2 < length && text[i] == ' ' && text[i+1] == '&' && text[i+2] == ' ') {
      passed = true;
      i += 2; //skip over the delimiter
      continue;
    }
    //check for delimiter " and "
    if (i + 4 < length &&
      text[i] == ' ' &&
      text[i+1] == 'a' &&
      text[i+2] == 'n' &&
      text[i+3] == 'd' &&
      text[i+4] == ' ') {
      passed = true;
      i += 4; //skip over the delimiter
      continue;
    }
    if (!passed)
      first.push_back(text[i]);
    else
      second.push_back(text[i]);
  }

  return std::make_pair(first, second);
}

std::string getMissingStreetInputsMessage(const std::string &s1, const std::string &s2,
                                            const std::string &s3, const std::string &s4) {
  std::vector<std::pair<std::string, std::string>> inputs;
  inputs.push_back(std::make_pair(std::string("1"), s1));
  inputs.push_back(std::make_pair(std::string("2"), s2));
  inputs.push_back(std::make_pair(std::string("3"), s3));
  inputs.push_back(std::make_pair(std::string("4"), s4));

  std::string message;
  for (size_t i = 0; i < inputs.size(); i++) {
    if (inputs[i].second.empty()) {
      if (!message.empty()) {
        message += ", ";
      }
      message += inputs[i].first;
    }
  }
  if (!message.empty()) {
    message = "Missing input for street: " + message;
  }
  return message;
}

std::string getInvalidStreetNamesMessage(const std::vector<StreetIdx>& s1_ids,
                                          const std::vector<StreetIdx>& s2_ids,
                                          const std::vector<StreetIdx>& s3_ids,
                                          const std::vector<StreetIdx>& s4_ids) {
  std::vector<std::pair<std::string, bool>> inputs;

  inputs.push_back(std::make_pair("1", s1_ids.empty()));
  inputs.push_back(std::make_pair("2", s2_ids.empty()));
  inputs.push_back(std::make_pair("3", s3_ids.empty()));
  inputs.push_back(std::make_pair("4", s4_ids.empty()));

  std::string message;
  for (size_t i = 0; i < inputs.size(); i++) {
    if (inputs[i].second) {
      if (!message.empty()) {
        message += ", ";
      }
      message += inputs[i].first;
    }
  }
  if (!message.empty()) {
    message = "Invalid names for street: " + message;
  }
  return message;
}

std::string getInvalidIntersectionSelectionMessage(const std::vector<IntersectionIdx>& start_intersections,
                                                     const std::vector<IntersectionIdx>& end_intersections) {
  std::vector<std::string> invalid;
  if (start_intersections.empty()) {
    invalid.push_back("Streets 1&2");
  }
  if (end_intersections.empty()) {
    invalid.push_back("Streets 3&4");
  }
  std::string message;
  for (size_t i = 0; i < invalid.size(); i++) {
    if (!message.empty()) {
      message += " and ";
    }
    message += invalid[i];
  }
  if (!message.empty()) {
    message = "Invalid intersection selection/Non-unique street name for: " + message;
  }
  return message;
}

void set_font_with_fallback(ezgl::renderer *g, const std::string &text, double size = 10.0) {
  //iterate through the each char and pick font based on unicode range
  for (char c : text) {
    unsigned char uc = static_cast<unsigned char>(c);

    //arabic
    if (uc >= 0xD8 && uc <= 0xDB) {
      g->format_font("Noto Sans Arabic", ezgl::font_slant::normal, ezgl::font_weight::normal, size);
      return;
    }

    //chinese
    if (uc >= 0xE4 && uc <= 0xE9) {
      g->format_font("Noto Sans CJK SC", ezgl::font_slant::normal, ezgl::font_weight::normal, size);
      return;
    }

    //japanese
    if (uc >= 0xE3) {
      g->format_font("Noto Sans CJK JP", ezgl::font_slant::normal, ezgl::font_weight::normal, size);
      return;
    }

    //korean
    if (uc >= 0xEC && uc <= 0xED) {
      g->format_font("Noto Sans CJK KR", ezgl::font_slant::normal, ezgl::font_weight::normal, size);
      return;
    }
  }

  //default font for latin-based languages
  g->format_font("Sans", ezgl::font_slant::normal, ezgl::font_weight::normal, size);
}