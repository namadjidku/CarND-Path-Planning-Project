#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;
  
  //  Stat lane (0, 1 or 2) left, middle and right
  int lane = 1;
  
  // Initial FSM state 
  string current_state = "KL";
  
  // current velocity 
  double cur_vel = 0.0;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }
    

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy, &lane, &cur_vel, 
               &current_state]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];
          
          // Helps with transitions between paths 
          int prev_size = previous_path_x.size();

          json msgJson;
          
          
          /******************************************************************************
          
          Predictions 
          
          *******************************************************************************/
          
          vector<prediction> predictions;
          
          // Loop through all available agent cars
          for (int i = 0; i < sensor_fusion.size(); i++) {
            // Check in which lane the agent car is 
            float d = sensor_fusion[i][6]; // returns the d value (fernet coordinate) of the corresponding agent car   
            double vx = sensor_fusion[i][3];
            double vy = sensor_fusion[i][4];
            double check_speed = sqrt(pow(vx,2) + pow(vy,2));
            double check_car_s = sensor_fusion[i][5]; // returns the s value (fernet coordinate) of the corresponding agent car

            // Looking at where the agent car will be in the future
            check_car_s += ((double) prev_size * 0.02 * check_speed);
            
            predictions.push_back({check_car_s, d, check_speed});  
          }
          
          
          /******************************************************************************
          
          Behavior Planning
          
          *******************************************************************************/
          
          if (prev_size > 0) {
            car_s = end_path_s;
          }
                    
          vector<string> states = successor_states(current_state, lane);
          float cost = 0;
          string optimal_state = current_state;
          trajectory final_trajectory;

          for (vector<string>::iterator it = states.begin(); it != states.end(); ++it) {
            
            vector<trajectory> trajectories = generate_trajectories(*it, predictions, lane, car_s, car_d);
            
            if (trajectories.size() != 0) {
              
              for (vector<trajectory>::iterator tt = trajectories.begin(); tt != trajectories.end(); ++tt) {
              
                float tt_cost = calculate_cost(*tt, cur_vel, lane);
                
                //cout << "cost " << tt_cost << endl;
                
                if (cost == 0 || tt_cost < cost) {
                  cost = tt_cost;
                  optimal_state = *it;
                  final_trajectory = *tt;
                }
              }
              
            }
          }
          
          
          lane = final_trajectory.lane;
          
          //cout << "lane  " <<  lane << endl;
          current_state = optimal_state;
          
          if(cur_vel > final_trajectory.lane_vel) {
            cur_vel -= .224;
          }
          else if (cur_vel < (ref_vel - 0.5)) {
            cur_vel += .224;
          } 
         
          /******************************************************************************
          
          Trajectory Generation
          
          *******************************************************************************/
          
          // Generate sparsed waypoints (ex. 30m apart) to incorporate them with a spline and 
          // fill it with more points that will help in controlling the speed of the vehicle.  
          vector <double> ptsx;
          vector <double> ptsy;
          
          // Keep track of the reference point - x,y and yaw - (either the starting point of where the vehicle is or the last point of the previous path)
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);
          
          
          // If we still have points left in the previous path, we will use the endpoint of the previous path to be the reference 
          // else If the previous path contains less then two points, we create a starting reference of the new path that is tangent to the car's angle
          
          if (prev_size >=2) { 
            // Set the reference variables to be equal to the endpoint of the previous path
            ref_x = previous_path_x[prev_size-1];
            ref_y = previous_path_y[prev_size-1];
            
            // First point (ref_x_prev, ref_y_prev)
            // Find the point prior to the one set as the reference so it can be used as to make the new path tangent to the endpoint of the previous one 
            ptsx.push_back(previous_path_x[prev_size-2]);
            ptsy.push_back(previous_path_y[prev_size-2]);
            ref_yaw = atan2(ref_y - ptsy[0], ref_x - ptsx[0]);

            // Second point
            ptsx.push_back(ref_x);
            ptsy.push_back(ref_y);              
          }
          else {
            // First point (prev_car_x, prev_car_y)
            // Create a path that is tangent to the car's angle 
            ptsx.push_back(ref_x - cos(car_yaw));
            ptsy.push_back(ref_y - sin(car_yaw));
            
            // Second point
            ptsx.push_back(ref_x);
            ptsy.push_back(ref_y);
          }
          
          
          // Define three other points in fernet equally spaced by 30m starting after the reference point           
          vector <double> next_w;
          int dist[3] = {30, 60, 90};
          
          for (int i = 0; i < 3; i++) {
            next_w = getXY(car_s + dist[i], 2 + 4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
              
            ptsx.push_back(next_w[0]);
            ptsy.push_back(next_w[1]);
          }

          
          // Transfer to the local car's coordinates for the ease of calculations (transformation and rotation to have (x, y, yaw) = (0, 0, 0))) 
          for (int i = 0; i < ptsx.size(); i++)
          {
            
            // Shift the car's refrence angle to 0 degrees
            double shift_x = ptsx[i] - ref_x;
            double shift_y = ptsy[i] - ref_y;
            
            ptsx[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
            ptsy[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
          }
            

          // Define a spline
          tk::spline s;

          // Set the set of x and y points we generated earlier to the spline
          s.set_points(ptsx,ptsy);

          // Define sets that will include the points used for the planner (path planning points)
          vector <double> next_x_vals;
          vector <double> next_y_vals;

          // If I have some left points from the previous path (the simulator didn’t go through them) - Helps in getting a smooth transition 
          for (int i = 0; i < previous_path_x.size(); i++ )
          {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }


          // Calculate how to break up spline points so that we travel at our desired reference velocity
          double target_x = 30.0;
          double target_y = s(target_x);
          double target_dist = sqrt(pow(target_x,2) + pow(target_y,2));
          double x_add_on = 0;

          // Feed more planning path points - Filling with the number of points needed to reach 50 after adding the left points from the previous path
          for (int i = 0; i <= 50 - previous_path_x.size(); i++)
          {


            double N = (target_dist / (0.02 * cur_vel / 2.24));
            double x_point = x_add_on + (target_x) / N;
            double y_point = s(x_point);

            // Update x_add_on for the next iteration
            x_add_on = x_point;

            // Variables used for shifting back to the global coordinates from the local coordinates.
            double x_ref = x_point;
            double y_ref = y_point;

            // Rotating back
            x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
            y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

            // Transitioning back
            x_point += ref_x;
            y_point += ref_y;

            // Add to the planning path points sets
            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
          }

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  
  h.run();
}