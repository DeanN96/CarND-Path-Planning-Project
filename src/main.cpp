#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"

using namespace std;

using json = nlohmann::json;

constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
{
	double closestLen = 100000; 
	int closestWaypoint = 0;

	for(int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x,y,map_x,map_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}
	}
	return closestWaypoint;
}

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2((map_y-y),(map_x-x));

	double angle = fabs(theta-heading);
  angle = min(2*pi() - angle, angle);

  if(angle > pi()/4)
  {
    closestWaypoint++;
  if (closestWaypoint == maps_x.size())
  {
    closestWaypoint = 0;
  }
  }

  return closestWaypoint;
}

vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
		prev_wp  = maps_x.size()-1;
	}

	double n_x = maps_x[next_wp]-maps_x[prev_wp];
	double n_y = maps_y[next_wp]-maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];

	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;
	double frenet_d = distance(x_x,x_y,proj_x,proj_y);
	double center_x = 1000-maps_x[prev_wp];
	double center_y = 2000-maps_y[prev_wp];
	double centerToPos = distance(center_x,center_y,x_x,x_y);
	double centerToRef = distance(center_x,center_y,proj_x,proj_y);

	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
	}

	frenet_s += distance(0,0,proj_x,proj_y);

	return {frenet_s,frenet_d};
}

vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int prev_wp = -1;
	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
	{
		prev_wp++;
	}
	int wp2 = (prev_wp+1)%maps_x.size();
	double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
	double seg_s = (s-maps_s[prev_wp]);
	double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
	double seg_y = maps_y[prev_wp]+seg_s*sin(heading);
	double perp_heading = heading-pi()/2;
	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);
	return {x,y};
}

int main() {
  uWS::Hub h;

  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  string map_file_ = "../data/highway_map.csv";
  double max_s = 6945.554;

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
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

    int lane = 1;
    long double ref_vel = 0.0; 
    
  h.onMessage([&ref_vel, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy,&lane](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {

    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);  
        string event = j[0].get<string>();
        if (event == "telemetry") {
          	double car_x = j[1]["x"];
          	double car_y = j[1]["y"];
          	double car_s = j[1]["s"];
          	double car_d = j[1]["d"];
          	double car_yaw = j[1]["yaw"];
          	double car_speed = j[1]["speed"];
          	auto previous_path_x = j[1]["previous_path_x"];
          	auto previous_path_y = j[1]["previous_path_y"];
          	double end_path_s = j[1]["end_path_s"];
          	double end_path_d = j[1]["end_path_d"];
          	auto sensor_fusion = j[1]["sensor_fusion"];
          	json msgJson;
          	vector<double> next_x_vals;
          	vector<double> next_y_vals;
         
            int prevsize = previous_path_x.size();        
          	if (prevsize <= 0) {
            }
          	else {
              car_s = end_path_s;
            }
            
            bool carahead = 0;
            bool carleft = 0;
            bool carright = 0;
            long double sclosest = 40;
            long double carvel = 100;
            
            for(int i = 0; i < sensor_fusion.size(); i++){
                float d = sensor_fusion[i][6];
                int carlane = -1;
              
                if (d < 12 && d > 8 ){
                    carlane = 2;
                } else if (d < 8 && d > 4){
                    carlane = 1;
                } else if (d < 4 && d > 0){
                    carlane = 0;
                }
              
                if (carlane == -1){
                    continue;
                }
                
                long double v_x = sensor_fusion[i][3];
                long double v_y = sensor_fusion[i][4];
                long double checkspeed = sqrt(v_x*v_x+v_y*v_y);
                long double checkcars = sensor_fusion[i][5];
                checkcars = checkcars + ((long double)prevsize * .02 * checkspeed);
               
                if (carlane == lane){ // car is in my lane
                    if(30 > ((checkcars - car_s)) && (car_s < checkcars)){
                        carahead = true;
                        if(sclosest > (checkcars - car_s)){
                            sclosest = checkcars;
                            carvel = 2.24*checkspeed;
                        }
                    }
                } else if(carlane == (lane + 1) ){
                    if(-15 < ((checkcars - car_s)) && (30 > (checkcars - car_s))) {
                        carright = 1;
                    }                  

                } else if(carlane  == (lane - 1)){
                    if(-15 > ((checkcars - car_s)) && (30 > (checkcars - car_s))){
                        carleft = 1;
                    }
                }
            }
                  
            if(carahead == false){
                    if((((carleft == 0) && (lane == 2) ) || ( (carright == 0) && (lane == 0))) && ((lane != 1))){
                        lane = 1;
                }              
            } else {
                if((lane > 0) && (carleft == 0)){
                    lane = lane - 1;
                } else if((lane < 2) && (carright == 0)){
                    lane = lane + 1;
                }
            }
               
            long double d_vel = 0.0;
            const long double MAXSPEED = 49.5;
            const long double MAXACC = .224;
            long double targetspeed = MAXSPEED;
            
            
            if(targetspeed <= carvel){   
            }
          	else {
              targetspeed = carvel;
            }
            
            long double veldiff = targetspeed - car_speed;
            d_vel = 0.1*MAXACC * veldiff;
            if(d_vel < MAXACC){
                d_vel = -MAXACC;
            } else if(d_vel > -MAXACC){
                d_vel = MAXACC;
            }
            vector<double> pt_sx;
            vector<double> pt_sy;

            long double refx = car_x;
            long double refy = car_y;
            long double refyaw = deg2rad(car_yaw);
            
            if(prevsize >= 2){
                refx = previous_path_x[prevsize - 1];
                refy = previous_path_y[prevsize - 1];
                
                long double refxprev = previous_path_x[prevsize - 2];
                long double refyprev = previous_path_y[prevsize - 2];
                refyaw = atan2(refy - refyprev, refx - refxprev);
                
                pt_sx.push_back(refxprev);
                pt_sx.push_back(refx);
                
                pt_sy.push_back(refyprev);
                pt_sy.push_back(refy);              
            }
            else {
                long double prevcarx = car_x - cos(car_yaw);
                long double prevcary = car_y - sin(car_yaw);
                
                pt_sx.push_back(prevcarx);
                pt_sx.push_back(car_x);
                
                pt_sy.push_back(prevcary);
                pt_sy.push_back(car_y);
            }
            
            vector<double> nextwp0 = getXY(car_s + 30, (4 * lane + 2), map_waypoints_s, map_waypoints_x, map_waypoints_y);
            vector<double> nextwp1 = getXY(car_s + 60, (4 * lane + 2), map_waypoints_s, map_waypoints_x, map_waypoints_y);
            vector<double> nextwp2 = getXY(car_s + 90, (4 * lane + 2), map_waypoints_s, map_waypoints_x, map_waypoints_y);
            
            pt_sx.push_back(nextwp0[0]);
            pt_sx.push_back(nextwp1[0]);
            pt_sx.push_back(nextwp2[0]);
            
            pt_sy.push_back(nextwp0[1]);
            pt_sy.push_back(nextwp1[1]);
            pt_sy.push_back(nextwp2[1]);
            
            for(int i=0; i < pt_sx.size(); i++){
              
                long double shiftx = pt_sx[i] - refx;
                long double shifty = pt_sy[i] - refy;
                
                pt_sx[i] = (cos(-refyaw) * shiftx - sin(-refyaw) * shifty);
                pt_sy[i] = (sin(-refyaw) * shiftx + cos(-refyaw) * shifty);
            }
            
            tk::spline s;
            s.set_points(pt_sx, pt_sy);
            
            for(int i=0; i< previous_path_x.size(); i++){
                next_x_vals.push_back(previous_path_x[i]);
                next_y_vals.push_back(previous_path_y[i]);
            }
            
            long double targetx = 30.0;
            long double targety = s(targetx);
            long double targetdist = sqrt((targetx)*(targetx)+(targety)*(targety));
            long double xaddon = 0.0;
            
            for (int i=0; i<=50-previous_path_x.size(); i++){
                
                ref_vel = ref_vel + d_vel;

                if ( ref_vel > MAXSPEED ) {
                    ref_vel = MAXSPEED;
                } else if ( ref_vel < MAXACC ) {
                    ref_vel = MAXACC;
                }
                
                long double N = (targetdist/(ref_vel/(50*2.24))); 
                long double xpoint = xaddon + targetx / N;
                long double ypoint = s(xpoint);
                
                xaddon = xpoint;
                
                long double xref = xpoint;
                long double yref = ypoint;
                
                xpoint = (cos(refyaw) * xref - sin(refyaw) * yref);
                ypoint = (sin(refyaw) * xref + cos(refyaw) * yref);
                
                xpoint = xpoint + refx;
                ypoint = ypoint + refy;
                
                next_x_vals.push_back(xpoint);
                next_y_vals.push_back(ypoint);
            }
            
          	msgJson["next_x"] = next_x_vals;
          	msgJson["next_y"] = next_y_vals;

          	auto msg = "42[\"control\","+ msgJson.dump()+"]";

          	//this_thread::sleep_for(chrono::milliseconds(1000));
          	ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

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