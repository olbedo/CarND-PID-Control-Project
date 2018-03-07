#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include "Twiddle.h"
#include <math.h>
#include <fstream>      // std::ifstream, std::ofstream

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main()
{
  uWS::Hub h;

  // TODO: Initialize the pid variable.
  // number of iterations between coefficient updates
  unsigned int num_iter = 4500;
  // coefficients for PID controllers
  std::vector<double> coeffs = {0.23, 0.002, 6.1, 1.9, 0.0, 0.00};
  // increments for coefficient updates
  std::vector<double> incr = {0.01, 0.0001, 0.5, 0.1, 0.0, 0.0001};

  // create PID controller for steering value
  PID pid;
  pid.Init(coeffs[0], coeffs[1], coeffs[2], num_iter);

  // create PID controller for the throttle value
  PID pid_throttle;
  pid_throttle.Init(coeffs[3], coeffs[4], coeffs[5], num_iter);
  
  // Ziegler-Nichols method
  //unsigned int counter = 0;
  //std::ofstream outfile;
  //outfile.open ("out.csv");
  //outfile << "Kp=" << pid.coeffs[0] << "\tKi=" << pid.coeffs[1] << "\tKd=" << pid.coeffs[2] << std::endl;
  //outfile << "Counter" << "\t" << "CTE" << "\t" << "Speed" << "\t" << "Steering Angle" << std::endl;

  // create instance for parameter tuning
  //Twiddle twiddle;
  //twiddle.Init(coeffs, incr, 4, num_iter);

  //h.onMessage([&pid,&outfile,&counter](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
  //h.onMessage([&pid, &pid_throttle, &twiddle](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
  h.onMessage([&pid, &pid_throttle](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(std::string(data).substr(0, length));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          double steer_value;
          /*
          * TODO: Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */
		  
		  // record values for Ziegler-Nichols method
          //outfile << counter << "\t" << cte << "\t" << speed << "\t" << angle << std::endl;
		  
		  // calculate steering value
          pid.UpdateError(cte);
          steer_value = pid.TotalError();
          //pid.Twiddle();
		  
		  // update throttle controller
		  pid_throttle.UpdateError(fabs(steer_value));

		  // clip steering value
          if (steer_value < -1.0) steer_value = -1.0;
          if (steer_value > 1.0) steer_value = 1.0;

		  // Ziegler-Nichols method
          //pid.ZieglerNichols();
          //++counter;

		  // calculate throttle value
          double throttle = 0.75 + pid_throttle.TotalError(); //0.5;
          if (throttle < 0.1) throttle = 0.1;
          if (throttle > 1.0) throttle = 1.0;
          if (speed < 20.0) throttle = 0.3;

		  /*
		  // Tune PID coefficients
          twiddle.Update(cte, speed);
          pid.UpdateCoeffs(twiddle.coeffs[0], twiddle.coeffs[1], twiddle.coeffs[2]);
          pid_throttle.UpdateCoeffs(twiddle.coeffs[3], twiddle.coeffs[4], twiddle.coeffs[5]);
		  */

          // DEBUG
          //std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;
          //std::cout << "Speed: " << speed << " Steering Angle: " << angle << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle; //0.3;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1)
    {
      res->end(s.data(), s.length());
    }
    else
    {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
  
  //outfile.close();
}
