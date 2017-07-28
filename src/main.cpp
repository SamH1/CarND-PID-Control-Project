#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>


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

// global parameters for Twiddle algorithm
bool use_twiddle = true;
long timer = 0;
long tt = 0;
double err = 1000;
double tot_str;
double p[] = {0.1,0,0.9};
double dp[] = {.01,0,.01};
double best_err = err;
int step = 0;
int p_index = 0;

//twiddle
void twiddle (){
        
    if (step == 0){
      p[p_index] += dp[p_index];
      //pid.Init(-p[0], -p[1], -p[2]);
      step = 1;
    } else if (step == 1){
        if (err < best_err){
            best_err = err;
            dp[p_index] *= 1.1;
            p_index = (p_index + 1) % 3;
            step = 0;
        } else {
            p[p_index] -= 2*dp[p_index];
            //pid.Init(-p[0], -p[1], -p[2]);
            step = 2;
        }       
    } else {
        if (err < best_err){
            best_err = err;
            dp[p_index] *= 1.1;
            p_index = (p_index + 1) % 3;
            step = 0;
        } else {
            p[p_index] += dp[p_index];
            dp[p_index] *= 0.9;
            //pid.Init(-p[0], -p[1], -p[2]);
            p_index = (p_index + 1) % 3;
            step = 0;
        } 
    }
}

int main()
{
  uWS::Hub h;

  PID pid;
  // TODO: Initialize the pid variable.
  double Kp0 = -0.1;
  double Ki0 = 0.;
  double Kd0 = -0.9;
  pid.Init(Kp0, Ki0, Kd0);

  h.onMessage([&pid](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    timer += 1;
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
          pid.UpdateError(cte);
          steer_value = pid.TotalError();
          err += std::abs(pid.p_error);
          tot_str += std::abs(steer_value);

          if (timer == 500 and use_twiddle){
            std::cout << std::fixed;
            std::cout << std::setprecision(4);           
            std::cout << "\n----total_steering: " << tot_str << " error: " << pid.p_error << ", " << pid.i_error << ", " << pid.d_error << ", "<< pid.Kp * pid.p_error + pid.Ki * pid.i_error + pid.Kd * pid.d_error << std::endl;
            std::cout << "----iteration: " << tt << " err: " << err  << " p[" << pid.Kp << ", " << pid.Ki << ", " << pid.Kd << "]\n" << std::endl;
            twiddle();
            pid.Init(-p[0], -p[1], -p[2]);
            if (p_index == 1) {p_index = 2;}
            timer = 0;
            err = 0;
            tot_str = 0;
            tt += 1;
          }
          // DEBUG
          std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = 0.3;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
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
}
