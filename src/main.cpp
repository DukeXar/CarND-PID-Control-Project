#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <list>
#include "PID.h"
#include "json.hpp"
#include "wip/optimizer.h"

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
  } else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

class SlidingWindowAvg {
 public:
  explicit SlidingWindowAvg(size_t limit) : limit_(limit) {}

  void Add(double value) {
    sum_ += value;
    values_.push_back(value);
    if (values_.size() > limit_) {
      sum_ -= values_.front();
      values_.pop_front();
    }
  }

  bool IsFull() const { return values_.size() == limit_; }

  bool IsEmpty() const { return values_.empty(); }

  double Avg() const { return sum_ / values_.size(); }

  void Reset() {
    values_.clear();
    sum_ = 0;
  }

 private:
  size_t limit_;
  std::list<double> values_;
  double sum_;
};

int main() {
  uWS::Hub h;

  bool training_mode = true;
  const unsigned training_steps_limit = 4500;

  std::cout << "Training mode: " << training_mode << std::endl;
  std::cout << "Training steps: " << training_steps_limit << std::endl;

  HillClimbingOptimizer opt(training_steps_limit);

  PID pid;
  pid.Init(opt.kp(), opt.ki(), opt.kd());
  std::cout << "Steering PID: " << pid << std::endl;

  const double target_speed = 20.0;

  PID speed_pid;
  speed_pid.Init(1, 0.001, 0.1);  // target_speed = 30.0
  std::cout << "Throttle PID: " << speed_pid << std::endl;
  std::cout << "Target speed: " << target_speed << std::endl;

  bool start_processing = false;
  auto start_time = std::chrono::high_resolution_clock::now();

  SlidingWindowAvg avg_speed(50);

  h.onMessage([&pid, &speed_pid, target_speed, &training_mode, &opt,
               &start_processing, &start_time,
               &avg_speed](uWS::WebSocket<uWS::SERVER> ws, char *data,
                           size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message
    // event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(std::string(data).substr(0, length));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (start_processing && event == "telemetry") {
          bool send_reset = false;

          auto now = std::chrono::high_resolution_clock::now();
          using sec = std::chrono::duration<double>;
          auto delta_time = std::chrono::duration_cast<sec>(now - start_time);
          // std::cout << delta_time.count() << std::endl;
          start_time = now;

          // On first run, delta can be large, try to cope with that and still
          // produce control by assuming we just started.
          if (delta_time > sec(1.0)) {
            delta_time = sec(1.0);
          }

          // j[1] is the data JSON object
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());

          double cte = std::stod(j[1]["cte"].get<std::string>());
          pid.UpdateError(cte, delta_time.count());

          double steer_value = pid.GetControl();
          if (steer_value < -1) {
            steer_value = -1;
          } else if (steer_value > 1) {
            steer_value = 1;
          }

          avg_speed.Add(speed);
          speed_pid.UpdateError(speed - target_speed, delta_time.count());

          if (training_mode) {
            const bool too_diverged = (std::abs(cte) > 4);
            const bool is_stuck = (avg_speed.IsFull() && avg_speed.Avg() < 0.1);
            const bool bad_update = too_diverged || is_stuck;
            if (bad_update) {
              std::cout << "Bad state: ";
              if (too_diverged) {
                std::cout << "too diverged from target";
                if (is_stuck) {
                  std::cout << ", ";
                }
              }
              if (is_stuck) {
                std::cout << "stuck (speed zero)";
              }
              std::cout << std::endl;
            }
            switch (opt.Update(cte, bad_update)) {
              case ResetMode::None:
                break;
              case ResetMode::Repeat: {
                std::cout << "Results of last run: " << std::endl;
                std::cout << "Updating optimizer" << std::endl;
                pid.Init(opt.kp(), opt.ki(), opt.kd());
                std::cout << "Steering PID: " << pid << std::endl;
                speed_pid.ResetState();
                avg_speed.Reset();
                send_reset = true;
              } break;
              case ResetMode::Finish: {
                std::cout << "Finished!" << std::endl;
                training_mode = false;
              } break;
            }
          }

          if (send_reset) {
            std::string msg = "42[\"reset\",{}]";
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            start_processing = false;
          } else {
            json msgJson;
            msgJson["steering_angle"] = steer_value;
            msgJson["throttle"] = speed_pid.GetControl();
            auto msg = "42[\"steer\"," + msgJson.dump() + "]";
            // std::cout << msg << std::endl;
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          }
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

  h.onConnection([&h, &start_processing](uWS::WebSocket<uWS::SERVER> ws,
                                         uWS::HttpRequest req) {
    std::cout << ">>>>> Connected!!!" << std::endl;
    start_processing = true;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "<<<<< Disconnected" << std::endl;
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
