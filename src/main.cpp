#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include "PID.h"
#include "json.hpp"

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

class RMSEAccumulator {
 public:
  RMSEAccumulator() : sum_(0), count_(0) {}
  RMSEAccumulator(double sum, unsigned count) : sum_(sum), count_(count) {}

  void Add(double value) {
    sum_ += value * value;
    count_++;
  }

  double Get() const { return sqrt(sum_ / count_); }

  void Reset() {
    sum_ = 0;
    count_ = 0;
  }

 private:
  double sum_;
  unsigned count_;
};

enum class ResetMode { None, Repeat, Finish };

template <typename T>
std::ostream &operator<<(std::ostream &os, const std::vector<T> &coll) {
  const char *delim = "";
  os << "[";
  for (const auto &item : coll) {
    os << delim << item;
    delim = ", ";
  }
  os << "]";
  return os;
}

class TwiddleOptimizer {
 public:
  TwiddleOptimizer(unsigned updates_per_iteration)
      : updates_per_iteration_(updates_per_iteration),
        current_iteration_(0),
        current_update_(0),
        curr_idx_(0),
        state_(-1) {
    d_[0] = 1;
    d_[1] = 0.05;
    d_[2] = 1;

    k_[0] = 0.424788;
    k_[1] = 0.0;
    k_[2] = 20.912;

    // k_[0] = 0.424788;
    // k_[1] = 0;
    // k_[2] = 3.912;

    // d_[0] = 0.2;
    // d_[1] = 0.05;
    // d_[2] = 0.5;
  }

  double kp() const { return k_[0]; }
  double ki() const { return k_[1]; }
  double kd() const { return k_[2]; }

  ResetMode Update(double cte) {
    rmse_.Add(cte);

    if (cte > 4) {
      std::cout << "Premature stopping - huge cte=" << cte << std::endl;
      bool result = finish(false);

      if (result) {
        return ResetMode::Finish;
      }
      return ResetMode::Repeat;
    }

    ++current_update_;

    if (current_update_ >= updates_per_iteration_) {
      bool result = finish(true);

      if (result) {
        return ResetMode::Finish;
      }
      return ResetMode::Repeat;
    }

    return ResetMode::None;
  }

 private:
  bool finish(bool good_run) {
    double value = good_run ? rmse_.Get() : std::numeric_limits<double>::max();

    if (state_ == -1) {
      best_ = value;
      state_ = 0;
    }

    bool try_combination = false;
    while (!try_combination) {
      switch (state_) {
        case 0: {
          orig_ = k_[curr_idx_];
          k_[curr_idx_] = orig_ + d_[curr_idx_];
          state_ = 1;
          try_combination = true;
        }; break;
        case 1: {
          if (value < best_) {
            best_ = value;
            d_[curr_idx_] *= 1.1;
            state_ = 0;
            curr_idx_ = (curr_idx_ + 1) % 3;
          } else {
            if (orig_ - d_[curr_idx_] >= 0) {
              state_ = 2;
              k_[curr_idx_] = orig_ - d_[curr_idx_];
              try_combination = true;
            } else {
              state_ = 2;
            }
          }
        }; break;
        case 2: {
          if (value < best_) {
            best_ = value;
            d_[curr_idx_] *= 1.1;
          } else {
            k_[curr_idx_] = orig_;
            d_[curr_idx_] *= 0.9;
          }
          state_ = 0;
          curr_idx_ = (curr_idx_ + 1) % 3;
        }; break;
      }
    }

    std::cout << "Optimizer state ----" << std::endl;
    std::cout << "    iteration=" << current_iteration_ << std::endl;
    std::cout << "    state=" << state_ << std::endl;
    std::cout << "    curr_idx_=" << curr_idx_ << std::endl;
    std::cout << "    k=" << k_[0] << ", " << k_[1] << ", " << k_[2]
              << std::endl;
    std::cout << "    d=" << d_[0] << ", " << d_[1] << ", " << d_[2]
              << std::endl;
    std::cout << "    rmse=" << rmse_.Get() << std::endl;
    std::cout << "--------------------" << std::endl;

    rmse_.Reset();
    current_update_ = 0;
    ++current_iteration_;

    if (d_[0] + d_[1] + d_[2] < 0.001) {
      return true;
    }
    return false;
  }

 private:
  unsigned updates_per_iteration_;
  unsigned current_iteration_;
  unsigned current_update_;

  double d_[3];
  double k_[3];
  int curr_idx_;
  int state_;
  double orig_;
  double best_;

  RMSEAccumulator rmse_;
};

int main() {
  uWS::Hub h;

  bool training_mode = false;
  const unsigned training_steps_limit = 1500;

  std::cout << "Training mode: " << training_mode << std::endl;
  std::cout << "Training steps: " << training_steps_limit << std::endl;

  TwiddleOptimizer opt(training_steps_limit);

  PID pid;
  pid.Init(opt.kp(), opt.ki(), opt.kd());
  std::cout << "Steering PID: " << pid << std::endl;

  const double target_speed = 30.0;

  PID speed_pid;
  speed_pid.Init(1, 0.001, 0.1);  // target_speed = 30.0
  std::cout << "Throttle PID: " << speed_pid << std::endl;
  std::cout << "Target speed: " << target_speed << std::endl;

  bool start_processing = false;
  auto start_time = std::chrono::high_resolution_clock::now();

  h.onMessage([&pid, &speed_pid, target_speed, &training_mode, &opt,
               &start_processing, &start_time](uWS::WebSocket<uWS::SERVER> ws,
                                               char *data, size_t length,
                                               uWS::OpCode opCode) {
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
          auto delta_time = now - start_time;
          start_time = now;

          // On first run, delta can be large, try to cope with that and still
          // produce control by assuming we just started.
          if (delta_time.count() > 1) {
            delta_time = std::chrono::milliseconds(1);
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

          speed_pid.UpdateError(speed - target_speed, delta_time.count());
          double throttle = speed_pid.GetControl();

          if (training_mode) {
            switch (opt.Update(cte)) {
              case ResetMode::None:
                break;
              case ResetMode::Repeat: {
                std::cout << "Results of last run: " << std::endl;
                std::cout << "Updating optimizer" << std::endl;
                pid.Init(opt.kp(), opt.ki(), opt.kd());
                std::cout << "Steering PID: " << pid << std::endl;
                speed_pid.ResetState();
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
            msgJson["throttle"] = throttle;
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
