#include <cmath>
#include <iostream>
#include <vector>

enum class ResetMode { None, Repeat, Finish };

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

class HillClimbingOptimizer {
 public:
  explicit HillClimbingOptimizer(unsigned updates_per_iteration)
      : updates_per_iteration_(updates_per_iteration),
        current_iteration_(0),
        current_update_(0),
        current_state_(),
        best_state_(),
        has_best_state_(false),
        acc_(1.2) {
    candidates_ = GenerateCandidates(current_state_);
  }

  double kp() const { return current_state_.k[0]; }
  double ki() const { return current_state_.k[1]; }
  double kd() const { return current_state_.k[2]; }

  ResetMode Update(double cte, bool bad_update);

 private:
  struct State {
    State() : k{0.0, 0.0, 0.0}, distance(0), next_step_size{1.0, 1.0, 1.0} {}

    std::vector<double> k;
    RMSEAccumulator rmse;
    int distance;

    std::vector<double> next_step_size;
  };

 private:
  ResetMode RecalculateState(bool bad_update);

  bool CreateNextState(bool bad_update);

  std::vector<State> GenerateCandidates(const State &state);

 private:
  unsigned updates_per_iteration_;
  unsigned current_iteration_;
  unsigned current_update_;
  State current_state_;
  State best_state_;
  bool has_best_state_;
  double acc_;

  std::vector<State> candidates_;
};