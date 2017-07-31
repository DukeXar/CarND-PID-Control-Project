#include <math.h>
#include <iostream>
#include <vector>

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
 private:
  struct State {
    State() : k{0.0, 0.0, 0.0}, distance(0), next_step_size{1.0, 1.0, 1.0} {}

    std::vector<double> k;
    RMSEAccumulator rmse;
    int distance;

    std::vector<double> next_step_size;
  };

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

  ResetMode Update(double cte, bool bad_update) {
    current_state_.rmse.Add(cte);
    current_state_.distance++;
    ++current_update_;

    if (bad_update || current_update_ >= updates_per_iteration_) {
      return RecalculateState(bad_update);
    }
    return ResetMode::None;
  }

 private:
  ResetMode RecalculateState(bool bad_update) {
    ++current_iteration_;

    std::cout << "Finished testing state ----" << std::endl;
    std::cout << "    finished iteration=" << current_iteration_ << std::endl;
    std::cout << "    k=" << current_state_.k << std::endl;
    std::cout << "    rmse=" << current_state_.rmse.Get() << std::endl;
    std::cout << "    distance=" << current_state_.distance << std::endl;
    std::cout << "    next_steps=" << current_state_.next_step_size
              << std::endl;

    ResetMode result;
    if (CreateNextState(bad_update)) {
      current_update_ = 0;
      result = ResetMode::Repeat;
    } else {
      result = ResetMode::Finish;
    }

    std::cout << "    best_state=" << best_state_.k << std::endl;
    std::cout << "    best_state rmse=" << best_state_.rmse.Get() << std::endl;
    std::cout << "    best_state distance=" << best_state_.distance
              << std::endl;
    std::cout << "--------------------" << std::endl;

    return result;
  }

  bool CreateNextState(bool bad_update) {
    if (!has_best_state_) {
      best_state_ = current_state_;
      has_best_state_ = true;
      std::cout << "... This was the first state" << std::endl;
    } else {
      double delta;
      if (bad_update) {
        std::cout << "... Bad update - use distance to compare" << std::endl;
        std::cout << "    distance=" << current_state_.distance
                  << ", best distance=" << best_state_.distance << std::endl;
        delta = current_state_.distance - best_state_.distance;
      } else {
        delta = best_state_.rmse.Get() - current_state_.rmse.Get();
      }
      std::cout << "... delta=" << delta << std::endl;
      if (delta > 0) {
        std::cout << "... Best state updated" << std::endl;
        best_state_ = current_state_;
      } else {
        std::cout << "... Not updating" << std::endl;
      }
      if (std::abs(delta) < 0.01) {
        return false;
      }
    }

    candidates_.erase(candidates_.begin());

    if (candidates_.empty()) {
      candidates_ = GenerateCandidates(best_state_);
      has_best_state_ = false;
    }

    current_state_ = candidates_.front();
    return true;
  }

  std::vector<State> GenerateCandidates(const State &state) {
    std::vector<State> result;
    const double shifts[] = {-acc_, -1 / acc_, 0, 1 / acc_, acc_};

    for (int i : {0, 2}) {
      for (double shift : shifts) {
        State s;
        s.k = state.k;
        s.k[i] = s.k[i] + s.next_step_size[i] * shift;

        s.next_step_size = state.next_step_size;
        if (shift == 0.0) {
          s.next_step_size[i] = s.next_step_size[i] / acc_;
        } else {
          s.next_step_size[i] = s.next_step_size[i] * shift;
        }

        result.push_back(s);
      }
    }
    return result;
  }

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
