#include <vector>

class HillClimbingOptimizer {
 private:
  struct State {
    State() : k{0.05, 0.0, 0.0}, next_step_size{0.05, 1.0, 1.0} {}

    std::vector<double> k;
    RMSEAccumulator rmse;

    std::vector<double> next_step_size;
  };

 public:
  explicit AnnealingOptimizer(unsigned updates_per_iteration)
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

  ResetMode Update(double cte) {
    current_state_.rmse.Add(cte);
    ++current_update_;

    if (current_update_ >= updates_per_iteration_) {
      return RecalculateState();
    }
    return ResetMode::None;
  }

 private:
  ResetMode RecalculateState() {
    ++current_iteration_;

    std::cout << "Optimizer state ----" << std::endl;
    std::cout << "    iteration=" << current_iteration_ << std::endl;
    std::cout << "    k=" << current_state_.k << std::endl;
    std::cout << "    rmse=" << current_state_.rmse.Get() << std::endl;
    std::cout << "    next_steps=" << current_state_.next_step_size
              << std::endl;
    std::cout << "--------------------" << std::endl;

    current_state_ = CreateNextState();
    current_update_ = 0;
    return ResetMode::Repeat;
  }

  bool IsBetter(const State &s1, const State &s2) {
    return s1.rmse.Get() < s2.rmse.Get();
  }

  State CreateNextState() {
    if (!has_best_state_ || IsBetter(current_state_, best_state_)) {
      best_state_ = current_state_;
    }

    candidates_.erase(candidates_.begin());

    if (candidates_.empty()) {
      candidates_ = GenerateCandidates(best_state_);
    }

    // FIXME How to report that no more iterations possible?

    return candidates_.front();
  }

  std::vector<State> GenerateCandidates(const State &state) {
    std::vector<State> result;
    const double shifts[] = {-acc_, -1 / acc_, 0, 1 / acc_, acc_};

    for (int i : {0, 2}) {
      for (double shift : shifts) {
        State s = state;
        s.k[i] = s.k[i] + s.next_step_size[i] * shift;

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
