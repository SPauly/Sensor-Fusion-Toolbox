#ifndef SENSFUS_SIM_RADAR_SIM_H
#define SENSFUS_SIM_RADAR_SIM_H

#include <vector>
#include <thread>
#include <mutex>

#include "sensfus/types.h"
#include "sensfus/sim/sim_base.h"

namespace sensfus {
namespace sim {

struct RadarDataType {
  std::vector<ScalarType> zx, zy;      // Measurement in 2D
  std::vector<double> range, azimuth;  // Measurement of range and azimuth
};

using RadarSimState =
    SimState<std::vector<std::vector<ObjectState2D>>, RadarDataType>;

class RadarSim : public SimBase {
 public:
  explicit RadarSim() = default;
  ~RadarSim() override = default;

  virtual void Init() override;
  virtual void StartSimulation() override;
  virtual void HaltSimulation() override;
  virtual void Stop() override;
  virtual void ChangeUpdateRate(double rate_in_ns) override;

  virtual RadarSimState GetState() const = 0;

 private:
  double update_rate_;
};

}  // namespace sim
}  // namespace sensfus

#endif  // SENSFUS_SIM_RADAR_SIM_H