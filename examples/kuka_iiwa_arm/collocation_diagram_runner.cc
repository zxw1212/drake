#include <iostream>
#include <memory>
#include <string>

#include "lcm/lcm-cpp.hpp"
#include <gflags/gflags.h>

#include "drake/common/find_resource.h"
#include "drake/common/is_approx_equal_abstol.h"
#include "drake/geometry/drake_visualizer.h"
#include "drake/geometry/scene_graph.h"
#include "drake/lcmt_iiwa_command.hpp"
#include "drake/lcmt_iiwa_status.hpp"
#include "drake/multibody/parsing/parser.h"
#include "drake/planning/trajectory_optimization/direct_collocation.h"
#include "drake/solvers/solve.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/controllers/pid_controlled_system.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/trajectory_source.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace {
const char* const kLcmStatusChannel = "IIWA_STATUS";
const char* const kLcmCommandChannel = "IIWA_COMMAND";

using drake::solvers::SolutionResult;
using multibody::MultibodyPlant;
using planning::trajectory_optimization::DirectCollocation;
using trajectories::PiecewisePolynomial;

DEFINE_double(target_realtime_rate, 1.0,
              "Playback speed.  See documentation for "
              "Simulator::set_target_realtime_rate() for details.");
DEFINE_double(sim_dt, 0.0,
              "The time step to use for MultibodyPlant model "
              "discretization.");
DEFINE_string(urdf, "", "Name of urdf to load");

class LCMNode {
 private:
  ::lcm::LCM lcm_;
  lcmt_iiwa_status iiwa_status_;
  void HandleStatus(const ::lcm::ReceiveBuffer*, const std::string&,
                    const lcmt_iiwa_status* status) {
    iiwa_status_ = *status;
  }

 public:
  LCMNode() { lcm_.subscribe(kLcmStatusChannel, &LCMNode::HandleStatus, this); }
  void Publish(lcmt_iiwa_command cmd) {
    lcm_.publish(kLcmCommandChannel, &cmd);
  }
  lcmt_iiwa_status& status() { return iiwa_status_; }
  int handleTimeout(double time_out) { return lcm_.handleTimeout(time_out); }
};

int DoMain() {
  systems::DiagramBuilder<double> builder;

  // Adds a plant.
  auto [plant, scene_graph] =
      multibody::AddMultibodyPlantSceneGraph(&builder, FLAGS_sim_dt);
  const char* kModelPath =
      "drake/manipulation/models/iiwa_description/"
      "urdf/iiwa14_polytope_collision.urdf";
  const std::string urdf =
      (!FLAGS_urdf.empty() ? FLAGS_urdf : FindResourceOrThrow(kModelPath));
  auto arm_instance =
      multibody::Parser(&plant, &scene_graph).AddModels(urdf).at(0);
  plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("base"));

  // add new object(besides arm arm) to scene
  // const std::string obj_url =
  //     "package://drake/examples/kuka_iiwa_arm/models/objects/"
  //     "simple_sphere.urdf";
  // const auto model =
  //     multibody::Parser(&plant,
  //     &scene_graph).AddModelsFromUrl(obj_url).at(0);
  // plant.WeldFrames(
  //     plant.world_frame(), plant.GetFrameByName("sphere_base", model),
  //     math::RigidTransform<double>(math::RollPitchYaw<double>(0.0, 0.0, 0.0),
  //                                  Eigen::Vector3d(-0.3, 0.0, 1.0)));

  plant.Finalize();
  plant.set_name("kuka_iiwa_arm");
  const std::string kActuationPortName = "plant_input";
  builder.ExportInput(plant.get_actuation_input_port(arm_instance),
                      kActuationPortName);
  auto diagram = builder.Build();
  auto context = diagram->CreateDefaultContext();
  const int kNumTimeSamples = 5;
  const double kMinimumTimeStep = 0.1;
  const double kMaximumTimeStep = 0.5;
  DirectCollocation dircol(
      diagram.get(), *context, kNumTimeSamples, kMinimumTimeStep,
      kMaximumTimeStep, diagram->GetInputPort(kActuationPortName).get_index(),
      true /*assume_non_continuous_state_are_fixed*/);

  auto& prog = dircol.prog();

  dircol.AddEqualTimeIntervalsConstraints();

  const double kTorqueLimit = 100;  // N*m.
  const solvers::VectorXDecisionVariable& u = dircol.input();
  dircol.AddConstraintToAllKnotPoints(
      -kTorqueLimit * Eigen::VectorXd::Ones(plant.num_positions()) <= u);
  dircol.AddConstraintToAllKnotPoints(
      u <= kTorqueLimit * Eigen::VectorXd::Ones(plant.num_positions()));

  Eigen::VectorXd init_state;
  Eigen::VectorXd final_state;
  Eigen::VectorXd init_position;
  Eigen::VectorXd final_position;
  Eigen::VectorXd init_vel;
  Eigen::VectorXd final_vel;
  init_state.resize(2 * plant.num_positions());
  final_state.resize(2 * plant.num_positions());
  init_position.resize(plant.num_positions());
  init_vel.resize(plant.num_positions());
  final_position.resize(plant.num_positions());
  final_vel.resize(plant.num_positions());
  init_position << 0.0, 0.0, 0.0, 1.57, 0.0, 0.0, 0.0;
  final_position << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  init_vel.setZero();
  final_vel.setZero();
  init_state << init_position, init_vel;
  final_state << final_position, final_vel;
  prog.AddLinearConstraint(dircol.initial_state() == init_state);
  prog.AddLinearConstraint(dircol.final_state() == final_state);

  const double R = 10;  // Cost on input "effort".
  dircol.AddRunningCost((R * u.cast<symbolic::Expression>()).dot(u));

  const double timespan_init = 4;
  auto traj_init_x = PiecewisePolynomial<double>::FirstOrderHold(
      {0, timespan_init}, {init_state, init_state});
  dircol.SetInitialTrajectory(PiecewisePolynomial<double>(), traj_init_x);
  const auto result = solvers::Solve(dircol.prog());
  if (!result.is_success()) {
    std::cerr << "Failed to solve optimization for the swing-up trajectory"
              << std::endl;
    return 1;
  }

  const PiecewisePolynomial<double> pp_traj =
      dircol.ReconstructInputTrajectory(result);
  const PiecewisePolynomial<double> pp_xtraj =
      dircol.ReconstructStateTrajectory(result);

  // publish joint state command /////////////////////////////
  LCMNode lcm_node;
  lcm_node.status().utime = -1;
  double start_time = -1;
  bool first_flag = true;

  lcmt_iiwa_command iiwa_command;
  iiwa_command.num_joints = plant.num_positions();
  iiwa_command.joint_position.resize(plant.num_positions(), 0.);
  iiwa_command.num_torques = plant.num_positions();
  iiwa_command.joint_torque.resize(plant.num_positions(), 0.);
  std::cout << "Polyminal duration: " << pp_xtraj.end_time() << std::endl;
  while (1) {
    // Call lcm handle until at least one status message is
    // processed.
    while (0 == lcm_node.handleTimeout(10) || lcm_node.status().utime == -1) {
      std::cout << "lcm waiting" << std::endl;
    }

    if (first_flag) {
      first_flag = false;
      start_time = lcm_node.status().utime / 1e6;
    }

    const auto joint_state_cmd =
        pp_xtraj.value(lcm_node.status().utime / 1e6 - start_time);
    // std::cout << joint_state_cmd.block(0, 0, 7, 1).transpose() << std::endl;
    // std::cout << joint_state_cmd.block(7, 0, 7, 1).transpose() << std::endl;

    iiwa_command.utime = lcm_node.status().utime;

    for (int joint = 0; joint < plant.num_positions(); joint++) {
      iiwa_command.joint_position[joint] = joint_state_cmd(joint);
    }

    lcm_node.Publish(iiwa_command);
  }
  //////////////////////////////////////////////////////////////////////////////

  return 0;
}

}  // namespace
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::kuka_iiwa_arm::DoMain();
}
