#include <iostream>
#include <memory>
#include <string>

#include "lcm/lcm-cpp.hpp"
#include <gflags/gflags.h>

#include "drake/common/find_resource.h"
#include "drake/geometry/drake_visualizer.h"
#include "drake/geometry/scene_graph.h"
#include "drake/lcmt_iiwa_command.hpp"
#include "drake/lcmt_iiwa_status.hpp"
#include "drake/multibody/inverse_kinematics/inverse_kinematics.h"
#include "drake/multibody/inverse_kinematics/minimum_distance_lower_bound_constraint.h"
#include "drake/multibody/inverse_kinematics/orientation_constraint.h"
#include "drake/multibody/inverse_kinematics/position_constraint.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/planning/trajectory_optimization/kinematic_trajectory_optimization.h"
#include "drake/solvers/ipopt_solver.h"
#include "drake/solvers/solve.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace {
const char* const kLcmStatusChannel = "IIWA_STATUS";
const char* const kLcmCommandChannel = "IIWA_COMMAND";

using drake::solvers::SolutionResult;
using multibody::MultibodyPlant;

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
      "urdf/iiwa14_spheres_collision.urdf";
  const std::string urdf =
      (!FLAGS_urdf.empty() ? FLAGS_urdf : FindResourceOrThrow(kModelPath));
  auto arm_instance =
      multibody::Parser(&plant, &scene_graph).AddModels(urdf).at(0);
  plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("base"));

  // add new object(besides arm arm) to scene
  const std::string obj_url =
      "package://drake/examples/kuka_iiwa_arm/models/objects/"
      "folding_table.urdf";
  const auto model =
      multibody::Parser(&plant, &scene_graph).AddModelsFromUrl(obj_url).at(0);
  plant.WeldFrames(
      plant.world_frame(), plant.GetFrameByName("table_surface_center", model),
      math::RigidTransform<double>(math::RollPitchYaw<double>(0.0, 0.0, 0.0),
                                   Eigen::Vector3d(0.0, 0.0, 1.0)));

  plant.Finalize();
  plant.set_name("kuka_iiwa_arm");
  const std::string kActuationPortName = "plant_input";
  builder.ExportInput(plant.get_actuation_input_port(arm_instance),
                      kActuationPortName);
  auto diagram = builder.Build();
  auto context = diagram->CreateDefaultContext();
  auto& plant_context = plant.GetMyMutableContextFromRoot(context.get());
  const int num_q = plant.num_positions();

  constexpr int kNumCtrlPoints = 5;
  auto trajopt =
      planning::trajectory_optimization::KinematicTrajectoryOptimization(
          num_q, kNumCtrlPoints);
  auto& prog = trajopt.get_mutable_prog();
  // These two costs may cause no solution!
  trajopt.AddDurationCost(1.0);
  // trajopt.AddPathLengthCost(1.0, true);
  trajopt.AddPositionBounds(plant.GetPositionLowerLimits(),
                            plant.GetPositionUpperLimits());
  trajopt.AddVelocityBounds(plant.GetVelocityLowerLimits(),
                            plant.GetVelocityUpperLimits());
  trajopt.AddAccelerationBounds(plant.GetAccelerationLowerLimits(),
                                plant.GetAccelerationUpperLimits());
  trajopt.AddDurationConstraint(0.5, 5);

  // start constraint
  constexpr char kFlangeFrameName[] = "iiwa_link_ee";
  Eigen::VectorXd init_q;
  init_q.resize(num_q);
  init_q << 0.0, 0.0, 0.0, 1.57, 0.0, 0.0, 0.0;
  Eigen::MatrixXd eye = Eigen::MatrixXd::Identity(num_q, num_q);

  trajopt.AddPathPositionConstraint(init_q, init_q, 0);
  trajopt.AddPathVelocityConstraint(Eigen::MatrixXd::Zero(num_q, 1),
                                    Eigen::MatrixXd::Zero(num_q, 1), 0);
  prog.AddQuadraticErrorCost(eye, init_q, trajopt.control_points().col(0));

  // final pose constraint
  // auto target_cartesian_pose = math::RigidTransform<double>(
  //     math::RollPitchYaw<double>(3.14159, 0.0, 3.14159),
  //     Eigen::Vector3d(-0.5259, 0.0, 0.8804));
  // auto target_cartesian_pose = math::RigidTransform<double>(
  //     math::RollPitchYaw<double>(-3.141592653589793, -1.5707963267948961,
  //                                3.141592653589793),
  //     Eigen::Vector3d(0.0, 0.0, 1.306));
  auto target_cartesian_pose =
      math::RigidTransform<double>(math::RollPitchYaw<double>(0.0, 0.0, 0.0),
                                   Eigen::Vector3d(0.526, 0.0, 0.780));

  auto final_position_constraint =
      std::make_shared<multibody::PositionConstraint>(
          &plant, plant.world_frame(),
          target_cartesian_pose.translation() /*xyz_lower*/,
          target_cartesian_pose.translation() /*xyz_upper*/,
          plant.GetFrameByName(kFlangeFrameName), Eigen::Vector3d::Zero(),
          &plant_context);
  constexpr double kRotAccuracy = 0.001;
  auto final_orientation_constraint =
      std::make_shared<multibody::OrientationConstraint>(
          &plant, plant.world_frame(), target_cartesian_pose.rotation(),
          plant.GetFrameByName(kFlangeFrameName),
          math::RotationMatrix<double>::Identity(), kRotAccuracy,
          &plant_context);
  trajopt.AddPathPositionConstraint(final_position_constraint, 1);
  trajopt.AddPathPositionConstraint(final_orientation_constraint, 1);
  trajopt.AddPathVelocityConstraint(Eigen::MatrixXd::Zero(num_q, 1),
                                    Eigen::MatrixXd::Zero(num_q, 1), 1);
  prog.AddQuadraticErrorCost(
      eye, init_q,
      trajopt.control_points().col(trajopt.control_points().cols() - 1));

  // initial guess
  // multibody::InverseKinematics ik(plant, &plant_context, false);
  // ik.get_mutable_prog()->AddQuadraticErrorCost(eye, init_q, ik.q());
  // ik.AddPositionConstraint(plant.GetFrameByName(kFlangeFrameName),
  //                          Eigen::Vector3d::Zero(), plant.world_frame(),
  //                          target_cartesian_pose.translation(),
  //                          target_cartesian_pose.translation());
  // ik.AddOrientationConstraint(
  //     plant.world_frame(), target_cartesian_pose.rotation(),
  //     plant.GetFrameByName(kFlangeFrameName),
  //     math::RotationMatrix<double>::Identity(), kRotAccuracy);
  // ik.get_mutable_prog()->SetSolverOption(solvers::IpoptSolver::id(),
  // "max_iter",
  //                                        10000);
  // ik.get_mutable_prog()->SetSolverOption(solvers::IpoptSolver::id(), "tol",
  //                                        1.0e-4);
  // const auto ik_result = solvers::Solve(ik.prog());
  // if (!ik_result.is_success()) {
  //   std::cout << "The provided final target pose is not sovlable via IK."
  //             << std::endl;
  //   return 1;
  // }
  // const auto q_ik = ik_result.GetSolution(ik.q());
  // std::vector<Eigen::MatrixXd> q_control_points;
  // q_control_points.resize(kNumCtrlPoints);
  // for (int i = 0; i < kNumCtrlPoints; i++) {
  //   Eigen::MatrixXd q_control_point;
  //   q_control_point.resize(num_q, 1);
  //   q_control_point =
  //       (q_ik - init_q) * (double(i) / double(kNumCtrlPoints - 1)) + init_q;
  //   q_control_points[i] = q_control_point;
  //   std::cout << q_control_point.transpose() << std::endl;
  // }
  // auto path_guess = trajectories::BsplineTrajectory<double>(trajopt.basis(),
  //                                                           q_control_points);
  // trajopt.SetInitialGuess(path_guess);

  // add collision avoidance
  auto collision_constraint =
      std::make_shared<multibody::MinimumDistanceLowerBoundConstraint>(
          &plant, 0.05, &plant_context);
  for (double i = 0.0; i <= 1.0; i = i + 0.1) {
    trajopt.AddPathPositionConstraint(collision_constraint, i);
  }

  const auto result = solvers::Solve(prog);
  if (!result.is_success()) {
    std::cerr << "Failed to solve optimization" << std::endl;
    return 1;
  }

  trajectories::BsplineTrajectory<double> q =
      trajopt.ReconstructTrajectory(result);
  auto qdot = q.MakeDerivative();
  auto qddot = qdot->MakeDerivative();

  // publish joint state command /////////////////////////////
  LCMNode lcm_node;
  lcm_node.status().utime = -1;
  double start_time = -1;
  bool first_flag = true;

  lcmt_iiwa_command iiwa_command;
  iiwa_command.num_joints = num_q;
  iiwa_command.joint_position.resize(num_q, 0.);
  iiwa_command.num_torques = num_q;
  iiwa_command.joint_torque.resize(num_q, 0.);
  std::cout << "BsplineTrajectory duration: " << q.end_time() << std::endl;
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
        q.value(lcm_node.status().utime / 1e6 - start_time);
    iiwa_command.utime = lcm_node.status().utime;

    for (int joint = 0; joint < num_q; joint++) {
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
