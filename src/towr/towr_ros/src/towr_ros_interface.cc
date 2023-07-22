/******************************************************************************
Copyright (c) 2018, Alexander W. Winkler. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#include <towr_ros/towr_ros_interface.h>

#include <std_msgs/Int32.h>

#include <xpp_states/convert.h>
#include <xpp_msgs/topic_names.h>
#include <xpp_msgs/TerrainInfo.h>

#include <towr/terrain/height_map.h>
#include <towr/terrain/examples/height_map_examples.h>
#include <towr/variables/euler_converter.h>
#include <towr_ros/topic_names.h>
#include <towr_ros/towr_xpp_ee_map.h>
// #include <towr_ros/goal_pose_publisher.h>
// #include <towr_ros/towr_ros_interface.h>
// #include <towr_ros/towr_user_interface.h>


// double goal_pos_x = 1.0;
// double goal_pos_y = 0.0;
// double goal_pos_z = 0.0;
// double goal_q_x = 0.0;
// double goal_q_y = 0.0;
// double goal_q_z = 0.0;
// double goal_q_w = 0.0;

namespace towr {


TowrRosInterface::TowrRosInterface ()
{
    ::ros::NodeHandle n;

    user_command_sub_ = n.subscribe(towr_msgs::user_command, 1,
                                    &TowrRosInterface::UserCommandCallback, this);

    initial_state_pub_  = n.advertise<xpp_msgs::RobotStateCartesian>
                                            (xpp_msgs::robot_state_desired, 1);

    robot_parameters_pub_  = n.advertise<xpp_msgs::RobotParameters>
                                        (xpp_msgs::robot_parameters, 1);

    solver_ = std::make_shared<ifopt::IpoptSolver>();

    visualization_dt_ = 0.01;
}

BaseState
TowrRosInterface::GetGoalState(const TowrCommandMsg& msg) const
{
    BaseState goal;
    goal.lin.at(kPos) = xpp::Convert::ToXpp(msg.goal_lin.pos);
    goal.lin.at(kVel) = xpp::Convert::ToXpp(msg.goal_lin.vel);
    goal.ang.at(kPos) = xpp::Convert::ToXpp(msg.goal_ang.pos);
    goal.ang.at(kVel) = xpp::Convert::ToXpp(msg.goal_ang.vel);

    return goal;
}

void
TowrRosInterface::UserCommandCallback(const TowrCommandMsg& msg)
{
    // robot model
    formulation_.model_ = RobotModel(static_cast<RobotModel::Robot>(msg.robot));
    auto robot_params_msg = BuildRobotParametersMsg(formulation_.model_);
    robot_parameters_pub_.publish(robot_params_msg);

    // terrain
    auto terrain_id = static_cast<HeightMap::TerrainID>(msg.terrain);
    formulation_.terrain_ = HeightMap::MakeTerrain(terrain_id);

    int n_ee = formulation_.model_.kinematic_model_->GetNumberOfEndeffectors();
    formulation_.params_ = GetTowrParameters(n_ee, msg);
    formulation_.final_base_ = GetGoalState(msg);

    SetTowrInitialState();

    // solver parameters
    SetIpoptParameters(msg);

    // visualization
    PublishInitialState();

    // Defaults to /home/user/.ros/
    // TODO：落足点优化求解
    std::string bag_file = "towr_trajectory.bag";
    if (msg.optimize || msg.play_initialization) {
        nlp_ = ifopt::Problem();
        for (auto c : formulation_.GetVariableSets(solution))
        nlp_.AddVariableSet(c);
        for (auto c : formulation_.GetConstraints(solution))
        nlp_.AddConstraintSet(c);
        for (auto c : formulation_.GetCosts())
        nlp_.AddCostSet(c);

        solver_->Solve(nlp_);
        SaveOptimizationAsRosbag(bag_file, robot_params_msg, msg, false);
    }

    // playback using terminal commands
    if (msg.replay_trajectory || msg.play_initialization || msg.optimize) {
        int success = system(("rosbag play --topics "
            + xpp_msgs::robot_state_desired + " "
            + xpp_msgs::terrain_info
            + " -r " + std::to_string(msg.replay_speed)
            + " --quiet " + bag_file).c_str());
    }

    if (msg.plot_trajectory) {
        int success = system(("killall rqt_bag; rqt_bag " + bag_file + "&").c_str());
    }

    // to publish entire trajectory (e.g. to send to controller)
    // xpp_msgs::RobotStateCartesianTrajectory xpp_msg = xpp::Convert::ToRos(GetTrajectory());
}

void
TowrRosInterface::PublishInitialState()
{
    int n_ee = formulation_.initial_ee_W_.size();
    xpp::RobotStateCartesian xpp(n_ee);
    xpp.base_.lin.p_ = formulation_.initial_base_.lin.p();
    xpp.base_.ang.q  = EulerConverter::GetQuaternionBaseToWorld(formulation_.initial_base_.ang.p());

    for (int ee_towr=0; ee_towr<n_ee; ++ee_towr) {
        int ee_xpp = ToXppEndeffector(n_ee, ee_towr).first;
        xpp.ee_contact_.at(ee_xpp)   = true;
        xpp.ee_motion_.at(ee_xpp).p_ = formulation_.initial_ee_W_.at(ee_towr);
        xpp.ee_forces_.at(ee_xpp).setZero(); // zero for visualization
    }

    initial_state_pub_.publish(xpp::Convert::ToRos(xpp));
}

std::vector<TowrRosInterface::XppVec>
TowrRosInterface::GetIntermediateSolutions ()
{
    std::vector<XppVec> trajectories;

    for (int iter=0; iter<nlp_.GetIterationCount(); ++iter) {
        nlp_.SetOptVariables(iter);
        trajectories.push_back(GetTrajectory());
    }

    return trajectories;
}

// 从 goal_set.csv 中读取 timestep和goal信息
void ReadGoalFromCSV(int &time_step, double goal_set[7]) {
    std::ifstream f_goal("/home/rrqq/towr_workspace/goal_set.csv");
    if (!f_goal) {
        std::cout << "Error opening goal_set file!\n";
        return;
    }

    std::string line;
    if (std::getline(f_goal, line)) {
        std::istringstream iss(line);
        std::string value;
        int i = 0;
        while (std::getline(iss, value, ',')) {
            if (i == 0) {
                time_step = std::stod(value);
                i++;
            }
            else {
                goal_set[i-1] = std::stod(value);
                i++;
                if (i >= 8) {
                    break;
                }
            }
        }
    }

    f_goal.close();
}

// 从 terrain_set.csv 中读取 terrain_type和terrain_param信息
void ReadTerrainParamsFromCSV(int &terrain_type, double terrain_param[11]) {
    std::ifstream f_terrain("/home/rrqq/towr_workspace/terrain_set.csv");
    if (!f_terrain) {
        std::cout << "Error opening terrain_set file!\n";
        return;
    }

    std::string line;
    if (std::getline(f_terrain, line)) {
        std::istringstream iss(line);
        std::string value;
        int i = 0;
        while (std::getline(iss, value, ',')) {
            if (i == 0) {
                terrain_type = std::stod(value);
                i++;
            }
            else {
                terrain_param[i-1] = std::stod(value);
                i++;
                if (i >= 12) {
                    break;
                }
            }
        }
    }
    f_terrain.close();
}


// TODO：将生成的轨迹写入 CSV 文件
TowrRosInterface::XppVec
TowrRosInterface::GetTrajectory() const
{   
    // 从 goal_set.csv 中读取 timestep和goal信息
    int time_step;
    double goal_set_get[7];
    ReadGoalFromCSV(time_step, goal_set_get);

    // 不从0开始采样数据集
    time_step = time_step + 359;

    // 从 terrain_set.csv 中读取 terrain_type和terrain_param信息
    int terrain_type;
    double terrain_param[8];
    ReadTerrainParamsFromCSV(terrain_type, terrain_param);

    // 落足点轨迹存放目录：f_traj_path = "/home/rrqq/towr_workspace/traj_log/traj_x.csv";
    std::stringstream f_traj_path;
    f_traj_path << "/home/rrqq/towr_workspace/traj_log/traj_" << time_step << ".csv";
    FILE* f_traj = fopen(f_traj_path.str().c_str(), "w");
    if (!(f_traj)) std::cout << "Create traj file failed!\n";
    else std::cout << "Create traj file success!\n";

    // LF, RF, LH, RH 四只脚接触状态
    bool contactLF, contactRF, contactLH, contactRH;
    double x, y, z, qx, qy, qz, qw, wx, wy, wz, wxd, wyd, wzd;

    // 输出轮次 + 目标点位姿
    fprintf(f_traj, "TimeStep, Goal_X, Goal_Y, Goal_Z, Goal_QX, Goal_QY, Goal_QZ, Goal_QW\n");
    // goal_set_get = goal_publisher.GetGoal();
    fprintf(f_traj, "%d, %lf, %lf, %lf, %lf, %lf, %lf, %lf\n", time_step, goal_set_get[0], goal_set_get[1], goal_set_get[2], 
                                                               goal_set_get[3], goal_set_get[4], goal_set_get[5], goal_set_get[6]);
    // printf("@@@@@@ %lf, %lf, %lf, %lf, %lf, %lf, %lf\n", goal_set_get[0], goal_set_get[1], goal_set_get[2], 
    //                                                      goal_set_get[3], goal_set_get[4], goal_set_get[5], goal_set_get[6]);

    // 输出地形类型和地形参数
    fprintf(f_traj, "TerrainType, "
    "step_start, step_height, step_width, "
    "stair_start, stair_height, stair_width, "
    "fissure_start, fissure_width, "
    "down_start, down_height, down_width\n");
    // printf("@@@@@ Terrain params: %lf, %lf, %lf, %lf, %lf, %lf, %lf\n", 
    //        step_start_set, step_height_set, step_width_set, stair_start_set, stair_height_set, stair_width_set, fissure_start_set, fissure_width_set);
    // fprintf(f_traj, "%d, ", TerrainType);
    // fprintf(f_traj, "%lf, %lf, %lf, ", step_start_set, step_height_set, step_width_set);
    // fprintf(f_traj, "%lf, %lf, %lf, ", stair_start_set, stair_height_set, stair_width_set);
    // fprintf(f_traj, "%lf, %lf\n", fissure_start_set, fissure_width_set);
    fprintf(f_traj, "%d, ", terrain_type);
    fprintf(f_traj, "%lf, %lf, %lf, ", terrain_param[0], terrain_param[1], terrain_param[2]);
    fprintf(f_traj, "%lf, %lf, %lf, ", terrain_param[3], terrain_param[4], terrain_param[5]);
    fprintf(f_traj, "%lf, %lf, ", terrain_param[6], terrain_param[7]);
    fprintf(f_traj, "%lf, %lf, %lf\n", terrain_param[8], terrain_param[9], terrain_param[10]);

    // 输出表头
    fprintf(f_traj, "Time, LF_Contact, RF_Contact, LH_Contact, RH_Contact, "
    "LF_X, LF_Y, LF_Z, RF_X, RF_Y, RF_Z, LH_X, LH_Y, LH_Z, RH_X, RH_Y, RH_Z, "
    "Base_X, Base_Y, Base_Z, Base_VX, Base_VY, Base_VZ, Base_AX, Base_AY, Base_AZ, "
    "Base_QX, Base_QY, Base_QZ, Base_QW, "
    "Base_WX, Base_WY, Base_WZ, Base_WXd, Base_WYd, Base_WZd\n");


    XppVec trajectory;
    double t = 0.0;
    double T = solution.base_linear_->GetTotalTime();

    EulerConverter base_angular(solution.base_angular_);

    // 输出落足点位姿序列
    while (t<=T+1e-5) {
        int n_ee = solution.ee_motion_.size();
        xpp::RobotStateCartesian state(n_ee);

        state.base_.lin = ToXpp(solution.base_linear_->GetPoint(t));

        state.base_.ang.q  = base_angular.GetQuaternionBaseToWorld(t);
        state.base_.ang.w  = base_angular.GetAngularVelocityInWorld(t);
        state.base_.ang.wd = base_angular.GetAngularAccelerationInWorld(t);

        for (int ee_towr=0; ee_towr<n_ee; ++ee_towr) {
            int ee_xpp = ToXppEndeffector(n_ee, ee_towr).first;

            state.ee_contact_.at(ee_xpp) = solution.phase_durations_.at(ee_towr)->IsContactPhase(t);
            state.ee_motion_.at(ee_xpp)  = ToXpp(solution.ee_motion_.at(ee_towr)->GetPoint(t));
            state.ee_forces_ .at(ee_xpp) = solution.ee_force_.at(ee_towr)->GetPoint(t).p();
        }

        state.t_global_ = t;

        // 将数据按顺序存入 CSV 文件
        // 四只脚末端接触状态
        contactLF = state.ee_contact_.at(0);
        contactRF = state.ee_contact_.at(1);
        contactLH = state.ee_contact_.at(2);
        contactRH = state.ee_contact_.at(3);
        fprintf(f_traj, "%lf, %d, %d, %d, %d, ", t, contactLF, contactRF, contactLH, contactRH);

        // LF脚末端位置
        x = state.ee_motion_.at(0).p_(0);
        y = state.ee_motion_.at(0).p_(1);
        // 如果是高台地形（裂隙、下行楼梯），则z轴位置加上高台高度
        if ((TerrainType == 3) || (TerrainType == 4)) z = state.ee_motion_.at(0).p_(2) + fissure_height_set;
        else z = state.ee_motion_.at(0).p_(2);
        fprintf(f_traj, "%lf, %lf, %lf, ", x, y, z);

        // RF脚末端位置
        x = state.ee_motion_.at(1).p_(0);
        y = state.ee_motion_.at(1).p_(1);
        // 如果是高台地形（裂隙、下行楼梯），则z轴位置加上高台高度
        if ((TerrainType == 3) || (TerrainType == 4)) z = state.ee_motion_.at(1).p_(2) + fissure_height_set;
        else z = state.ee_motion_.at(1).p_(2);
        fprintf(f_traj, "%lf, %lf, %lf, ", x, y, z);

        // LH脚末端位置
        x = state.ee_motion_.at(2).p_(0);
        y = state.ee_motion_.at(2).p_(1);
        // 如果是高台地形（裂隙、下行楼梯），则z轴位置加上高台高度
        if ((TerrainType == 3) || (TerrainType == 4)) z = state.ee_motion_.at(2).p_(2) + fissure_height_set;
        else z = state.ee_motion_.at(2).p_(2);
        fprintf(f_traj, "%lf, %lf, %lf, ", x, y, z);

        // RH脚末端位置
        x = state.ee_motion_.at(3).p_(0);
        y = state.ee_motion_.at(3).p_(1);
        // 如果是高台地形（裂隙、下行楼梯），则z轴位置加上高台高度
        if ((TerrainType == 3) || (TerrainType == 4)) z = state.ee_motion_.at(3).p_(2) + fissure_height_set;
        else z = state.ee_motion_.at(3).p_(2);
        fprintf(f_traj, "%lf, %lf, %lf, ", x, y, z);

        // 身体位置
        x = state.base_.lin.p_(0);
        y = state.base_.lin.p_(1);
        // 如果是高台地形（裂隙、下行楼梯），则z轴位置加上高台高度
        if ((TerrainType == 3) || (TerrainType == 4)) z = state.base_.lin.p_(2) + fissure_height_set;
        else z = state.base_.lin.p_(2);
        fprintf(f_traj, "%lf, %lf, %lf, ", x, y, z);

        // 身体速度
        x = state.base_.lin.v_(0);
        y = state.base_.lin.v_(1);
        z = state.base_.lin.v_(2);
        fprintf(f_traj, "%lf, %lf, %lf, ", x, y, z);

        // 身体加速度
        x = state.base_.lin.a_(0);
        y = state.base_.lin.a_(1);
        z = state.base_.lin.a_(2);
        fprintf(f_traj, "%lf, %lf, %lf, ", x, y, z);

        // 身体姿态四元数
        qx = state.base_.ang.q.x();
        qy = state.base_.ang.q.y();
        qz = state.base_.ang.q.z();
        qw = state.base_.ang.q.w();
        fprintf(f_traj, "%lf, %lf, %lf, %lf, ", qx, qy, qz, qw);

        // 身体角速度
        wx = state.base_.ang.w.x();
        wy = state.base_.ang.w.y();
        wz = state.base_.ang.w.z();
        fprintf(f_traj, "%lf, %lf, %lf, ", wx, wy, wz);

        // 身体角加速度
        wxd = state.base_.ang.wd.x();
        wyd = state.base_.ang.wd.y();
        wzd = state.base_.ang.wd.z();
        fprintf(f_traj, "%lf, %lf, %lf\n", wxd, wyd, wzd);

        trajectory.push_back(state);
        t += visualization_dt_;
    }

    fclose(f_traj);
    std::cout << "Write traj file success!\n";
    return trajectory;
}

xpp_msgs::RobotParameters
TowrRosInterface::BuildRobotParametersMsg(const RobotModel& model) const
{
    xpp_msgs::RobotParameters params_msg;
    auto max_dev_xyz = model.kinematic_model_->GetMaximumDeviationFromNominal();
    params_msg.ee_max_dev = xpp::Convert::ToRos<geometry_msgs::Vector3>(max_dev_xyz);

    auto nominal_B = model.kinematic_model_->GetNominalStanceInBase();
    int n_ee = nominal_B.size();
    for (int ee_towr=0; ee_towr<n_ee; ++ee_towr) {
        Vector3d pos = nominal_B.at(ee_towr);
        params_msg.nominal_ee_pos.push_back(xpp::Convert::ToRos<geometry_msgs::Point>(pos));
        params_msg.ee_names.push_back(ToXppEndeffector(n_ee, ee_towr).second);
    }

    params_msg.base_mass = model.dynamic_model_->m();

    return params_msg;
}

void
TowrRosInterface::SaveOptimizationAsRosbag (const std::string& bag_name,
                                   const xpp_msgs::RobotParameters& robot_params,
                                   const TowrCommandMsg user_command_msg,
                                   bool include_iterations)
{
    rosbag::Bag bag;
    bag.open(bag_name, rosbag::bagmode::Write);
    ::ros::Time t0(1e-6); // t=0.0 throws ROS exception

    // save the a-priori fixed optimization variables
    bag.write(xpp_msgs::robot_parameters, t0, robot_params);
    bag.write(towr_msgs::user_command+"_saved", t0, user_command_msg);

    // save the trajectory of each iteration
    if (include_iterations) {
        auto trajectories = GetIntermediateSolutions();
        int n_iterations = trajectories.size();
        for (int i=0; i<n_iterations; ++i)
            SaveTrajectoryInRosbag(bag, trajectories.at(i), towr_msgs::nlp_iterations_name + std::to_string(i));

        // save number of iterations the optimizer took
        std_msgs::Int32 m;
        m.data = n_iterations;
        bag.write(towr_msgs::nlp_iterations_count, t0, m);
    }

    // save the final trajectory
    auto final_trajectory = GetTrajectory();
    SaveTrajectoryInRosbag(bag, final_trajectory, xpp_msgs::robot_state_desired);
    //   SaveTrajectoryInCSV(final_trajectory);

    bag.close();
}

void
TowrRosInterface::SaveTrajectoryInRosbag (rosbag::Bag& bag,
                                 const XppVec& traj,
                                 const std::string& topic) const
{
    for (const auto state : traj) {
        auto timestamp = ::ros::Time(state.t_global_ + 1e-6); // t=0.0 throws ROS exception

        xpp_msgs::RobotStateCartesian msg;
        msg = xpp::Convert::ToRos(state);
        bag.write(topic, timestamp, msg);

        xpp_msgs::TerrainInfo terrain_msg;
        for (auto ee : state.ee_motion_.ToImpl()) {
        Vector3d n = formulation_.terrain_->GetNormalizedBasis(HeightMap::Normal, ee.p_.x(), ee.p_.y());
        terrain_msg.surface_normals.push_back(xpp::Convert::ToRos<geometry_msgs::Vector3>(n));
        terrain_msg.friction_coeff = formulation_.terrain_->GetFrictionCoeff();
        }

        bag.write(xpp_msgs::terrain_info, timestamp, terrain_msg);
    }
}


 } /* namespace towr */

