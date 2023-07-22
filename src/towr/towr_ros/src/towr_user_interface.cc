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

#include <towr_ros/towr_user_interface.h>

#include <ncurses.h>
#include <math.h>
#include <random>
#include <chrono>
#include <thread>
#include <fstream>

#include <xpp_states/convert.h>

#include <towr_ros/TowrCommand.h>
#include <towr_ros/topic_names.h>
#include <towr/terrain/height_map.h>
#include <towr/terrain/examples/height_map_examples.h>
#include <towr/initialization/gait_generator.h>
#include <towr/models/robot_model.h>
#include <towr_ros/towr_ros_interface.h>


// extern double goal_pos_x;
// extern double goal_pos_y;
// extern double goal_pos_z;
// extern double goal_q_x;
// extern double goal_q_y;
// extern double goal_q_z;
// extern double goal_q_w;


namespace towr {

GoalPublusher goal_publisher;

enum YCursorRows {HEADING=6, OPTIMIZE=8, VISUALIZE, INITIALIZATION, PLOT,
                  REPLAY_SPEED, GOAL_POS, GOAL_ORI, ROBOT,
                  GAIT, OPTIMIZE_GAIT, TERRAIN, DURATION, CLOSE, END};
static constexpr int Y_STATUS      = END+1;
static constexpr int X_KEY         = 1;
static constexpr int X_DESCRIPTION = 10;
static constexpr int X_VALUE       = 35;


TowrUserInterface::TowrUserInterface ()
{   
    // printw(" ******************************************************************************\n");
    // printw("                          TOWR user interface (v1.4) \n");
    // printw("                            \u00a9 Alexander W. Winkler \n");
    // printw("                        https://github.com/ethz-adrl/towr\n");
    // printw(" ******************************************************************************\n\n");

    ::ros::NodeHandle n;
    user_command_pub_ = n.advertise<towr_ros::TowrCommand>(towr_msgs::user_command, 1);

    goal_geom_.lin.p_.setZero();
    goal_geom_.lin.p_ << 3.2, 0.0, 0.0;
    goal_geom_.ang.p_ << 0.0, 0.0, 0.0; // roll, pitch, yaw angle applied Z->Y'->X''

    robot_      = RobotModel::JueYing;
    terrain_    = HeightMap::FlatID;
    gait_combo_ = GaitGenerator::C0;
    total_duration_ = 4.0;
    //   total_duration_ = 2.4;
    visualize_trajectory_ = false;
    plot_trajectory_ = false;
    replay_speed_ = 1.0; // realtime
    // optimize_ = true;
    optimize_ = false;
    publish_optimized_trajectory_ = false;
    optimize_phase_durations_ = false;
    
    // 如果是自动运行（非键盘输入参数）则下面这行要注释掉
    // PrintScreen();
}

void
TowrUserInterface::PrintScreen() const
{
    wmove(stdscr, HEADING, X_KEY);
    printw("Key");
    wmove(stdscr, HEADING, X_DESCRIPTION);
    printw("Description");
    wmove(stdscr, HEADING, X_VALUE);
    printw("Info");

    wmove(stdscr, OPTIMIZE, X_KEY);
    printw("o");
    wmove(stdscr, OPTIMIZE, X_DESCRIPTION);
    printw("Optimize motion");
    wmove(stdscr, OPTIMIZE, X_VALUE);
    printw("-");

    wmove(stdscr, VISUALIZE, X_KEY);
    printw("v");
    wmove(stdscr, VISUALIZE, X_DESCRIPTION);
    printw("visualize motion in rviz");
    wmove(stdscr, VISUALIZE, X_VALUE);
    printw("-");

    wmove(stdscr, INITIALIZATION, X_KEY);
    printw("i");
    wmove(stdscr, INITIALIZATION, X_DESCRIPTION);
    printw("play initialization");
    wmove(stdscr, INITIALIZATION, X_VALUE);
    printw("-");

    wmove(stdscr, PLOT, X_KEY);
    printw("p");
    wmove(stdscr, PLOT, X_DESCRIPTION);
    printw("Plot values (rqt_bag)");
    wmove(stdscr, PLOT, X_VALUE);
    printw("-");

    wmove(stdscr, REPLAY_SPEED, X_KEY);
    printw(";/'");
    wmove(stdscr, REPLAY_SPEED, X_DESCRIPTION);
    printw("Replay speed");
    wmove(stdscr, REPLAY_SPEED, X_VALUE);
    printw("%.2f", replay_speed_);

    wmove(stdscr, GOAL_POS, X_KEY);
    printw("arrows");
    wmove(stdscr, GOAL_POS, X_DESCRIPTION);
    printw("Goal x-y");
    wmove(stdscr, GOAL_POS, X_VALUE);
    PrintVector2D(goal_geom_.lin.p_.topRows(2));
    printw(" [m]");

    wmove(stdscr, GOAL_ORI, X_KEY);
    printw("keypad");
    wmove(stdscr, GOAL_ORI, X_DESCRIPTION);
    printw("Goal r-p-y");
    wmove(stdscr, GOAL_ORI, X_VALUE);
    PrintVector(goal_geom_.ang.p_);
    printw(" [rad]");

    wmove(stdscr, ROBOT, X_KEY);
    printw("r");
    wmove(stdscr, ROBOT, X_DESCRIPTION);
    printw("Robot");
    wmove(stdscr, ROBOT, X_VALUE);
    printw("%s\n", robot_names.at(static_cast<RobotModel::Robot>(robot_)).c_str());

    wmove(stdscr, GAIT, X_KEY);
    printw("g");
    wmove(stdscr, GAIT, X_DESCRIPTION);
    printw("Gait");
    wmove(stdscr, GAIT, X_VALUE);
    printw("%s", std::to_string(gait_combo_).c_str());

    wmove(stdscr, OPTIMIZE_GAIT, X_KEY);
    printw("y");
    wmove(stdscr, OPTIMIZE_GAIT, X_DESCRIPTION);
    printw("Optimize gait");
    wmove(stdscr, OPTIMIZE_GAIT, X_VALUE);
    optimize_phase_durations_? printw("On\n") : printw("off\n");

    wmove(stdscr, TERRAIN, X_KEY);
    printw("t");
    wmove(stdscr, TERRAIN, X_DESCRIPTION);
    printw("Terrain");
    wmove(stdscr, TERRAIN, X_VALUE);
    printw("%s\n", terrain_names.at(static_cast<HeightMap::TerrainID>(terrain_)).c_str());

    wmove(stdscr, DURATION, X_KEY);
    printw("+/-");
    wmove(stdscr, DURATION, X_DESCRIPTION);
    printw("Duration");
    wmove(stdscr, DURATION, X_VALUE);
    printw("%.2f [s]", total_duration_);

    wmove(stdscr, CLOSE, X_KEY);
    printw("q");
    wmove(stdscr, CLOSE, X_DESCRIPTION);
    printw("Close user interface");
    wmove(stdscr, CLOSE, X_VALUE);
    printw("-");
}

void
TowrUserInterface::CallbackKey (int c)
{
    const static double d_lin = 0.1;  // [m]
    const static double d_ang = 0.25; // [rad]

    switch (c) {
        case KEY_RIGHT:
        goal_geom_.lin.p_.x() -= d_lin;
        break;
        case KEY_LEFT:
        goal_geom_.lin.p_.x() += d_lin;
        break;
        case KEY_DOWN:
        goal_geom_.lin.p_.y() += d_lin;
        break;
        case KEY_UP:
        goal_geom_.lin.p_.y() -= d_lin;
        break;
        case KEY_PPAGE:
        goal_geom_.lin.p_.z() += 0.5*d_lin;
        break;
        case KEY_NPAGE:
        goal_geom_.lin.p_.z() -= 0.5*d_lin;
        break;

        // desired goal orientations
        case '4':
        goal_geom_.ang.p_.x() -= d_ang; // roll-
        break;
        case '6':
        goal_geom_.ang.p_.x() += d_ang; // roll+
        break;
        case '8':
        goal_geom_.ang.p_.y() += d_ang; // pitch+
        break;
        case '2':
        goal_geom_.ang.p_.y() -= d_ang; // pitch-
        break;
        case '1':
        goal_geom_.ang.p_.z() += d_ang; // yaw+
        break;
        case '9':
        goal_geom_.ang.p_.z() -= d_ang; // yaw-
        break;

        // terrains
        case 't':
        terrain_ = AdvanceCircularBuffer(terrain_, HeightMap::TERRAIN_COUNT);
        break;

        case 'g':
        gait_combo_ = AdvanceCircularBuffer(gait_combo_, GaitGenerator::COMBO_COUNT);
        break;

        case 'r':
        robot_ = AdvanceCircularBuffer(robot_, RobotModel::ROBOT_COUNT);
        break;

        // duration
        case '+':
        total_duration_ += 0.2;
        break;
        case '-':
        total_duration_ -= 0.2;
        break;
        case '\'':
        replay_speed_ += 0.1;
        break;
        case ';':
        replay_speed_ -= 0.1;
        break;
        case 'y':
        optimize_phase_durations_ = !optimize_phase_durations_;
        break;


        case 'o':
        optimize_ = true;
        wmove(stdscr, Y_STATUS, 0);
        printw("Optimizing motion\n\n");
        break;
        case 'v':
        visualize_trajectory_ = true;
        wmove(stdscr, Y_STATUS, 0);
        printw("Visualizing current bag file\n\n");
        break;
        case 'i':
        play_initialization_ = true;
        wmove(stdscr, Y_STATUS, 0);
        printw("Visualizing initialization (iteration 0)\n\n");
        break;
        case 'p':
        plot_trajectory_ = true;
        wmove(stdscr, Y_STATUS, 0);
        printw("In rqt_bag: right-click on xpp/state_des -> View -> Plot.\n"
                "Then expand the values you wish to plot on the right\n");
        break;
        case 'q':
        printw("Closing user interface\n");
        ros::shutdown(); break;
        default:
        break;
    }

    PublishCommand(0);
}


void TowrUserInterface::PublishCommand(int time_step)
{
    towr_ros::TowrCommand msg;
    msg.goal_lin                 = xpp::Convert::ToRos(goal_geom_.lin);
    msg.goal_ang                 = xpp::Convert::ToRos(goal_geom_.ang);
    msg.total_duration           = total_duration_;
    msg.replay_trajectory        = visualize_trajectory_;
    msg.play_initialization      = play_initialization_;
    msg.replay_speed             = replay_speed_;
    msg.optimize                 = optimize_;
    msg.terrain                  = terrain_;
    msg.gait                     = gait_combo_;
    msg.robot                    = robot_;
    msg.optimize_phase_durations = optimize_phase_durations_;
    msg.plot_trajectory          = plot_trajectory_;

    user_command_pub_.publish(msg);

    Eigen::Quaterniond q = xpp::GetQuaternionFromEulerZYX(msg.goal_ang.pos.z, msg.goal_ang.pos.y, msg.goal_ang.pos.x);
    auto terrain_id = static_cast<HeightMap::TerrainID>(msg.terrain);
    auto Terrain = HeightMap::MakeTerrain(terrain_id);
    towr::goal_publisher.SetGoalToCSV(time_step, msg.goal_lin.pos.x, msg.goal_lin.pos.y, Terrain->GetHeight(msg.goal_lin.pos.x, msg.goal_lin.pos.y), 
                                      q.x(), q.y(), q.z(), q.w());

    // 如果是自动运行（非键盘输入参数）则下面这行要注释掉
    // PrintScreen();

    optimize_ = false;
    visualize_trajectory_ = false;
    plot_trajectory_ = false;
    play_initialization_ = false;
    publish_optimized_trajectory_ = false;

}

int TowrUserInterface::AdvanceCircularBuffer(int& curr, int max) const
{
  return curr==(max-1)? 0 : curr+1;
}

void
TowrUserInterface::PrintVector(const Eigen::Vector3d& v) const
{
  printw("%.2f  %.2f  %.2f", v.x(), v.y(), v.z());
}

void
TowrUserInterface::PrintVector2D(const Eigen::Vector2d& v) const
{
  printw("%.2f  %.2f", v.x(), v.y());
}


// TODO：地形类型及对应参数采样函数
void SampleTerrainParameters(int &terrain) {
    // Random number generator
    std::random_device rd;
    std::mt19937 gen(rd());

    // 生成地形类型
    std::uniform_int_distribution<> terrain_type(0, 4);
    // terrain = 0;
    terrain = terrain_type(gen);
    printf("@@@@@ Select Terrain: %d\n", terrain);
    auto terrain_id = static_cast<towr::HeightMap::TerrainID>(terrain);

    switch (terrain_id) {
        case StepID:
            // Step terrain parameter ranges (you can adjust these as needed)
            step_start_set = std::normal_distribution<>(0.5, 0.03)(gen);
            step_height_set = std::normal_distribution<>(0.07, 0.03)(gen);
            step_width_set = std::normal_distribution<>(0.7, 0.03)(gen);
            break;
        case StairID:
            // Stair terrain parameter ranges (you can adjust these as needed)
            stair_start_set = std::normal_distribution<>(0.5, 0.03)(gen);
            stair_height_set = std::normal_distribution<>(0.08, 0.03)(gen);
            stair_width_set = std::normal_distribution<>(1.0, 0.03)(gen);
            break;
        case FissureID:
            // Fissure terrain parameter ranges (you can adjust these as needed)
            fissure_start_set = std::normal_distribution<>(1.0, 0.03)(gen);
            fissure_width_set = std::normal_distribution<>(0.15, 0.03)(gen);
            break;
        case DownStepID:
            // Step terrain parameter ranges (you can adjust these as needed)
            down_start_set = std::normal_distribution<>(0.5, 0.03)(gen);
            down_height_set = std::normal_distribution<>(0.07, 0.03)(gen);
            down_width_set = std::normal_distribution<>(0.7, 0.03)(gen);
            break;
        default:
            break;
    }

    towr::goal_publisher.SetTerrainParamsToCSV(terrain, 
                                               step_start_set, step_height_set, step_width_set, 
                                               stair_start_set, stair_height_set, stair_width_set,
                                               fissure_start_set, fissure_width_set,
                                               down_start_set, down_height_set, down_width_set);
}


// TODO：目标点位姿采样函数
void SampleGoalParameters (towr::HeightMap::TerrainID terrain_id,
                           double &pos_x, double &pos_y, double &pos_z, 
                           double &ang_x, double &ang_y, double &ang_z) {
    // 随机数生成器
    std::random_device rd;
    std::mt19937 gen(rd());

    // 只对位置x、y轴和z轴角度采样
    pos_z = 0.0;
    ang_x = 0.0;
    ang_y = 0.0;
    // 通用地形范围
    double pos_x_min = 0.8;
    double pos_x_max = 4.0;
    double pos_y_min = -3.0;
    double pos_y_max = 3.0;
    double ang_min = -M_PI * 2.0/3.0;
    double ang_max = M_PI * 2.0/3.0;

    // 生成地形
    auto Terrain = towr::HeightMap::MakeTerrain(terrain_id);

    // 根据地形分别采样
    switch (terrain_id) {
        case FlatID: {
            double flat_x_mean = (pos_x_min + pos_x_max) / 2.0;
            double flat_x_stddev = (pos_x_max - pos_x_min) / 6.0;
            std::normal_distribution<double> flat_pos_x_dist(flat_x_mean, flat_x_stddev);
            // std::uniform_real_distribution<double> pos_y_dist(pos_y_min, pos_y_max);
            std::normal_distribution<double> flat_pos_y_dist(0.0, pos_y_max/6.0);
            // std::uniform_real_distribution<double> ang_z_dist(ang_min, ang_max);
            std::normal_distribution<double> flat_ang_z_dist(0.0, ang_max/6.0);
            pos_x = flat_pos_x_dist(gen);
            pos_y = flat_pos_y_dist(gen);
            ang_z = flat_ang_z_dist(gen);
            break;}

        case StepID: {
            // 先随机指定一层楼梯
            std::uniform_int_distribution<> step_terrain_dist(1, step_num_set);
            int selected_step = step_terrain_dist(gen);

            // 获取该层楼梯的x、y轴范围
            double step_x_min = step_start_set + step_width_set * (selected_step - 1);
            double step_x_max = step_start_set + step_width_set * (selected_step);
            double step_y_min = -step_length_set/2.0;
            double step_y_max = step_length_set/2.0;

            // 在该层楼梯x轴中点为均值进行正态分布采样pos_x
            // 以0为均值正态分布采样pos_y、ang_z
            double step_x_mean = (step_x_min + step_x_max) / 2.0;
            double step_x_stddev = (step_x_max - step_x_min) / 6.0;
            std::normal_distribution<double> step_pos_x_dist(step_x_mean, step_x_stddev);
            std::normal_distribution<double> step_pos_y_dist(0.0, pos_y_max/6.0);
            // std::uniform_real_distribution<double> ang_z_dist(ang_min, ang_max);
            std::normal_distribution<double> step_ang_z_dist(0.0, ang_max/6.0);
            pos_x = step_pos_x_dist(gen);
            pos_y = step_pos_y_dist(gen);
            ang_z = step_ang_z_dist(gen);
            break;}

        case StairID: {
            // 先随机指定一层楼梯
            std::uniform_int_distribution<> stair_terrain_dist_x(1, stair_num_set);
            std::uniform_int_distribution<> stair_terrain_dist_y(0, stair_num_set-2);
            std::uniform_int_distribution<> stair_terrain_dist_y_orient(0, 1);   // y轴左侧0或右侧1
            int selected_stair_x = stair_terrain_dist_x(gen);
            int selected_stair_y = stair_terrain_dist_y(gen);
            int selected_stair_y_orient = stair_terrain_dist_y_orient(gen);

            // 获取该层楼梯的x、y轴范围
            double stair_x_min = stair_start_set + stair_width_set * (selected_stair_x - 1);
            double stair_x_max = stair_start_set + stair_width_set * (selected_stair_x);
            double stair_y_min, stair_y_max = 0.0;
            if (selected_stair_y_orient == 0) {
                stair_y_min = stair_width_set * (selected_stair_y - 1);
                stair_y_max = stair_width_set * (selected_stair_y);
            } else if (selected_stair_y_orient == 1) {
                stair_y_min = -stair_width_set * (selected_stair_y);
                stair_y_max = -stair_width_set * (selected_stair_y - 1);
            }
            
            // 在该层台阶x、y轴中点为均值进行正态分布采样pos_x、pos_y
            // 均匀分布采样ang_z
            double stair_x_mean = (stair_x_min + stair_x_max) / 2.0;
            double stair_x_stddev = (stair_x_max - stair_x_min) / 6.0;
            double stair_y_mean = (stair_y_min + stair_y_max) / 2.0;
            double stair_y_stddev = (stair_y_max - stair_y_min) / 6.0;
            std::normal_distribution<double> stair_pos_x_dist(stair_x_mean, stair_x_stddev);
            std::normal_distribution<double> stair_pos_y_dist(stair_y_mean, stair_y_stddev);
            // std::uniform_real_distribution<double> ang_z_dist(ang_min, ang_max);
            std::normal_distribution<double> stair_ang_z_dist(0.0, ang_max/6.0);
            pos_x = stair_pos_x_dist(gen);
            pos_y = stair_pos_y_dist(gen);
            ang_z = stair_ang_z_dist(gen);
            break;}

        case FissureID: {
            double fissure_threshold = 0.8;     // 开始采样目标点到边缘的距离
            double fissure_x_min = fissure_start_set + fissure_width_set + fissure_threshold;
            double fissure_x_max = pos_x_max;
            double fissure_x_mean = (fissure_x_min + fissure_x_max) / 2.0;
            double fissure_x_stddev = (fissure_x_max - fissure_x_min) / 6.0;
            std::normal_distribution<double> fissure_pos_x_dist(fissure_x_mean, fissure_x_stddev);
            std::normal_distribution<double> fissure_pos_y_dist(0.0, pos_y_max/6.0);
            std::normal_distribution<double> fissure_ang_z_dist(0.0, ang_max/6.0);
            pos_x = fissure_pos_x_dist(gen);
            pos_y = fissure_pos_y_dist(gen);
            ang_z = fissure_ang_z_dist(gen);
            break;}

        case DownStepID: {
            // 先随机指定一层楼梯
            std::uniform_int_distribution<> down_terrain_dist(1, down_num_set);
            int selected_step = down_terrain_dist(gen);

            // 获取该层楼梯的x、y轴范围
            double down_x_min = down_start_set + down_width_set * (selected_step - 1);
            double down_x_max = down_start_set + down_width_set * (selected_step);
            double down_y_min = -down_length_set/2.0;
            double down_y_max = down_length_set/2.0;

            // 在该层楼梯x轴中点为均值进行正态分布采样pos_x
            // 以0为均值正态分布采样pos_y、ang_z
            double down_x_mean = (down_x_min + down_x_max) / 2.0;
            double down_x_stddev = (down_x_max - down_x_min) / 6.0;
            std::normal_distribution<double> down_pos_x_dist(down_x_mean, down_x_stddev);
            std::normal_distribution<double> down_pos_y_dist(0.0, pos_y_max/6.0);
            // std::uniform_real_distribution<double> ang_z_dist(ang_min, ang_max);
            std::normal_distribution<double> down_ang_z_dist(0.0, ang_max/6.0);
            pos_x = down_pos_x_dist(gen);
            pos_y = down_pos_y_dist(gen);
            ang_z = down_ang_z_dist(gen);
            break;}

        default: {
            break;}
    }
}

} /* namespace towr */


// TODO：以一定间隔自动采样并生成落足点序列
int main(int argc, char* argv[])
{
    ros::init(argc, argv, "towr_user_interface");
    printf("@@@@@ Start auto sampling!\n");
    // printf("----------------------------------");

    // ros::Rate loop_rate(1.0); // 1/120 Hz 发布频率
    // 每次发布新指令的等待时间
    int step_duration_time = 60;
    // 每种地形的数据集数量
    int des_terrain_data_num = 144;
    int terrain_data_num[5] = {0};

    towr::TowrUserInterface user_interface;

    int time_step = 0;           // 第一次规划无效，因此从0开始

    while (ros::ok()) {
        printf("@@@@@ time_step = %d\n", time_step);

        // 获取当前时间
        auto current_time_start = std::chrono::system_clock::now();
        std::time_t time_start = std::chrono::system_clock::to_time_t(current_time_start);

        // 正态分布采样：地形参数
        int terrain;         // 采样：地形类别（0平坦 1楼梯 2方形台阶 3裂隙）
        towr::SampleTerrainParameters(terrain);
        // 如果该种地形类型的数据已经到达期望值，则重新采样
        while (terrain_data_num[terrain] >= des_terrain_data_num) {
            towr::SampleTerrainParameters(terrain);
            printf("@@@@@ Repetitive Terrain Type!\n");
        }
        if (time_step >= 1){
            terrain_data_num[terrain] ++;
        }

        printf("@@@@@ Terrain Data Num: %d, %d, %d, %d, %d\n", terrain_data_num[0], terrain_data_num[1], terrain_data_num[2], 
                                                               terrain_data_num[3], terrain_data_num[4]);

        // 生成地形
        towr::HeightMap::TerrainID terrain_id = static_cast<towr::HeightMap::TerrainID>(terrain);
        towr::HeightMap::Ptr Terrain = towr::HeightMap::MakeTerrain(terrain_id);

        // 正态分布采样：目标点位置和姿态
        double pos_x, pos_y, pos_z, ang_x, ang_y, ang_z;
        towr::SampleGoalParameters(terrain_id, pos_x, pos_y, pos_z, ang_x, ang_y, ang_z);
        Eigen::Quaterniond q = xpp::GetQuaternionFromEulerZYX(ang_z, ang_y, ang_x);

        // 将采样的目标点和地形类型存入文件中
        towr::goal_publisher.SetGoal(pos_x, pos_y, Terrain->GetHeight(pos_x, pos_y), 
                                     q.x(), q.y(), q.z(), q.w());

        // 根据起点到目标点的二维距离粗略估计所需总时间
        int total_duration;
        total_duration = floor(sqrt(pos_x*pos_x + pos_y*pos_y)) + 1;


        // 设置参数内容
        user_interface.goal_geom_.lin.p_.x() = pos_x;
        user_interface.goal_geom_.lin.p_.y() = pos_y;
        user_interface.goal_geom_.lin.p_.z() = pos_z;

        user_interface.goal_geom_.ang.p_.x() = ang_x;
        user_interface.goal_geom_.ang.p_.y() = ang_y;
        user_interface.goal_geom_.ang.p_.z() = ang_z;

        user_interface.terrain_ = terrain;                // 地形类型
        user_interface.total_duration_ = total_duration;   // 所需时间

        user_interface.optimize_ = true;
        user_interface.visualize_trajectory_ = true;
        user_interface.plot_trajectory_ = true;
        user_interface.publish_optimized_trajectory_ = true;

        // 发布消息
        user_interface.PublishCommand(time_step);

        time_step ++;

        // 计算剩余的等待时间并进行延迟
        auto current_time_end = std::chrono::system_clock::now();
        std::time_t time_end = std::chrono::system_clock::to_time_t(current_time_end);
        std::time_t time_left = step_duration_time - (time_end - time_start);
        if (time_left > 0) std::this_thread::sleep_for(std::chrono::seconds(time_left));
        else std::cout << "Warning: Loop execution took longer than " << step_duration_time
                       << " seconds. Consider increasing the step_duration_time.\n";

        // 等本轮其余进程结束后，如果所有地形的数据都已经到达期望值，则退出
        if (terrain_data_num[0] >= des_terrain_data_num && terrain_data_num[1] >= des_terrain_data_num && 
            terrain_data_num[2] >= des_terrain_data_num && terrain_data_num[3] >= des_terrain_data_num && 
            terrain_data_num[4] >= des_terrain_data_num) {
            printf("@@@@@ All terrain data have been sampled!\n");
            printf("@@@@@ Sample Finished!\n");
            break;
        }
        // ros::spinOnce();
        // loop_rate.sleep();
    }

    return 1;
}


// int main(int argc, char *argv[])
// {
//     ros::init(argc, argv, "towr_user_interface");
//     ROS_INFO_STREAM("Start auto sampling!");
//     // printf("----------------------------------");

//     initscr();
//     cbreak();              // disables buffering of types characters
//     noecho();              // suppresses automatic output of typed characters
//     keypad(stdscr, TRUE);  // to capture special keypad characters

//     towr::TowrUserInterface keyboard_user_interface;

//     while (ros::ok())
//     {
//         int c = getch(); // call your non-blocking input function
//         keyboard_user_interface.CallbackKey(c);
//         refresh();
//     }

//     endwin();

//     return 1;
// }