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

#ifndef TOWR_ROS_USER_INTERFACE_H_
#define TOWR_ROS_USER_INTERFACE_H_

#include <ros/ros.h>

#include <xpp_states/state.h>

namespace towr {


class GoalPublusher {
public:
    double goal_set[7];
    GoalPublusher(){}
    void SetGoal(double goal_pos_x, double goal_pos_y, double goal_pos_z, 
                  double goal_q_x, double goal_q_y, double goal_q_z, double goal_q_w) {
        goal_set[0] = goal_pos_x;
        goal_set[1] = goal_pos_y;
        goal_set[2] = goal_pos_z;
        goal_set[3] = goal_q_x;
        goal_set[4] = goal_q_y;
        goal_set[5] = goal_q_z;
        goal_set[6] = goal_q_w;
        // printf("++++goal_set: %f, %f, %f, %f, %f, %f, %f\n", goal_set[0], goal_set[1], goal_set[2], goal_set[3], goal_set[4], goal_set[5], goal_set[6]);
    }

    double* GetGoal() {
        // printf("goal_set: %f, %f, %f, %f, %f, %f, %f\n", 
        // goal_set[0], goal_set[1], goal_set[2], goal_set[3], goal_set[4], goal_set[5], goal_set[6]);
        return goal_set;
    }

    void SetGoalToCSV(int time_step, double goal_pos_x, double goal_pos_y, double goal_pos_z, 
                      double goal_q_x, double goal_q_y, double goal_q_z, double goal_q_w) {
        FILE* f_goal = fopen("/home/rrqq/towr_workspace/goal_set.csv", "w");
        // if (!(f_goal)) std::cout << "Create goal_set file failed!\n";
        // else std::cout << "Create goal_set file success!\n";

        goal_set[0] = goal_pos_x;
        goal_set[1] = goal_pos_y;
        goal_set[2] = goal_pos_z;
        goal_set[3] = goal_q_x;
        goal_set[4] = goal_q_y;
        goal_set[5] = goal_q_z;
        goal_set[6] = goal_q_w;

        fprintf(f_goal, "%d, %lf, %lf, %lf, %lf, %lf, %lf, %lf\n", 
                time_step, goal_set[0], goal_set[1], goal_set[2], goal_set[3], goal_set[4], goal_set[5], goal_set[6]);
        
        fclose(f_goal);
        // std::cout << "Write goal_set file success!\n";
    }

    void SetTerrainParamsToCSV(int terrain, double step_start, double step_height, double step_width, 
                                            double stair_start, double stair_height, double stair_width,
                                            double fissure_start_set, double fissure_width,
                                            double down_start, double down_height, double down_width) {
        FILE* f_terrain = fopen("/home/rrqq/towr_workspace/terrain_set.csv", "w");
        fprintf(f_terrain, "%d, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf\n", 
                terrain, step_start, step_height, step_width, 
                         stair_start, stair_height, stair_width, 
                         fissure_start_set, fissure_width,
                         down_start, down_height, down_width);
        
        fclose(f_terrain);
    }
};

extern GoalPublusher goal_publisher;

// 地形参数采样函数
extern void SampleTerrainParameters(int &terrain);

/**
 * @brief Translates user input into the ROS message TowrCommand.msg.
 *
 * This includes high level input about where to go (e.g. converting
 * keyboard input into a goal state), which terrain to visualize, etc.
 */
class TowrUserInterface {
public:
    /**
     * @brief  Constructs default object to interact with framework.
     */
    TowrUserInterface ();
    virtual ~TowrUserInterface () = default;

    /**
     * Called whenever a keyboard key is pressed.
     * @param c  Unicode character of that key (see ncurses library).
     */
    void CallbackKey(int c);

    xpp::State3dEuler goal_geom_;
    int terrain_;
    int gait_combo_;
    int robot_;
    bool visualize_trajectory_;
    bool play_initialization_;
    double replay_speed_;
    bool plot_trajectory_;
    bool optimize_;
    bool publish_optimized_trajectory_;
    double total_duration_;
    bool optimize_phase_durations_;

    void PublishCommand(int time_step);
    int AdvanceCircularBuffer(int& curr, int max) const;

private:
    ::ros::Publisher  user_command_pub_; ///< the output message to TOWR.


    //   void PublishCommand();

    //   xpp::State3dEuler goal_geom_;
    //   int terrain_;
    //   int gait_combo_;
    //   int robot_;
    //   bool visualize_trajectory_;
    //   bool play_initialization_;
    //   double replay_speed_;
    //   bool plot_trajectory_;
    //   bool optimize_;
    //   bool publish_optimized_trajectory_;
    //   double total_duration_;
    //   bool optimize_phase_durations_;

    //   int AdvanceCircularBuffer(int& curr, int max) const;

    void PrintVector(const Eigen::Vector3d& v) const;
    void PrintVector2D(const Eigen::Vector2d& v) const;
    void PrintScreen() const;
};

} /* namespace towr */

#endif /* TOWR_ROS_USER_INTERFACE_H_ */
