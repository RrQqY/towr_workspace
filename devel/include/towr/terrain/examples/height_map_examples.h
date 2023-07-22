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

#ifndef TOWR_TOWR_ROS_INCLUDE_TOWR_ROS_HEIGHT_MAP_EXAMPLES_H_
#define TOWR_TOWR_ROS_INCLUDE_TOWR_ROS_HEIGHT_MAP_EXAMPLES_H_

#include <towr/terrain/height_map.h>

namespace towr {

/**
 * @addtogroup Terrains
 * @{
 */

/**
 * 平坦地形
 */
class FlatGround : public HeightMap {
public:
  FlatGround(double height = 0.0);
  double GetHeight(double x, double y)  const override { return height_; };

private:
  double height_; // [m]
};

/**
 * @brief Sample terrain with a step in height in x-direction.
 */
class Block : public HeightMap {
public:
  double GetHeight(double x, double y)  const override;
  double GetHeightDerivWrtX(double x, double y) const override;

private:
  double block_start = 0.7;
  double length_     = 3.5;
  double height_     = 0.5; // [m]

  double eps_ = 0.03; // approximate as slope
  const double slope_ = height_/eps_;
};

/**
 * @brief Sample terrain with a two-steps in height in x-direction.
 */
class Stairs : public HeightMap {
public:
  double GetHeight(double x, double y) const override;

private:
  double first_step_start_  = 1.0;
  double first_step_width_  = 0.4;
  double height_first_step  = 0.2;
  double height_second_step = 0.4;
  double width_top = 1.0;
};

/**
 * @brief Sample terrain with parabola-modeled gap in x-direction.
 */
class Gap : public HeightMap {
public:
  double GetHeight(double x, double y) const override;
  double GetHeightDerivWrtX(double x, double y) const override;
  double GetHeightDerivWrtXX(double x, double y) const override;

private:
  const double gap_start_ = 1.0;
  const double w = 0.5;
  const double h = 1.5;

  const double slope_ = h/w;
  const double dx = w/2.0;
  const double xc = gap_start_ + dx; // gap center
  const double gap_end_x = gap_start_ + w;

  // generated with matlab
  // see matlab/gap_height_map.m
  // coefficients of 2nd order polynomial
  // h = a*x^2 + b*x + c
  const double a = (4*h)/(w*w);
  const double b = -(8*h*xc)/(w*w);
  const double c = -(h*(w - 2*xc)*(w + 2*xc))/(w*w);
};

/**
 * @brief Sample terrain with an increasing and then decreasing slope in x-direction.
 */
class Slope : public HeightMap {
public:
  double GetHeight(double x, double y) const override;
  double GetHeightDerivWrtX(double x, double y) const override;

private:
  const double slope_start_ = 1.0;
  const double up_length_   = 1.0;
  const double down_length_ = 1.0;
  const double height_center = 0.7;

  const double x_down_start_ = slope_start_+up_length_;
  const double x_flat_start_ = x_down_start_ + down_length_;
  const double slope_ = height_center/up_length_;
};

/**
 * @brief Sample terrain with a tilted vertical wall to cross a gap.
 */
class Chimney : public HeightMap {
public:
  double GetHeight(double x, double y) const override;
  double GetHeightDerivWrtY(double x, double y) const override;

private:
  const double x_start_ = 1.0;
  const double length_  = 1.5;
  const double y_start_ = 0.5; // distance to start of slope from center at z=0
  const double slope_   = 3.0;

  const double x_end_ = x_start_+length_;
};

/**
 * @brief Sample terrain with two tilted vertical walls to cross a gap.
 */
class ChimneyLR : public HeightMap {
public:
  double GetHeight(double x, double y) const override;
  double GetHeightDerivWrtY(double x, double y) const override;

private:
  const double x_start_ = 0.5;
  const double length_  = 1.0;
  const double y_start_ = 0.5; // distance to start of slope from center at z=0
  const double slope_   = 2;

  const double x_end1_ = x_start_+length_;
  const double x_end2_ = x_start_+2*length_;
};


/* 下面为自定义地形 */
enum TerrainID {FlatID,
                StepID,
                StairID,
                FissureID,
                DownStepID,
                BridgeID,
                PileID,
                TERRAIN_COUNT };
// 地形参数
// 楼梯地形
extern double step_start_set;
extern double step_height_set;
extern double step_width_set;
extern double step_length_set;
extern int    step_num_set;
// 方形台阶地形
extern double stair_start_set;
extern double stair_height_set;
extern double stair_width_set;
extern int    stair_num_set;
// 裂隙地形
extern double fissure_start_set;
extern double fissure_width_set;
extern double fissure_height_set;
// 下楼楼梯地形
extern double down_start_set;
extern double down_height_set;
extern double down_width_set;
extern double down_length_set;
extern int    down_num_set;


/**
 * 楼梯地形
 */
class Step : public HeightMap {
public:
    double GetHeight(double x, double y)  const override;
    double GetHeightDerivWrtX(double x, double y) const override;

    double step_start       = step_start_set;    // 0.5
    double step_length      = step_length_set;
    double step_height      = step_height_set;   // 0.08
    double step_width       = step_width_set;    // 0.8
    int    step_num         = step_num_set;

private:
    double eps_ = 0.01; // approximate as slope
    const double slope_ = step_height/eps_;
};

/**
 * @brief 方形台阶地形
 */
class Stair : public HeightMap {
public:
    double GetHeight(double x, double y) const override;
    double GetHeightDerivWrtX(double x, double y) const override;
    double GetHeightDerivWrtY(double x, double y) const override;

    double stair_start       = stair_start_set;    // 0.5
    double stair_height      = stair_height_set;   // 0.08
    double stair_width       = stair_width_set;    // 1
    int    stair_num         = stair_num_set;

private:
    double eps_ = 0.01; // approximate as slope
    const double slope_ = stair_height/eps_;
};

/**
 * @brief 裂隙地形
 */
class Fissure : public HeightMap {
public:
    double GetHeight(double x, double y) const override;
    double GetHeightDerivWrtX(double x, double y) const override;
    double GetHeightDerivWrtXX(double x, double y) const override;

    double fissure_start = fissure_start_set;    // 0.5
    double fissure_width = fissure_width_set;    // 0.15
    double fissure_height = fissure_height_set;  // 3

private:
    const double slope_ = fissure_height/fissure_width;
    const double dx = fissure_width/2.0;
    const double xc = fissure_start + dx; // gap center

    // generated with matlab
    // see matlab/gap_height_map.m
    // coefficients of 2nd order polynomial
    // h = a*x^2 + b*x + c
    const double a = (4*fissure_height)/(fissure_width*fissure_width);
    const double b = -(8*fissure_height*xc)/(fissure_width*fissure_width);
    const double c = -(fissure_height*(fissure_width - 2*xc)*(fissure_width + 2*xc))/(fissure_width*fissure_width);
};

/**
 * 下楼楼梯地形
 */
class DownStep : public HeightMap {
public:
    double GetHeight(double x, double y)  const override;
    double GetHeightDerivWrtX(double x, double y) const override;

    double down_start       = down_start_set;    // 0.5
    double down_length      = down_length_set;
    double down_height      = down_height_set;   // 0.08
    double down_width       = down_width_set;    // 0.8
    int    down_num         = down_num_set;

private:
    double eps_ = 0.01; // approximate as slope
    const double slope_ = down_height/eps_;
};

/**
 * @brief 桥梁地形
 */
class Bridge : public HeightMap {
public:
    double GetHeight(double x, double y) const override;
    double GetHeightDerivWrtX(double x, double y) const override;
    double GetHeightDerivWrtXX(double x, double y) const override;
    double GetHeightDerivWrtY(double x, double y) const override;

private:
    double bridge_start = 0.5;
    double bridge_width = 0.44;
    double bridge_gap_width = 1.2;
    double bridge_height = 3;

    const double slope_ = bridge_height/bridge_gap_width;
    const double eps_   = 0.03;
    const double slope_y_ = bridge_height/eps_;
    const double dx = bridge_gap_width/2.0;
    const double xc = bridge_start + dx; // gap center

    // generated with matlab
    // see matlab/gap_height_map.m
    // coefficients of 2nd order polynomial
    // h = a*x^2 + b*x + c
    const double a = (4*bridge_height)/(bridge_gap_width*bridge_gap_width);
    const double b = -(8*bridge_height*xc)/(bridge_gap_width*bridge_gap_width);
    const double c = -(bridge_height*(bridge_gap_width - 2*xc)*(bridge_gap_width + 2*xc))/(bridge_gap_width*bridge_gap_width);
};

/**
 * @brief 梅花桩地形
 */
class Pile : public HeightMap {
public:
    double GetHeight(double x, double y) const override;
    double GetHeightDerivWrtX(double x, double y) const override;
    double GetHeightDerivWrtXX(double x, double y) const override;
    double GetHeightDerivWrtY(double x, double y) const override;

private:
    double pile_start = 0.5;
    double pile_width = 0.12;
    double pile_gap_width = 2.0;
    double pile_height = 5;
    double pile_clearance = 0.3;
    int    pile_num = 6;
    double pile_clearance_x = (pile_gap_width - pile_num*pile_width) / (pile_num+1);

    const double slope_ = pile_height/pile_clearance_x;
    const double eps_   = 0.03;
    const double slope_y_ = pile_height/eps_;
    // generated with matlab
    // see matlab/gap_height_map.m
    // coefficients of 2nd order polynomial
    // h = a*x^2 + b*x + c
    const double a = (4*pile_height)/(pile_clearance_x*pile_clearance_x);
    const double b = -(4*pile_height)/(pile_clearance_x);
    const double c = 0;
};

} /* namespace towr */

#endif /* TOWR_TOWR_ROS_INCLUDE_TOWR_ROS_HEIGHT_MAP_EXAMPLES_H_ */
