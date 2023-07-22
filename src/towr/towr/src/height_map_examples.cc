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

#include <towr/terrain/examples/height_map_examples.h>

namespace towr {

// 地形设置参数
// 楼梯地形
double step_start_set = 0.5;
double step_height_set = 0.07;
double step_width_set = 0.7;
double step_length_set = 2.0;
int    step_num_set = 4;
// 方形台阶地形
double stair_start_set = 0.5;
double stair_height_set = 0.08;
double stair_width_set = 1.0;
int    stair_num_set = 4;
// 裂隙地形
double fissure_start_set = 0.5;
double fissure_width_set = 0.15;
double fissure_height_set = 3.0;
// 下楼楼梯地形
double down_start_set = 0.5;
double down_height_set = 0.07;
double down_width_set = 0.7;
double down_length_set = 2.0;
int    down_num_set = 4;


FlatGround::FlatGround(double height)
{
  height_ = height;
}

double
Block::GetHeight (double x, double y) const
{
  double h = 0.0;

  // very steep ramp leading up to block
  if (block_start <= x && x <=block_start+eps_)
    h = slope_*(x-block_start);

  if (block_start+eps_ <= x && x <= block_start+length_)
    h = height_;

  return h;
}

double
Block::GetHeightDerivWrtX (double x, double y) const
{
  double dhdx = 0.0;

  // very steep ramp leading up to block
  if (block_start <= x && x <=block_start+eps_)
    dhdx = slope_;

  return dhdx;
}


// STAIRS
double
Stairs::GetHeight (double x, double y) const
{
  double h = 0.0;

  if (x>=first_step_start_)
    h = height_first_step;

  if (x>=first_step_start_+first_step_width_)
    h = height_second_step;

  if (x>=first_step_start_+first_step_width_+width_top)
    h = 0.0;

  return h;
}


// GAP
double
Gap::GetHeight (double x, double y) const
{
  double h = 0.0;

  // modelled as parabola
  if (gap_start_ <= x && x <= gap_end_x)
    h = a*x*x + b*x + c;

  return h;
}

double
Gap::GetHeightDerivWrtX (double x, double y) const
{
  double dhdx = 0.0;

  if (gap_start_ <= x && x <= gap_end_x)
    dhdx = 2*a*x + b;

  return dhdx;
}

double
Gap::GetHeightDerivWrtXX (double x, double y) const
{
  double dzdxx = 0.0;

  if (gap_start_ <= x && x <= gap_end_x)
    dzdxx = 2*a;

  return dzdxx;
}


// SLOPE
double
Slope::GetHeight (double x, double y) const
{
  double z = 0.0;
  if (x >= slope_start_)
    z = slope_*(x-slope_start_);

  // going back down
  if (x >= x_down_start_) {
    z = height_center - slope_*(x-x_down_start_);
  }

  // back on flat ground
  if (x >= x_flat_start_)
    z = 0.0;

  return z;
}

double
Slope::GetHeightDerivWrtX (double x, double y) const
{
  double dzdx = 0.0;
  if (x >= slope_start_)
    dzdx = slope_;

  if (x >= x_down_start_)
    dzdx = -slope_;

  if (x >= x_flat_start_)
    dzdx = 0.0;

  return dzdx;
}


// Chimney
double
Chimney::GetHeight (double x, double y) const
{
  double z = 0.0;

  if (x_start_<=x && x<=x_end_)
    z = slope_*(y-y_start_);

  return z;
}

double
Chimney::GetHeightDerivWrtY (double x, double y) const
{
  double dzdy = 0.0;

  if (x_start_<= x && x<= x_end_)
    dzdy = slope_;

  return dzdy;
}


// Chimney LR
double
ChimneyLR::GetHeight (double x, double y) const
{
  double z = 0.0;

  if (x_start_<=x && x<=x_end1_)
    z = slope_*(y-y_start_);

  if (x_end1_<=x && x<=x_end2_)
    z = -slope_*(y+y_start_);

  return z;
}

double
ChimneyLR::GetHeightDerivWrtY (double x, double y) const
{
  double dzdy = 0.0;

  if (x_start_ <= x && x <= x_end1_)
    dzdy = slope_;

  if (x_end1_<=x && x<=x_end2_)
    dzdy = -slope_;

  return dzdy;
}


/* 下面为自定义地形 */

// 楼梯地形
double
Step::GetHeight (double x, double y) const
{
    double h = 0.0;
    // 获得高度
    for (int i=0; i<step_num; i++) {
        // very steep ramp leading up to block
        // if (step_start+i*step_width <= x && x <=step_start+i*step_width+eps_){
        //     h = i*step_height + slope_*(x-step_start-i*step_width);
        //     break;
        // }
        if (step_start+i*step_width+eps_ <= x && x <= step_start+(i+1)*step_width){
            h = (i+1)*step_height;
            break;
        }
    }
    // 如果超过左右两侧宽度，则高度为0
    if (y < -step_length || y > step_length){
        h = 0.0;
    }
    return h;
}

double
Step::GetHeightDerivWrtX (double x, double y) const
{
    double dzdx = 0.0;
    // for (int i=0; i<step_num; i++) {
    //     // very steep ramp leading up to block
    //     if (step_start+i*step_width <= x && x <=step_start+i*step_width+eps_){
    //         dzdx = slope_;
    //         break;
    //     }
    // }
    // // 如果超过左右两侧宽度，则斜率为0
    // if (y < -step_width || y > step_width){
    //     dzdx = 0.0;
    // }
    return dzdx;
}


// 方形台阶地形
double
Stair::GetHeight (double x, double y) const
{   
    double h = 0.0;
    // 获得高度
    for (int i=0; i<stair_num; i++) {
        // y轴正半轴
        for (int j=0; j<stair_num; j++) {
            // very steep ramp leading up to block
            // // x轴方向梯度面
            // if (stair_start+i*stair_width <= x && x <=stair_start+i*stair_width+eps_){
            //     if (i==0){
            //         h = (i+j+1)*stair_height/eps_ * (x-stair_start);
            //     }
            //     else{
            //        h = (i+j)*stair_height + slope_*(x-stair_start-i*stair_width); 
            //     }
            //     break;
            // }
            // // y轴方向梯度面
            // if ((j != 0) && (j*stair_width <= y && y <=j*stair_width+eps_)){
            //     h = (i+j)*stair_height + slope_*(y-j*stair_width);
            //     break;
            // }
            // 台阶面
            if ((stair_start+i*stair_width+eps_ <= x && x <= stair_start+(i+1)*stair_width) &&
               (j*stair_width+eps_ <= y && y <= (j+1)*stair_width)){
                h = (i+j+1)*stair_height;
                break;
            }
        }
        // y轴负半轴
        for (int j=0; j<stair_num; j++) {
            // // very steep ramp leading up to block
            // // x轴方向梯度面
            // if (stair_start+i*stair_width <= x && x <=stair_start+i*stair_width+eps_){
            //     if (i==0){
            //         h = (i-j+1)*stair_height/eps_ * (x-stair_start);
            //     }
            //     else{
            //        h = (i-j)*stair_height + slope_*(x-stair_start-i*stair_width); 
            //     }
            //     break;
            // }
            // // y轴方向梯度面
            // if ((j != 0) && (j*stair_width-eps_ <= y && y <=j*stair_width)){
            //     h = (i-j)*stair_height + slope_*(y-j*stair_width);
            //     break;
            // }
            // 台阶面
            if ((stair_start+i*stair_width+eps_ <= x && x <= stair_start+(i+1)*stair_width) &&
               (j*stair_width+eps_ <= -y && -y <= (j+1)*stair_width)){
                h = (i+j+1)*stair_height;
                break;
            }
        }
    }
    return h;
}

double
Stair::GetHeightDerivWrtX (double x, double y) const
{
    double dzdx = 0.0;
    // for (int i=0; i<stair_num; i++) {
    //     for (int j=0; j<stair_num; j++) {
    //         // very steep ramp leading up to block
    //         if ((stair_start+i*stair_width <= x && x <=stair_start+i*stair_width+eps_) &&
    //             (j*stair_width+eps_ <= y && y <= (j+1)*stair_width)){
    //             if (i==0){
    //                 dzdx = (i+j+1)*stair_height/eps_;
    //             }
    //             else{
    //                 dzdx = slope_; 
    //             }
    //             break;
    //         }
    //     }
    // }
    // // 如果超过左右两侧宽度，则斜率为0
    // if (y < -stair_num*stair_width || y > stair_num*stair_width){
    //     dzdx = 0.0;
    // }
    return dzdx;
}

double
Stair::GetHeightDerivWrtY (double x, double y) const
{
    double dzdy = 0.0;
    // // y轴正半轴
    // for (int j=0; j<stair_num; j++) {
    //     // very steep ramp leading up to block
    //     if (j == 0) break;
    //     if ((j != 0) && (j*stair_width <= y && y <=j*stair_width+eps_)){
    //         dzdy = slope_;
    //         break;
    //     }
    // }
    // // // y轴负半轴
    // // for (int j=0; j<stair_num; j++) {
    // //     // very steep ramp leading up to block
    // //     if (j == 0) break;
    // //     if ((j != 0) && (j*stair_width-eps_ <= y && y <=j*stair_width)){
    // //         dzdy = slope_;
    // //         break;
    // //     }
    // // }
    // // 如果超过左右两侧宽度，则斜率为0
    // if (x < stair_start || x > stair_num*stair_width){
    //     dzdy = 0.0;
    // }
    return dzdy;
}


// 裂隙地形
double
Fissure::GetHeight (double x, double y) const
{
    double h = 0.0;

    // 裂隙地区较低
    if (fissure_start <= x && x <= fissure_start + fissure_width)
        // h = -fissure_height;
        h = a*x*x + b*x + c;

    return h;
}

double
Fissure::GetHeightDerivWrtX (double x, double y) const
{
    double dzdx = 0.0;

    if (fissure_start <= x && x <= fissure_start + fissure_width)
        dzdx = 2*a*x + b;

    return dzdx;
}

double
Fissure::GetHeightDerivWrtXX (double x, double y) const
{
    double dzdxx = 0.0;

    if (fissure_start <= x && x <= fissure_start + fissure_width)
        dzdxx = 2*a;

    return dzdxx;
}


// 下楼楼梯地形
double
DownStep::GetHeight (double x, double y) const
{
    double h = 0.0;
    // 获得高度
    for (int i=0; i<down_num; i++) {
        // very steep ramp leading up to block
        // if (step_start+i*step_width <= x && x <=step_start+i*step_width+eps_){
        //     h = i*step_height + slope_*(x-step_start-i*step_width);
        //     break;
        // }
        if (down_start+i*down_width+eps_ <= x && x <= down_start+(i+1)*down_width){
            h = -(i+1)*down_height;
            break;
        }
    }
    // 如果超过左右两侧宽度，则高度为0
    if (y < -down_length || y > down_length){
        h = 0.0;
    }
    return h;
}

double
DownStep::GetHeightDerivWrtX (double x, double y) const
{
    double dzdx = 0.0;
    // for (int i=0; i<step_num; i++) {
    //     // very steep ramp leading up to block
    //     if (step_start+i*step_width <= x && x <=step_start+i*step_width+eps_){
    //         dzdx = slope_;
    //         break;
    //     }
    // }
    // // 如果超过左右两侧宽度，则斜率为0
    // if (y < -step_width || y > step_width){
    //     dzdx = 0.0;
    // }
    return dzdx;
}


// 桥梁地形
double
Bridge::GetHeight (double x, double y) const
{
    double h = 0.0;

    // 非桥梁地区较低
    if ((bridge_start <= x && x <= bridge_start + bridge_gap_width) &&
        (bridge_width/2 <= y || y <= -bridge_width/2))
        // h = -bridge_height;
        h = a*x*x + b*x + c;

    return h;
}

double
Bridge::GetHeightDerivWrtX (double x, double y) const
{
    double dzdx = 0.0;

    if ((bridge_start <= x && x <= bridge_start + bridge_gap_width) &&
        (bridge_width/2 <= y || y <= -bridge_width/2))
        dzdx = 2*a*x + b;

    return dzdx;
}

double
Bridge::GetHeightDerivWrtXX (double x, double y) const
{
    double dzdxx = 0.0;

    if ((bridge_start <= x && x <= bridge_start + bridge_gap_width) &&
        (bridge_width/2 <= y || y <= -bridge_width/2))
        dzdxx = 2*a;

    return dzdxx;
}

double
Bridge::GetHeightDerivWrtY (double x, double y) const
{
    double dzdy = 0.0;

    if ((bridge_start <= x && x <= bridge_start + bridge_gap_width) &&
        ((bridge_width/2 <= y && y <= bridge_width/2 + eps_) || (-bridge_width/2 - eps_ <= y && y <= -bridge_width/2)))
        dzdy = slope_y_;

    return dzdy;
}


// 梅花桩地形
double
Pile::GetHeight (double x, double y) const
{
    double h = 0.0;

    // // 裂隙
    // if (pile_start <= x && x <= pile_start + pile_gap_width)
    //     h = -pile_height;
    // // 梅花桩
    // for (int i=0; i<pile_num; i++) {
    //     // 首先两列，然后判断每行的梅花桩
    //     if (((-pile_clearance / 2 - pile_width / 2 <= y && y <= -pile_clearance / 2 + pile_width / 2) ||
    //         (pile_clearance / 2 - pile_width / 2 <= y && y <= pile_clearance / 2 + pile_width / 2)) &&
    //         (pile_start + pile_clearance_x * (i + 1) - pile_width / 2 <= x && x <= pile_start + pile_clearance_x * (i + 1) + pile_width / 2)) {
    //         h = 0.0;
    //         // h = a*(x-pile_clearance_x*i)*(x-pile_clearance_x*i) + b*(x-pile_clearance_x*i) + c;
    //         break;
    //     }
    // }
    // 梅花桩间的间隙
    for (int i=0; i<pile_num+1; i++) {
        // 首先两列，然后判断每行梅花桩间隙
        if (((-pile_clearance / 2 - pile_width / 2 <= y && y <= -pile_clearance / 2 + pile_width / 2) ||
             (pile_clearance / 2 - pile_width / 2 <= y && y <= pile_clearance / 2 + pile_width / 2)) &&
            (pile_start + pile_clearance_x*i + pile_width*i <= x && x <= pile_start + pile_clearance_x*(i+1) + pile_width*i)) {
            // h = 0.0;
            h = a*(x-(pile_start + pile_clearance_x*i + pile_width*i))*(x-(pile_start + pile_clearance_x*i + pile_width*i)) + 
                b*(x-(pile_start + pile_clearance_x*i + pile_width*i)) + c;
            break;
        }
    }
    // 其他空白区域
    if ((pile_start <= x && x <= pile_start + pile_gap_width) && 
        ((-pile_clearance / 2 + pile_width / 2 <= y && y <= pile_clearance / 2 - pile_width / 2) || 
         (y <= -pile_clearance / 2 - pile_width / 2)|| (y >= pile_clearance / 2 + pile_width / 2)))
        h = -pile_height;

    return h;
}

double
Pile::GetHeightDerivWrtX (double x, double y) const
{
    double dzdx = 0.0;

    for (int i=0; i<pile_num+1; i++) {
        if (((-pile_clearance / 2 - pile_width / 2 <= y && y <= -pile_clearance / 2 + pile_width / 2) ||
            (pile_clearance / 2 - pile_width / 2 <= y && y <= pile_clearance / 2 + pile_width / 2)) &&
            (pile_start + pile_clearance_x*i + pile_width*i <= x && x <= pile_start + pile_clearance_x*(i+1) + pile_width*i)){
            dzdx = 2*a*(x-(pile_start + pile_clearance_x*i + pile_width*i)) + b;
            break;
        }
    }

    return dzdx;
}

double
Pile::GetHeightDerivWrtXX (double x, double y) const
{
    double dzdxx = 0.0;

    for (int i=0; i<pile_num+1; i++) {
        if (((-pile_clearance / 2 - pile_width / 2 <= y && y <= -pile_clearance / 2 + pile_width / 2) ||
            (pile_clearance / 2 - pile_width / 2 <= y && y <= pile_clearance / 2 + pile_width / 2)) &&
            (pile_start + pile_clearance_x*i + pile_width*i <= x && x <= pile_start + pile_clearance_x*(i+1) + pile_width*i)){
            dzdxx = 2*a;
            break;
        }
    }

    return dzdxx;
}

double
Pile::GetHeightDerivWrtY (double x, double y) const
{
    double dzdy = 0.0;

    for (int i=0; i<pile_num; i++) {
        if (((-pile_clearance/2 - pile_width/2 - eps_<= y && y <= -pile_clearance/2 - pile_width/2) ||
             (-pile_clearance/2 + pile_width/2 <= y && y <= -pile_clearance/2 + pile_width/2 + eps_) ||
             (pile_clearance/2 - pile_width/2 - eps_<= y && y <= pile_clearance/2 - pile_width/2) ||
             (pile_clearance/2 + pile_width/2 <= y && y <= pile_clearance/2 + pile_width/2 + eps_)) &&
            (pile_start + pile_clearance_x*(i+1) + pile_width*i <= x && x <= pile_start + pile_clearance_x*(i+1) + pile_width*(i+1))){
            dzdy = slope_y_;
            break;
        }
    }

    return dzdy;
}

} /* namespace towr */
