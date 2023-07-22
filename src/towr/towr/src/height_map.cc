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

#include <towr/terrain/height_map.h>
#include <towr/terrain/examples/height_map_examples.h>

#include <cmath>
#include <fstream>

namespace towr {

/**
 * 根据TerrainID，创建对应类型的地形对象
 */
// HeightMap::MakeTerrain (TerrainID type)
// {
//   switch (type) {
//     case FlatID:      return std::make_shared<FlatGround>(); break;
//     case BlockID:     return std::make_shared<Block>(); break;
//     case StairsID:    return std::make_shared<Stairs>(); break;
//     case GapID:       return std::make_shared<Gap>(); break;
//     case SlopeID:     return std::make_shared<Slope>(); break;
//     case ChimneyID:   return std::make_shared<Chimney>(); break;
//     case ChimneyLRID: return std::make_shared<ChimneyLR>(); break;
//     default: assert(false); break;
//   }
// }

int TerrainType = 0;


// 从 terrain_set.csv 中读取 terrain_param 信息
void ReadTerrainParamFromCSV(double terrain_param[11]) {
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
            if (i >= 1) {
                terrain_param[i-1] = std::stod(value);
                if (i >= 12) {
                    break;
                }
            }
            i++;
        }
    }
    f_terrain.close();
}


HeightMap::Ptr
HeightMap::MakeTerrain (TerrainID type)
{   
    double terrain_param[11];
    ReadTerrainParamFromCSV(terrain_param);

    step_start_set = terrain_param[0];
    step_height_set = terrain_param[1];
    step_width_set = terrain_param[2];
    stair_start_set = terrain_param[3];
    stair_height_set = terrain_param[4];
    stair_width_set = terrain_param[5];
    fissure_start_set = terrain_param[6];
    fissure_width_set = terrain_param[7];
    down_start_set = terrain_param[8];
    down_height_set = terrain_param[9];
    down_width_set = terrain_param[10];

    // // 地形参数随机采样
    // SampleTerrainParameters(type);
    // printf("@@@@@@@@ Terrain params: %lf, %lf, %lf, %lf, %lf, %lf, %lf\n", 
    //        step_start_set, step_height_set, step_width_set, stair_start_set, stair_height_set, stair_width_set, fissure_start_set, fissure_width_set);

    // 根据地形类型和地形参数生成地形
    switch (type) {
        case FlatID:      TerrainType = 0; return std::make_shared<FlatGround>(); break;
        case StepID:      TerrainType = 1; return std::make_shared<Step>(); break;
        case StairID:     TerrainType = 2; return std::make_shared<Stair>(); break;
        case FissureID:   TerrainType = 3; return std::make_shared<Fissure>(); break;
        case DownStepID:  TerrainType = 4; return std::make_shared<DownStep>(); break;
        case BridgeID:    TerrainType = 5; return std::make_shared<Bridge>(); break;
        case PileID:      TerrainType = 6; return std::make_shared<Pile>(); break;
        default: assert(false); break;
    }
}

double
HeightMap::GetDerivativeOfHeightWrt (Dim2D dim, double x, double y) const
{
  switch (dim) {
    case X: return GetHeightDerivWrtX(x,y);
    case Y: return GetHeightDerivWrtY(x,y);
    default: assert(false); // derivative dimension not implemented
  }
}

HeightMap::Vector3d
HeightMap::GetNormalizedBasis (Direction basis, double x, double y) const
{
  return GetBasis(basis, x, y).normalized();
}

HeightMap::Vector3d
HeightMap::GetBasis (Direction basis, double x, double y,
                                  const DimDerivs& deriv) const
{
  switch (basis) {
    case Normal:   return GetNormal(x,y, deriv);
    case Tangent1: return GetTangent1(x,y, deriv);
    case Tangent2: return GetTangent2(x,y, deriv);
    default: assert(false); // basis does not exist
  }
}

HeightMap::Vector3d
HeightMap::GetDerivativeOfNormalizedBasisWrt (Direction basis, Dim2D dim,
                                              double x, double y) const
{
  // inner derivative
  Vector3d dv_wrt_dim = GetBasis(basis, x, y, {dim});

  // outer derivative
  Vector3d v = GetBasis(basis, x,y, {});
  Vector3d dn_norm_wrt_n = GetDerivativeOfNormalizedVectorWrtNonNormalizedIndex(v, dim);
  return dn_norm_wrt_n.cwiseProduct(dv_wrt_dim);
}

HeightMap::Vector3d
HeightMap::GetNormal(double x, double y, const DimDerivs& deriv) const
{
  Vector3d n;

  bool basis_requested = deriv.empty();

  for (auto dim : {X_,Y_}) {
    if (basis_requested)
      n(dim) = -GetDerivativeOfHeightWrt(dim, x, y);
    else
      n(dim) = -GetSecondDerivativeOfHeightWrt(dim, deriv.front(), x, y);
  }

  n(Z) = basis_requested? 1.0 : 0.0;

  return n;
}

HeightMap::Vector3d
HeightMap::GetTangent1 (double x, double y, const DimDerivs& deriv) const
{
  Vector3d tx;

  bool basis_requested = deriv.empty();

  tx(X) = basis_requested? 1.0 : 0.0;
  tx(Y) = 0.0;
  tx(Z) = basis_requested? GetDerivativeOfHeightWrt(X_, x, y)
                         : GetSecondDerivativeOfHeightWrt(X_, deriv.front(), x, y);

  return tx;
}

HeightMap::Vector3d
HeightMap::GetTangent2 (double x, double y, const DimDerivs& deriv) const
{
  Vector3d ty;

  bool basis_requested = deriv.empty();

  ty(X) = 0.0;
  ty(Y) = basis_requested? 1.0 : 0.0;
  ty(Z) = basis_requested? GetDerivativeOfHeightWrt(Y_, x,y)
                         : GetSecondDerivativeOfHeightWrt(Y_, deriv.front(), x, y);
  return ty;
}

HeightMap::Vector3d
HeightMap::GetDerivativeOfNormalizedVectorWrtNonNormalizedIndex (
    const Vector3d& v, int idx) const
{
  // see notebook or
  // http://blog.mmacklin.com/2012/05/
  return 1/v.squaredNorm()*(v.norm() * Vector3d::Unit(idx) - v(idx)*v.normalized());
}

double
HeightMap::GetSecondDerivativeOfHeightWrt (Dim2D dim1, Dim2D dim2,
                                           double x, double y) const
{
  if (dim1 == X_) {
    if (dim2 == X_) return GetHeightDerivWrtXX(x,y);
    if (dim2 == Y_) return GetHeightDerivWrtXY(x,y);
  } else {
    if (dim2 == X_) return GetHeightDerivWrtYX(x,y);
    if (dim2 == Y_) return GetHeightDerivWrtYY(x,y);
  }

  assert(false); // second derivative not specified.
}

} /* namespace towr */
