//
// Created by Nobuyuki Umetani on 2021-10-05.
//

#ifndef HAIR_ELASTIC_H_
#define HAIR_ELASTIC_H_

#include "delfem2/fem_rod3_straight.h"
#include "delfem2/fem_distance3.h"
#include "delfem2/fem_rod2.h"
#include "delfem2/geo_polyline.h"
#include "delfem2/geo_curve_quadratic.h"
#include "delfem2/isrf_adf.h"

// the only dependencies of this file (camera and hair)
#include "camera_projection.h"
#include "hair.h"

void HairElasticLength_Newton(
    delfem2::BlockPentaDiagonalMatrix<3> &mat,
    std::vector<double> &res,
    float length_edge_ini,
    const std::vector<float> &vtx_xyz) {
  namespace dfm2 = delfem2;
  const unsigned int nvtx = vtx_xyz.size() / 3;
  if (nvtx < 2) { return; }
  for (unsigned int ivtx = 0; ivtx < nvtx - 1; ++ivtx) {
    unsigned int jvtx = ivtx + 1;
    const dfm2::CVec3d end_points_xyz[2] = {
        dfm2::CVec3d{vtx_xyz[ivtx * 3 + 0], vtx_xyz[ivtx * 3 + 1], vtx_xyz[ivtx * 3 + 2]},
        dfm2::CVec3d{vtx_xyz[jvtx * 3 + 0], vtx_xyz[jvtx * 3 + 1], vtx_xyz[jvtx * 3 + 2]}};
    double stiffness = 1.0;
    dfm2::CVec3d dw_dp[2];
    dfm2::CMat3d ddw_ddp[2][2];
    WdWddW_SquareLengthLineseg3D(
        dw_dp, ddw_ddp,
        stiffness,
        end_points_xyz,
        length_edge_ini);
    mat.MergeBlock(ivtx, ivtx, ddw_ddp[0][0].data());
    mat.MergeBlock(ivtx, jvtx, ddw_ddp[0][1].data());
    mat.MergeBlock(jvtx, ivtx, ddw_ddp[1][0].data());
    mat.MergeBlock(jvtx, jvtx, ddw_ddp[1][1].data());
    for (int idim = 0; idim < 3; ++idim) {
      res[ivtx * 3 + idim] += dw_dp[0][idim];
      res[jvtx * 3 + idim] += dw_dp[1][idim];
    }
  }
}

// optimize for small bend
void HairElasticBendStraight(
    delfem2::BlockPentaDiagonalMatrix<3> &mat,
    std::vector<double> &res,
    double stiff_bend,
    const std::vector<float> &vtx_xyz) {
  namespace dfm2 = delfem2;
  const unsigned int nvtx = vtx_xyz.size() / 3;
  if (nvtx < 3) { return; }
  for (unsigned int ivtx = 0; ivtx < nvtx - 2; ++ivtx) {
    unsigned int jvtx = ivtx + 1;
    unsigned int kvtx = ivtx + 2;
    const float end_points_xyz[3][3] = {
        {vtx_xyz[ivtx * 3 + 0], vtx_xyz[ivtx * 3 + 1], vtx_xyz[ivtx * 3 + 2]},
        {vtx_xyz[jvtx * 3 + 0], vtx_xyz[jvtx * 3 + 1], vtx_xyz[jvtx * 3 + 2]},
        {vtx_xyz[kvtx * 3 + 0], vtx_xyz[kvtx * 3 + 1], vtx_xyz[kvtx * 3 + 2]}};
    float c[3], dc[3][3][3];
    dfm2::CdC_Rod3BendStraight<float>(
        c, dc,
        end_points_xyz);
    for (unsigned int ib = 0; ib < 3; ++ib) {
      for (unsigned int jb = 0; jb < 3; ++jb) {
        double *p = mat.GetValuePointer(ivtx + ib, ivtx + jb);
        for (int kdim = 0; kdim < 3; ++kdim) {
          for (int idim = 0; idim < 3; ++idim) {
            for (int jdim = 0; jdim < 3; ++jdim) {
              p[idim * 3 + jdim] += stiff_bend * dc[kdim][ib][idim] * dc[kdim][jb][jdim];
            }
          }
        }
      }
    }
    for (unsigned int ib = 0; ib < 3; ++ib) {
      double *p = res.data() + (ivtx + ib) * 3;
      for (int kdim = 0; kdim < 3; ++kdim) {
        for (int idim = 0; idim < 3; ++idim) {
          p[idim] += stiff_bend * c[kdim] * dc[kdim][ib][idim];
        }
      }
    }
  }
}

void HairElasticBendStraightRoot(
    delfem2::BlockPentaDiagonalMatrix<3> &mat,
    std::vector<double> &res,
    const delfem2::CVec3d &trg,
    const delfem2::CVec3d &nrm,
    double stiff_bend,
    float length_edge_ini,
    const std::vector<float> &vtx_xyz_hair) {
  namespace dfm2 = delfem2;
  dfm2::CVec3f p0 = (trg - length_edge_ini * nrm).cast<float>();
  const float end_points_xyz[3][3] = {
      {p0.x, p0.y, p0.z},
      {vtx_xyz_hair[0], vtx_xyz_hair[1], vtx_xyz_hair[2]},
      {vtx_xyz_hair[3], vtx_xyz_hair[4], vtx_xyz_hair[5]}};
  float c[3], dc[3][3][3];
  dfm2::CdC_Rod3BendStraight<float>(
      c, dc,
      end_points_xyz);
  for (unsigned int ib = 0; ib < 2; ++ib) {
    for (unsigned int jb = 0; jb < 2; ++jb) {
      double *p = mat.GetValuePointer(ib, jb);
      for (int kdim = 0; kdim < 3; ++kdim) {
        for (int idim = 0; idim < 3; ++idim) {
          for (int jdim = 0; jdim < 3; ++jdim) {
            p[idim * 3 + jdim] += stiff_bend * dc[kdim][ib + 1][idim] * dc[kdim][jb + 1][jdim];
          }
        }
      }
    }
  }
  for (unsigned int ib = 0; ib < 2; ++ib) {
    for (int kdim = 0; kdim < 3; ++kdim) {
      for (int idim = 0; idim < 3; ++idim) {
        res[ib * 3 + idim] += stiff_bend * c[kdim] * dc[kdim][ib + 1][idim];
      }
    }
  }
}

void FitToSketchPolyline(
  delfem2::BlockPentaDiagonalMatrix<3> &mat,
  std::vector<double> &res,
  unsigned int ivtx,
  std::vector<float> &vtx_xyz,
  const delfem2::CMat4f &mat_modelviewprojection,
  const std::vector<delfem2::CVec2f> &polyline) {
  namespace dfm2 = delfem2;
  using V2 = dfm2::CVec2f;
  const V2 scr = dfm2::Vec2_Mat4Vec3_Homography(
    mat_modelviewprojection.data(), vtx_xyz.data() + ivtx * 3);
  const double nearest_param = dfm2::Nearest_Polyline<V2>(polyline, scr);
  if( nearest_param < 0 ){ return; }
  if( nearest_param >= polyline.size()-1){ return; }
  const V2 trg = dfm2::Sample_Polyline(polyline, nearest_param);
  V2 un;
  if( (trg - scr).norm() < 1.0e-5 ){
    V2 tangent = dfm2::Tangent_Polyline(polyline, nearest_param);
    tangent = tangent.normalized();
    un = {tangent.y, -tangent.x};
  }
  else{
    un = (trg-scr).normalized();
  }
  // compute the sensitivity how the current screen position moves when 3D hair vertex move
  float c[2], dc[2][3];
  dfm2::CdC_ScreenCoordinate<float>(
    c, dc,
    mat_modelviewprojection.data(),
    vtx_xyz.data() + ivtx * 3);
  c[0] -= trg.x;
  c[1] -= trg.y;
  float cn = c[0] * un.x + c[1] * un.y;
  double w = 100;
  {
    double *p = mat.GetValuePointer(static_cast<int>(ivtx), static_cast<int>(ivtx));
    for (int idim = 0; idim < 3; ++idim) {
      for (int jdim = 0; jdim < 3; ++jdim) {
        double a =
            + un.x * dc[0][idim] * dc[0][jdim] * un.x
            + un.x * dc[0][idim] * dc[1][jdim] * un.y
            + un.y * dc[1][idim] * dc[0][jdim] * un.x
            + un.y * dc[1][idim] * dc[1][jdim] * un.y;
        p[idim * 3 + jdim] += w * a;
      }
    }
  }
  for (int idim = 0; idim < 3; ++idim) {
    double a = cn * (dc[0][idim] * un.x + dc[1][idim] * un.y);
    res[ivtx * 3 + idim] += w * a;
  }
}


void FitToImplicit(
  delfem2::BlockPentaDiagonalMatrix<3> &mat,
  std::vector<double> &res,
  unsigned int ivtx,
  const std::vector<float> &vtx_xyz,
  const delfem2::CMat4f &mat_modelviewprojection,
  const ImplicitRbfApproximation &rbf) {
  namespace dfm2 = delfem2;
  using V2 = dfm2::CVec2f;
  const V2 scr = dfm2::Vec2_Mat4Vec3_Homography(
    mat_modelviewprojection.data(), vtx_xyz.data() + ivtx * 3);
  const auto[t, dtdx, dtdy] = rbf.Evaluate2Grad(
    scr.x, scr.y,
    [](double r) { return -std::exp(-r); }  // lambda for rbf gradient
    );
  // compute the sensitivity how the current screen position moves when 3D hair vertex move
  float c[2], dc[2][3];
  dfm2::CdC_ScreenCoordinate<float>(
    c, dc,
    mat_modelviewprojection.data(),
    vtx_xyz.data() + ivtx * 3);
  double w = 1;
  {
    double *p = mat.GetValuePointer(static_cast<int>(ivtx), static_cast<int>(ivtx));
    for (int idim = 0; idim < 3; ++idim) {
      for (int jdim = 0; jdim < 3; ++jdim) {
        double a = (dtdx * dc[0][idim] + dtdy * dc[1][idim]) * (dtdx * dc[0][jdim] + dtdy * dc[1][jdim]);
        p[idim * 3 + jdim] += w * a;
      }
    }
  }
  for (int idim = 0; idim < 3; ++idim) {
    res[ivtx * 3 + idim] += w * t * (dtdx * dc[0][idim] + dtdy * dc[1][idim]);
  }
}

void FitToSketch_BSpline(
    delfem2::BlockPentaDiagonalMatrix<3> &mat,
    std::vector<double> &res,
    unsigned int ivtx,
    std::vector<float> &vtx_xyz,
    const delfem2::CMat4f &mat_modelviewprojection,
    const std::vector<delfem2::CVec2f> &cps) { //
  namespace dfm2 = delfem2;
  if (cps.size() <= 2) { return; }
  using V2 = dfm2::CVec2f;
  const V2 scr = dfm2::Vec2_Mat4Vec3_Homography(
      mat_modelviewprojection.data(), vtx_xyz.data() + ivtx * 3);
  auto t_spline = dfm2::Nearest_QuadraticBSplineCurve<V2>(cps, scr);
  if (t_spline < 1.0e-10 || t_spline >= static_cast<double>(cps.size()) - 2 - 1.0e-10) { return; }
  const V2 trg0 = dfm2::Sample_QuadraticBsplineCurve(t_spline, cps);
  V2 tan0 = dfm2::Tangent_QuadraticBsplineCurve(t_spline, cps);
  V2 unit_normal0 = dfm2::rotate90(tan0).normalized();
  float c[2], dc[2][3];
  dfm2::CdC_ScreenCoordinate<float>(
      c, dc,
      mat_modelviewprojection.data(),
      vtx_xyz.data() + ivtx * 3);
  c[0] -= trg0.x;
  c[1] -= trg0.y;
  float cn = c[0] * unit_normal0.x + c[1] * unit_normal0.y;
  {
    double *p = mat.GetValuePointer(static_cast<int>(ivtx), static_cast<int>(ivtx));
    for (int idim = 0; idim < 3; ++idim) {
      for (int jdim = 0; jdim < 3; ++jdim) {
        p[idim * 3 + jdim] +=
            (unit_normal0.x * dc[0][idim] + unit_normal0.y * dc[1][idim]) *
            (unit_normal0.x * dc[0][jdim] + unit_normal0.y * dc[1][jdim]);
      }
    }
  }
  for (int idim = 0; idim < 3; ++idim) {
    res[ivtx * 3 + idim] += cn * (unit_normal0.x * dc[0][idim] + unit_normal0.y * dc[1][idim]);
  }
  /*
  float cn = c[0] * un.x + c[1] * un.y;
  for (int idim = 0; idim < 3; ++idim) {
    for (int jdim = 0; jdim < 3; ++jdim) {
      matrix_lm(ivtx * 3 + idim, ivtx * 3 + jdim) +=
        +un.x * dc[0][idim] * dc[0][jdim] * un.x
          + un.x * dc[0][idim] * dc[1][jdim] * un.y
          + un.y * dc[1][idim] * dc[0][jdim] * un.x
          + un.y * dc[1][idim] * dc[1][jdim] * un.y;
    }
    resvec_lm(ivtx * 3 + idim) += cn * (dc[0][idim] * un.x + dc[1][idim] * un.y);
  }
   */
}

struct SimulationParam {
  float length_edge_ini;
  float stiff_bend = 0.001;
  float stiff_contact = 0.1;
  float offset_contact = 0.01;
  float gravity_times_rho[3] = {0, 0.01, 0};
};

double DeformHair(
    Hair &hair,
    const SimulationParam& param,
    const delfem2::AdaptiveDistanceField3 &adf,
    const CameraConfig &cam_config) {
  namespace dfm2 = delfem2;

  const unsigned int nvtx = hair.vtx_xyz.size() / 3;
  if (nvtx < 3) { return 0.; }

  hair.linsys.Initialize(nvtx);
  hair.linsys.dof_bcflag[0] = 1;
  hair.linsys.dof_bcflag[1] = 1;
  hair.linsys.dof_bcflag[2] = 1;
  hair.linsys.BeginMerge();

  // gravity
  if (nvtx >= 2) {
    for (unsigned int ivtx = 0; ivtx < nvtx - 1; ++ivtx) {  // loop for 3d vtx
      unsigned int jvtx = ivtx + 1;
      float elen = param.length_edge_ini;
      hair.linsys.vec_r[ivtx * 3 + 0] += 0.5f * elen * param.gravity_times_rho[0];
      hair.linsys.vec_r[ivtx * 3 + 1] += 0.5f * elen * param.gravity_times_rho[1];
      hair.linsys.vec_r[ivtx * 3 + 2] += 0.5f * elen * param.gravity_times_rho[2];
      hair.linsys.vec_r[jvtx * 3 + 0] += 0.5f * elen * param.gravity_times_rho[0];
      hair.linsys.vec_r[jvtx * 3 + 1] += 0.5f * elen * param.gravity_times_rho[1];
      hair.linsys.vec_r[jvtx * 3 + 2] += 0.5f * elen * param.gravity_times_rho[2];
    }
  }

  // contact against head (represented with adf)
  for (unsigned int ivtx = 0; ivtx < nvtx; ++ivtx) {
    dfm2::CVec3d pos0(hair.vtx_xyz.data() + ivtx * 3);
    double n[3];
    double sd = adf.Projection(pos0.x, pos0.y, pos0.z, n);
    const dfm2::CVec3d unorm(n[0], n[1], n[2]);
    double penetration = sd + param.offset_contact;
    if (penetration < 0) { continue; }
    hair.linsys.vec_r[ivtx * 3 + 0] += -penetration * unorm.x * param.stiff_contact;
    hair.linsys.vec_r[ivtx * 3 + 1] += -penetration * unorm.y * param.stiff_contact;
    hair.linsys.vec_r[ivtx * 3 + 2] += -penetration * unorm.z * param.stiff_contact;
    double *p = hair.linsys.dia.GetValuePointer(static_cast<int>(ivtx), static_cast<int>(ivtx));
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        p[i * 3 + j] += param.stiff_contact * unorm(i) * unorm(j);
      }
    }
  }

  HairElasticLength_Newton(
      hair.linsys.dia,
      hair.linsys.vec_r,
      param.length_edge_ini,
      hair.vtx_xyz);
  HairElasticBendStraight(
      hair.linsys.dia,
      hair.linsys.vec_r,
      param.stiff_bend,
      hair.vtx_xyz);
  {
    delfem2::CVec3d root_dir = hair.root_rotation.RotateVector(hair.root_normal.data());
    HairElasticBendStraightRoot(
        hair.linsys.dia,
        hair.linsys.vec_r,
        hair.root_pos, root_dir,
        param.stiff_bend, param.length_edge_ini,
        hair.vtx_xyz);
  }

  // above: physics energy
  // ----------------------
  // below: sketch energy

  hair.vtx_res = hair.linsys.vec_r;

  for (const auto &camst: hair.camera_stroke) {
    unsigned int icam = camst.first;
    const auto[mat_modelview, mat_projection] = CameraTransformationMVP(cam_config, icam);
    const dfm2::CMat4f mat_mvp = mat_projection * mat_modelview;
    for (unsigned int ivtx = 1; ivtx < nvtx; ++ivtx) {
      if( !camst.second.rbf.IsInitailized2() ){
        continue;
      }
      /*
      FitToImplicit(
        hair.linsys.dia,
        hair.linsys.vec_r,
        ivtx, hair.vtx_xyz, mat_mvp, camst.second.rbf);
        */
      /*
      FitToSketch_BSpline(
          hair.linsys.dia,
          hair.linsys.vec_r,
          ivtx, hair.vtx_xyz, mat_mvp, camst.second.xys);
          */
      FitToSketchPolyline(
          hair.linsys.dia,
          hair.linsys.vec_r,
          ivtx, hair.vtx_xyz, mat_mvp, camst.second.xys);
    }
  }

  // above: making linear system
  // ------------------------------
  // below: solving linear system

  for (unsigned int ivtx = 0; ivtx < nvtx; ++ivtx) {
    for (unsigned int idim = 0; idim < 3; ++idim) {
      hair.linsys.AddValueToDiagonal(ivtx, idim, 1.0e-3);
    }
  }

  hair.linsys.Solve();
  double diff = 0.0;
  for (unsigned int ivtx = 0; ivtx < nvtx; ++ivtx) {
    for(int idim=0;idim<3;++idim) {
      double x0 = hair.linsys.vec_x[ivtx * 3 + idim];
      diff += x0 * x0;
      hair.vtx_xyz[ivtx * 3 + idim] -= static_cast<float>(x0);
    }
  }
  return std::sqrt(diff)/nvtx;
}

void AdjustNumberOfSegmentsInHair(
    Hair &hair,
    const CameraConfig &cam_config) {
  const unsigned int nvtx_hair = hair.vtx_xyz.size() / 3;
  std::vector<double> hairvtx_params(nvtx_hair, 10000);
  for (const auto &camst: hair.camera_stroke) {
    unsigned int idx_camera = camst.first;
    assert(idx_camera < cam_config.aCamera.size());
    const std::vector<delfem2::CVec2f> &stroke_xy = camst.second.xys;
    if (stroke_xy.size() < 3 ) { continue; }
    if (nvtx_hair < 2) { continue; }
    const auto[mat_modelview, mat_projection] = CameraTransformationMVP(cam_config, idx_camera);
    bool is_touch = false;
    {
      for (unsigned int ivtx = 0; ivtx < nvtx_hair; ++ivtx) {
        const delfem2::CVec2f p0 = delfem2::Vec2_Mat4Vec3_Homography(
            (mat_projection * mat_modelview).data(),
            hair.vtx_xyz.data() + ivtx * 3);
        float param = delfem2::Nearest_QuadraticBSplineCurve(stroke_xy, p0);
        const delfem2::CVec2f p1 = delfem2::Sample_QuadraticBsplineCurve(param, stroke_xy);
        if( (p0-p1).norm() < 0.05 ){ is_touch = true; break; }
      }
    }
    if( !is_touch ){ continue; }
    for (unsigned int ivtx = 0; ivtx < nvtx_hair; ++ivtx) {
      const delfem2::CVec2f p0 = delfem2::Vec2_Mat4Vec3_Homography(
          (mat_projection * mat_modelview).data(),
          hair.vtx_xyz.data() + ivtx * 3);
      float param = delfem2::Nearest_QuadraticBSplineCurve(stroke_xy, p0);
      param -= static_cast<float>(stroke_xy.size()-2);
      hairvtx_params[ivtx] = (param < hairvtx_params[ivtx]) ? param : hairvtx_params[ivtx];
    }
  }
  /*
  for (unsigned int ivtx = 0; ivtx < nvtx_hair; ++ivtx) {
    std::cout << ivtx << " " << hairvtx_params[ivtx] << std::endl;
  }
   */
  if( hairvtx_params[nvtx_hair-1] == 0 && hairvtx_params[nvtx_hair-2] == 0 ){
    hair.vtx_xyz.resize((nvtx_hair-1)*3);
  }
  else if( hairvtx_params[nvtx_hair-1] < -2 ){
    delfem2::CVec3f p0(hair.vtx_xyz.data()+hair.vtx_xyz.size()-6);
    delfem2::CVec3f p1(hair.vtx_xyz.data()+hair.vtx_xyz.size()-3);
    delfem2::CVec3f p2 = p1 + (p1-p0);
    hair.vtx_xyz.push_back(p2.x);
    hair.vtx_xyz.push_back(p2.y);
    hair.vtx_xyz.push_back(p2.z);
  }
}

void PhysicallySmooth_Polyline(
    std::vector<delfem2::CVec2f> &stroke_xy,
    delfem2::LinearSystemSolver_BlockPentaDiagonal<2> &ls_stroke,
    const std::vector<float> &vtx_xyz,
    const delfem2::CMat4f &mat_mvp,
    double edge_length = 0.05,
    double stiff_bend = 0.001,
    double damping = 0.5) {
  namespace dfm2 = delfem2;
  const double stiff_stretch = 1.0;
  const size_t np = stroke_xy.size();
  if (np < 3) {
    return;
  }
  if (ls_stroke.nblk() != np) {
    ls_stroke.Initialize(np);
  }
  ls_stroke.BeginMerge();
  double W = 0.0;
  for (unsigned int ivtx = 0; ivtx < vtx_xyz.size() / 3; ++ivtx) {
    const delfem2::CVec2f scr = delfem2::Vec2_Mat4Vec3_Homography(
        mat_mvp.data(), vtx_xyz.data() + ivtx * 3);
    const float t_spline = delfem2::Nearest_QuadraticBSplineCurve<delfem2::CVec2f>(stroke_xy, scr);
    if (t_spline < 1.0e-10 || t_spline >= static_cast<double>(stroke_xy.size()) - 2 - 1.0e-10) { continue; }
    const delfem2::CVec2f p0 = delfem2::Sample_QuadraticBsplineCurve(t_spline, stroke_xy);
    const delfem2::CVec2f t0 = delfem2::Tangent_QuadraticBsplineCurve(t_spline, stroke_xy);
    const delfem2::CVec2f un0 = delfem2::rotate90(t0).normalized();
    {
      const unsigned int num_segment = stroke_xy.size() - 2;
      const int idx_segment = static_cast<int>(t_spline) + (t_spline == float(num_segment) ? -1 : 0);
      assert(idx_segment >= 0 && idx_segment < int(num_segment));
      const float s = t_spline - float(idx_segment);  // parameter in this segment
      assert(s >= 0 && s <= 1);
      float coeff[3][3];
      delfem2::CoefficientsOfOpenUniformBSpline_Quadratic(coeff, idx_segment, num_segment);
      const double aw[3] = {
          coeff[0][0] + coeff[0][1] * s + coeff[0][2] * s * s,
          coeff[1][0] + coeff[1][1] * s + coeff[1][2] * s * s,
          coeff[2][0] + coeff[2][1] * s + coeff[2][2] * s * s };
      const unsigned int aip[3] = {
          static_cast<unsigned int>(idx_segment),
          static_cast<unsigned int>(idx_segment+1),
          static_cast<unsigned int>(idx_segment+2) };
      double tmp0 = (p0 - scr).dot(un0);
      for(unsigned int i=0;i<3;++i) {
        ls_stroke.vec_r[aip[i] * 2 + 0] -= aw[i] * tmp0 * un0.x;
        ls_stroke.vec_r[aip[i] * 2 + 1] -= aw[i] * tmp0 * un0.y;
      }
      double emat[3][3][2][2];
      for(unsigned int i=0;i<3;++i) {
        for(unsigned int j=0;j<3;++j) {
          emat[i][j][0][0] = aw[i]*aw[j]*un0.x*un0.x;
          emat[i][j][0][1] = aw[i]*aw[j]*un0.x*un0.y;
          emat[i][j][1][0] = aw[i]*aw[j]*un0.y*un0.x;
          emat[i][j][1][1] = aw[i]*aw[j]*un0.y*un0.y;
        }
      }
      ls_stroke.Merge<3,3,2,2>(aip,aip,emat);
    }
  }
  for (unsigned int ihinge = 0; ihinge < np - 2; ++ihinge) {
    const unsigned int aIP[3] = {ihinge, ihinge + 1, ihinge + 2};
    const double ap[3][2] = {
        {stroke_xy[aIP[0]][0], stroke_xy[aIP[0]][1]},
        {stroke_xy[aIP[1]][0], stroke_xy[aIP[1]][1]},
        {stroke_xy[aIP[2]][0], stroke_xy[aIP[2]][1]}};
    const double aL[2] = {edge_length, edge_length};
    double We, dWe[3][2], ddWe[3][3][2][2];
    delfem2::WdWddW_Rod2(
        We, dWe, ddWe,
        ap, aL, stiff_stretch, stiff_stretch, stiff_bend);
    W += We;
    for (int ino = 0; ino < 3; ++ino) {
      ls_stroke.vec_r[aIP[ino] * 2 + 0] -= dWe[ino][0];
      ls_stroke.vec_r[aIP[ino] * 2 + 1] -= dWe[ino][1];
    }
    ls_stroke.template Merge<3, 3, 2, 2>(aIP, aIP, ddWe);
  }
  for (unsigned int ip = 0; ip < np; ++ip) {
    ls_stroke.AddValueToDiagonal(ip, 0, damping);
    ls_stroke.AddValueToDiagonal(ip, 1, damping);
  }
  ls_stroke.Solve();
  // std::cout << W << std::endl;
  for (unsigned int ip = 0; ip < np; ++ip) {
    stroke_xy[ip][0] += static_cast<float>(ls_stroke.vec_x[ip * 2 + 0]);
    stroke_xy[ip][1] += static_cast<float>(ls_stroke.vec_x[ip * 2 + 1]);
  }
}

#endif //HAIR_ELASTIC_H_
