//
// Created by Nobuyuki Umetani on 2021-10-01.
//

#ifndef SAVE_SCALED_HEAD_HAIR_H_
#define SAVE_SCALED_HEAD_HAIR_H_

#include <Alembic/AbcGeom/All.h>
#include <Alembic/AbcCoreOgawa/All.h>

#include "headmesh.h"
#include "hair.h"
#include "delfem2/mat4.h"
#include "delfem2/vec3.h"
#include "delfem2/points.h"

template <typename T>
void SaveHair(
    const HeadMesh& head,
    const Hair& hair,
    T trans_x,
    T trans_y,
    T trans_z,
    T scale,
    const std::string& file_path) {

    std::vector<float> vtx_xyz1_original;
    std::vector<float> vtx_xyz1;
    std::vector<int> hair_nvtx;
    {
        vtx_xyz1.insert(
            vtx_xyz1.end(),
            hair.vtx_xyz.begin(), hair.vtx_xyz.end());
        vtx_xyz1_original.insert(
            vtx_xyz1_original.end(),
            hair.vtx_xyz.begin(), hair.vtx_xyz.end());
        hair_nvtx.push_back(static_cast<int>(hair.vtx_xyz.size() / 3));
    }
    {  // translate & scale points
        delfem2::Translate_Points3(
            vtx_xyz1,
            trans_x, trans_y, trans_z);
        delfem2::Scale_Points(
            vtx_xyz1.data(),
            vtx_xyz1.size() / 3, 3,
            scale);
    }

    const float dt = 1.0f / 60.0f;
    Alembic::Abc::OArchive archive(
        Alembic::AbcCoreOgawa::WriteArchive(),
        file_path);
    {
        Alembic::AbcGeom::OCurves curves_obj(
            Alembic::Abc::OObject(archive, Alembic::Abc::kTop),
            "curve");
        { // set time sampling
            const Alembic::Abc::TimeSampling time_sampling(dt, 0);
            const uint32_t time_sampling_index = archive.addTimeSampling(time_sampling);
            curves_obj.getSchema().setTimeSampling(time_sampling_index);
        }
        Alembic::AbcGeom::P3fArraySample coords(
            (const Alembic::Abc::V3f*)vtx_xyz1.data(),
            vtx_xyz1.size() / 3);
        const Alembic::AbcGeom::OCurvesSchema::Sample mesh_samp(
            coords,
            Alembic::AbcGeom::Int32ArraySample(hair_nvtx.data(), hair_nvtx.size()),
            Alembic::AbcGeom::kCubic,
            Alembic::AbcGeom::kNonPeriodic);
        // std::cout << mesh_samp.getNumCurves() << std::endl;
        Alembic::Abc::OInt16Property prop0(
            curves_obj.getSchema().getArbGeomParams(),
            "groom_version_major",
            1,
            Alembic::AbcGeom::kConstantScope);
        Alembic::Abc::OInt16Property prop1(
            curves_obj.getSchema().getArbGeomParams(),
            "groom_version_minor",
            5,
            Alembic::AbcGeom::kConstantScope);
        curves_obj.getSchema().set(mesh_samp);
    }
}

void SaveScaledHeadHair(
    const std::filesystem::path& path_head,
    const std::filesystem::path& path_hair_dir,
    const HeadMesh& head,
    const std::vector<Hair> &hairs,
    const delfem2::CVec3f &region_center)
{
  const double trans[3] = {
      static_cast<double>(-region_center.x),
      static_cast<double>(-region_center.y),
      static_cast<double>(-region_center.z) };
  double scale = 1;
  {
    double pmin[3], pmax[3];
    delfem2::BoundingBox3_Points3(
        pmin, pmax,
        head.vtx_xyz.data(), head.vtx_xyz.size()/3);
    const double size_y = pmax[1] - pmin[1];
    scale = 30 / size_y;
  }
  {
    head.SaveObj(path_head, trans, scale);
  }
  {
    for (int nh = 0; nh < hairs.size(); nh++) {
      std::stringstream stream;
      stream << std::setw(4) << std::setfill('0') << nh;
      std::string filename = "hair" + stream.str() + ".abc";
      std::filesystem::create_directory(path_hair_dir);
      std::filesystem::path path_hair_file = path_hair_dir / filename;
      SaveHair(
          head, hairs[nh],
          (float)trans[0], (float)trans[1], (float)trans[2],
          (float)scale, path_hair_file.string());
    }
  }
}

#endif // SAVE_SCALED_HEAD_HAIR_H_
