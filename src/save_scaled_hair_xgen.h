//
// Created by Shinichi Kinuwaki on 2021-10-09.
//

#ifndef SAVE_SCALED_XGEN_HAIR_H_
#define SAVE_SCALED_XGEN_HAIR_H_


#include "headmesh.h"
#include "hair.h"
#include "delfem2/mat4.h"
#include "delfem2/vec3.h"
#include "delfem2/points.h"
#include "delfem2/srchuni_v3.h"

void SaveXGenHair(
    const HeadMesh& head,
    const std::vector<float>& vtx_xyz1_original,
    const std::vector<float>& vtx_xyz1, /* translated & scaled */
    const std::vector<double>& vtx_normal,
    const std::vector<int>& hair_nvtx,
    const std::string file_path){

    const auto input_path_dir = std::filesystem::path(PATH_SOURCE_DIR) / ".." / "data" / "template.xgen";
    const auto output_path_dir = std::filesystem::path(file_path);

    // check if valid spline exsists and count it if it exsists
    int guide_spline_count = 0;
    for (unsigned int ns = 0; ns < hair_nvtx.size(); ns++) {
        if (hair_nvtx[ns] >= 3)
            guide_spline_count++;
    }
    if (guide_spline_count == 0)
        return;

    std::ifstream ifs(input_path_dir);
    if (!ifs) {
        return;
    }
    std::string xgen_buff((std::istreambuf_iterator<char>(ifs)), std::istreambuf_iterator<char>());
    ifs.close();

    std::ofstream ofs(output_path_dir);
    if (!ofs) {
        return;
    }

    ofs << xgen_buff << std::endl;
    ofs << "Guides	Spline	" << guide_spline_count << std::endl;

    auto index = 0;
    for (unsigned int ns = 0; ns < hair_nvtx.size(); ns++) {
        const int BUFF_SIZE = 256;
        char buff[BUFF_SIZE];

        if (hair_nvtx[ns] <= 2) {
            index += hair_nvtx[ns];
            continue;
        }

        const int backHeadIndex[2] = {5970, 8174};

        // calculate triangle id and uv
        delfem2::CVec3d intersection_point = { vtx_xyz1_original[3 * index], vtx_xyz1_original[3 * index + 1], vtx_xyz1_original[3 * index + 2] };
        auto near_surface_mesh = delfem2::Nearest_Point_MeshTri3D(
            intersection_point,
            head.vtx_xyz, head.tri_vtx_xyz);

        // calculate a-b / normal / a-c
       
        delfem2::CVec3d pa = delfem2::CVec3d(head.vtx_xyz.data() + head.tri_vtx_xyz[near_surface_mesh.itri * 3 + 0] * 3);
        delfem2::CVec3d pb = delfem2::CVec3d(head.vtx_xyz.data() + head.tri_vtx_xyz[near_surface_mesh.itri * 3 + 1] * 3);
        delfem2::CVec3d pc = delfem2::CVec3d(head.vtx_xyz.data() + head.tri_vtx_xyz[near_surface_mesh.itri * 3 + 2] * 3);
        delfem2::CVec3d ab = pb - pa;
        delfem2::CVec3d ac = pc - pa;
        delfem2::CVec3d bc = pc - pb;
        double ab_square = ab.squaredNorm();
        double ac_square = ac.squaredNorm();
        double bc_square = bc.squaredNorm();
        ab.normalize();
        ac.normalize();
        bc.normalize();

        delfem2::CVec3d unorm;
        unorm = ab.cross(ac);
        unorm.normalize();
        unorm = near_surface_mesh.UnitNormalOnMeshTri3(head.tri_vtx_nrm, vtx_normal);
        unorm.normalize();

        if (near_surface_mesh.itri < backHeadIndex[0] || near_surface_mesh.itri > backHeadIndex[1]) {
            index += hair_nvtx[ns];
            continue;
        }

        int xgen_face_index = (near_surface_mesh.itri - backHeadIndex[0]) * 3;

        auto u_location = near_surface_mesh.r0;
        auto v_location = near_surface_mesh.r1;

        int triangle_flag = 0;
        u_location = u_location * 2.0;
        v_location = v_location * 2.0;
        /*
        if (u_location >= 1.0 && v_location < 1.0) {
            u_location = 2.0 - u_location;
            xgen_face_index += 1;
            triangle_flag = 1;
        }
        else if(v_location >= 1.0) {
            v_location = 2.0 - v_location;
            xgen_face_index += 2;
            triangle_flag = 2;
        }
        */

        u_location = v_location = 0.0f;

        sprintf(buff, "%e %e %d",
            u_location, v_location, xgen_face_index);

        ofs << "	id			" << ns << std::endl;
        ofs << "	loc			" << buff << std::endl;
        ofs << "	blend			0.0000000000000000e+00" << ns << std::endl;
        ofs << "	interp			1.2086655547732382e+01:7.3457943937891947e+00:6.5895345249915893e-01:1.2086645547732383e+01:9.0641409043320298e-01:6.9868073085412412e+00:3.9951734312897953e+00" << std::endl;
        ofs << "	CVs			" << hair_nvtx[ns] << std::endl;
        
        delfem2::CVec3d root_position = { vtx_xyz1[3 * index], vtx_xyz1[3 * index + 1], vtx_xyz1[3 * index + 2] };
        index++;

        for (int np = 0; np < hair_nvtx[ns] - 1; np++)
        {
            delfem2::CVec3d hair_direction;
            hair_direction.x = vtx_xyz1[3 * index + 0] - root_position[0];
            hair_direction.y = vtx_xyz1[3 * index + 1] - root_position[1];
            hair_direction.z = vtx_xyz1[3 * index + 2] - root_position[2];

            delfem2::CVec3d projected_length;
            if (triangle_flag == 0)
            {
                /*
                projected_length.x = hair_direction.dot(xasis);
                projected_length.y = hair_direction.dot(unorm);
                projected_length.z = hair_direction.dot(zasis);
                */
            } else if (triangle_flag == 1){
                projected_length.x = - hair_direction.dot(bc);
                projected_length.y = hair_direction.dot(unorm);
                projected_length.z = - hair_direction.dot(ab);
            } else if (triangle_flag == 2) {
                projected_length.x = -hair_direction.dot(ac);
                projected_length.y = -hair_direction.dot(unorm);
                projected_length.z = hair_direction.dot(ab);
            }

            sprintf(buff, "%e %e %e\n",
                projected_length.x, projected_length.y, projected_length.z);
            ofs << "	" << buff;
            index++;
        }
    }

    ofs << "endObject\nendPatches\n" << std::endl;
    ofs.close();

    std::cout << "save hair models" << std::endl;
}


#endif // SAVE_SCALED_XGEN_HAIR_H_
