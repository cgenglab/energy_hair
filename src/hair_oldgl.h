//
// Created by Nobuyuki Umetani on 2021-10-06.
//

#ifndef HAIR_OLDGL_H_
#define HAIR_OLDGL_H_

#include "delfem2/opengl/old/funcs.h"

void DrawHair(
    const std::vector<float> &vtx_xyz,
    bool is_selected,
    bool has_sketch ) {
  const size_t num_vtx = vtx_xyz.size() / 3;

//  ::glDisable(GL_DEPTH_TEST);
  ::glDisable(GL_TEXTURE_2D);
  ::glDisable(GL_LIGHTING);

  // draw polyline
  if( !is_selected) {
    if( has_sketch ) {
      ::glColor3d(0, 0, 0.5);
    }
    else{
      ::glColor3d(0, 0, 0);
    }
  }
  else{
    ::glColor3d(1, 0, 0);
  }
  if (num_vtx > 0) {
    ::glLineWidth(1.f);
    ::glBegin(GL_LINE_STRIP);
    for (unsigned int ivtx = 0; ivtx < num_vtx - 1; ++ivtx) {
      unsigned int jvtx = ivtx + 1;
      ::glVertex3fv(vtx_xyz.data() + ivtx * 3);
      ::glVertex3fv(vtx_xyz.data() + jvtx * 3);
    }
    ::glEnd();
  }

  // draw points
  for (unsigned int ivtx = 0; ivtx < num_vtx; ++ivtx) {
    if( !is_selected ){
      if( has_sketch ) {
        ::glColor3d(0, 0, 0.5);
      }
      else {
        ::glColor3d(0, 0, 0);
      }
    }
    else {
      if (ivtx == 0) {
        ::glColor3d(0, 1, 0);
      } else {
        ::glColor3d(0, 0, 1);
      }
    }
    delfem2::opengl::DrawSphereAt(
        32, 32, 0.01,
        vtx_xyz[ivtx * 3 + 0],
        vtx_xyz[ivtx * 3 + 1],
        vtx_xyz[ivtx * 3 + 2]);
  }
}

#endif //HAIR_OLDGL_H_
