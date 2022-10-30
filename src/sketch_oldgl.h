//
// Created by Nobuyuki Umetani on 2021-10-06.
//

#ifndef SKETCH_OLDGL_H_
#define SKETCH_OLDGL_H_

void DrawSketch(
    const std::vector<delfem2::CVec2f> &polyline) {
  ::glMatrixMode(GL_PROJECTION);
  ::glLoadIdentity();
  ::glMatrixMode(GL_MODELVIEW);
  ::glLoadIdentity();
  ::glDisable(GL_DEPTH_TEST);
  ::glDisable(GL_TEXTURE_2D);
  ::glLineWidth(3);
  ::glColor3d(1, 1, 1);
  ::glBegin(GL_LINE_STRIP);
  for (const auto &p: polyline) {
    ::glVertex2d(p.x, p.y);
  }
  ::glEnd();
}

#endif //SKETCH_OLDGL_H_
