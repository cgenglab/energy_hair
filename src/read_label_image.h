//
// Created by Nobuyuki Umetani on 2021-08-29.
//

#ifndef READ_LABEL_IMAGE_H
#define READ_LABEL_IMAGE_H

#define STB_IMAGE_IMPLEMENTATION
#include "stb/stb_image.h"

void ReadLabelImage(
    std::vector<int> &aFlag,
    const std::string &path_seg) {
  int width = 0, height = 0, num_channel = 0;
  unsigned char *buff = stbi_load(path_seg.c_str(),
                                  &width, &height, &num_channel, 4);
  std::cout << width << " " << height << " " << num_channel << std::endl;
  aFlag.resize(width * height, 0);
  for (int ih = 0; ih < height; ++ih) {
    for (int iw = 0; iw < width; ++iw) {
      unsigned char r = buff[(ih * width + iw) * 4 + 0];
      unsigned char g = buff[(ih * width + iw) * 4 + 1];
      unsigned char b = buff[(ih * width + iw) * 4 + 2];
      unsigned char a = buff[(ih * width + iw) * 4 + 3];
      if (r == 0 && g == 255 && b == 0 && a == 255) { aFlag[ih * width + iw] = 1; } // remove
      if (r == 0 && g == 0 && b == 255 && a == 255) { aFlag[ih * width + iw] = 2; } // remove
    }
  }
  stbi_image_free(buff);
}

#endif // READ_LABEL_IMAGE_H
