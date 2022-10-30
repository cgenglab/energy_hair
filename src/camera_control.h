//
// Created by Shinichi Kinuwaki on 2021-12-05.
//

#ifndef CAMERA_CONTROL_H_
#define CAMERA_CONTROL_H_

class CameraControl {
  public:
    CameraControl() {
      width = height = current_x = current_y = 0;
      valid_camera_control = false;
      has_camera_position = false;
    }

    bool ReadXML(const std::string xml_path) {
        pugi::xml_document doc;
        const int MAX_CHAR = 1024;
        char search_string[MAX_CHAR];

        doc.load_file(xml_path.c_str());
        pugi::xml_node root = doc.document_element();
        pugi::xml_node photo_array_node = doc.select_node("/photo_array").node();
        pugi::xml_attribute attribute = photo_array_node.attribute("width");
        if (!attribute) return false;
        width = attribute.as_int();
        if (width <= 0) return false;
        attribute = photo_array_node.attribute("height");
        if (!attribute) return false;
        height = attribute.as_int();
        if (height <= 0) return false;

        fileindex_array.resize(width);
        for (int nw = 0; nw < width; nw++) {
            fileindex_array[nw].resize(height);
        }
        camera_position_array.resize(width * height * 3);

        bool has_position_flag = true;

        if (photo_array_node) {
            for (int nw = 0; nw < width; nw++) {
                sprintf(search_string, "//photo[@id=\"%d\"]", nw);
                pugi::xml_node photo_node = photo_array_node.select_node(search_string).node();

                for (int nh = 0; nh < height; nh++) {
                    sprintf(search_string, "height_%d_idx", nh);
                    attribute = photo_node.attribute(search_string);
                    if (!attribute) return false;
                    fileindex_array[nw][nh] = attribute.as_int();

                    sprintf(search_string, "position_%d", nh);
                    attribute = photo_node.attribute(search_string);
                    if (!attribute) {
                        has_position_flag = false;
                        continue;
                    }

                    std::string strand_string = attribute.as_string();

                    std::string item_string;
                    std::stringstream string_stream(strand_string);
                    int xyz_index = 0;
                    while (std::getline(string_stream, item_string, ' ')) {
                        camera_position_array[(nh + nw * height) * 3 + xyz_index] = std::stof(item_string);
                        xyz_index++;
                    }
                }
            }
        }
        
        valid_camera_control = true;
        has_camera_position = has_position_flag;
        return true;
    }

    int MoveUp() {
        current_y = std::min(current_y + 1, height - 1);
        return GetCurrentIndex();
    }
    int MoveDown() {
        current_y = std::max(current_y - 1, 0);
        return GetCurrentIndex();
    }
    int MoveRight() {
        if (current_x == width - 1)
            current_x = 0;
        else
            current_x = std::min(current_x + 1, width - 1);
        return GetCurrentIndex();
    }
    int MoveLeft() {
        if (current_x == 0)
            current_x = width - 1;
        else
            current_x = std::max(current_x - 1, 0);
        return GetCurrentIndex();
    }

    int GetCurrentIndex() {
        return fileindex_array[current_x][current_y];
    }
  public:
    bool valid_camera_control;
    bool has_camera_position;
    int current_x, current_y;
    int width, height;
    std::vector< std::vector<int> > fileindex_array;
    std::vector< float > camera_position_array;
};


#endif
