//
// Created by Shinichi Kinuwaki on 2021-10-21.
//

#ifndef HAIR_IO_STATE_H_
#define HAIR_IO_STATE_H_

#include <pugixml.hpp>

void SaveHairState(
    const std::vector<Hair>& hairs,
    const float& stiff_bend,
    const float gravity[],
    const float& stiff_contact,
    const std::string& xml_path) {

    pugi::xml_document doc;
    auto version_node = doc.append_child("hair_data");
    version_node.append_attribute("version") = 0.01;

    // add parameters
    pugi::xml_node stiff_bend_node = doc.append_child("stiff_bend");
    stiff_bend_node.append_attribute("value") = stiff_bend;

    pugi::xml_node gravity_node = doc.append_child("gravity_x");
    gravity_node.append_attribute("value") = gravity[0];
    gravity_node = doc.append_child("gravity_y");
    gravity_node.append_attribute("value") = gravity[1];
    gravity_node = doc.append_child("gravity_z");
    gravity_node.append_attribute("value") = gravity[2];

    pugi::xml_node stiff_contact_node = doc.append_child("stiff_contact");
    stiff_contact_node.append_attribute("value") = stiff_contact;

    // add hair informations
    pugi::xml_node hairs_node = doc.append_child("hairs");
    hairs_node.append_attribute("size") = hairs.size();

    int stroke_id = 0;

    for (unsigned int nh = 0; nh < hairs.size(); nh++)
    {
        pugi::xml_node hair_node = hairs_node.append_child("hair");

        // adding id
        hair_node.append_attribute("id") = nh;

        // adding is_converged
        {
            pugi::xml_node is_converged_node = hair_node.append_child("is_converged");
            is_converged_node.append_attribute("value") = hairs[nh].is_converged;
        }

        // adding camera_stroke
        {
            pugi::xml_node camera_stroke_node = hair_node.append_child("camera_stroke");
            camera_stroke_node.append_attribute("size") = hairs[nh].camera_stroke.size();
            int index = 0;
            for (auto itr = hairs[nh].camera_stroke.begin(); itr != hairs[nh].camera_stroke.end(); ++itr) {
                pugi::xml_node stroke_node = camera_stroke_node.append_child("stroke");
                stroke_node.append_attribute("id") = stroke_id;
                stroke_id++;
                stroke_node.append_attribute("camera_id") = itr->first;
                stroke_node.append_attribute("size") = itr->second.xys.size();
                std::string xys_string;
                for (int nxy = 0; nxy < itr->second.xys.size(); nxy++)
                {
                    xys_string += std::to_string(itr->second.xys[nxy].x) + " ";
                    xys_string += std::to_string(itr->second.xys[nxy].y) + " ";
                }
                stroke_node.append_attribute("value") = xys_string.c_str();
                index++;
            }
        }

        // adding vtx_xyz
        {
            pugi::xml_node vtx_xyz_node = hair_node.append_child("vtx_xyz");
            vtx_xyz_node.append_attribute("size") = hairs[nh].vtx_xyz.size();
            std::string vtx_xyz_string;
            for (float nvtx : hairs[nh].vtx_xyz)
            {
                vtx_xyz_string += std::to_string(nvtx) + " ";
            }
            vtx_xyz_node.append_attribute("value") = vtx_xyz_string.c_str();
        }

        // adding root_on_mesh
        {
            pugi::xml_node root_on_mesh_node = hair_node.append_child("root_on_mesh");
            root_on_mesh_node.append_attribute("itri") = hairs[nh].root_on_mesh.itri;
            root_on_mesh_node.append_attribute("r0") = hairs[nh].root_on_mesh.r0;
            root_on_mesh_node.append_attribute("r1") = hairs[nh].root_on_mesh.r1;
        }

        // adding root_pos
        {
            pugi::xml_node root_pos_node = hair_node.append_child("root_pos");
            std::string root_pos_string;
            root_pos_string += std::to_string(hairs[nh].root_pos.x) + " ";
            root_pos_string += std::to_string(hairs[nh].root_pos.y) + " ";
            root_pos_string += std::to_string(hairs[nh].root_pos.z);
            root_pos_node.append_attribute("value") = root_pos_string.c_str();
        }

        // adding root_normal
        {
            pugi::xml_node root_normal_node = hair_node.append_child("root_normal");
            std::string root_normal_string;
            root_normal_string += std::to_string(hairs[nh].root_normal.x) + " ";
            root_normal_string += std::to_string(hairs[nh].root_normal.y) + " ";
            root_normal_string += std::to_string(hairs[nh].root_normal.z);
            root_normal_node.append_attribute("value") = root_normal_string.c_str();
        }

        // adding root_rotation
        {
            pugi::xml_node root_rotation_node = hair_node.append_child("root_rotation");
            std::string root_rotation_string;
            root_rotation_string += std::to_string(hairs[nh].root_rotation.x) + " ";
            root_rotation_string += std::to_string(hairs[nh].root_rotation.y) + " ";
            root_rotation_string += std::to_string(hairs[nh].root_rotation.z) + " ";
            root_rotation_string += std::to_string(hairs[nh].root_rotation.w);
            root_rotation_node.append_attribute("value") = root_rotation_string.c_str();
        }
    }

    bool saveSucceeded = doc.save_file(xml_path.c_str(), PUGIXML_TEXT("  "));
    assert(saveSucceeded);
}

template <typename T>
void LoadHairParameterNode(
    T& value,
    const pugi::xml_node& node,
    const std::string& node_name) {

    pugi::xml_node selected_node = node.select_node(node_name.c_str()).node();
    assert(selected_node);
    pugi::xml_attribute attribute = selected_node.attribute("value");
    assert(attribute);

    if constexpr (std::is_same_v<T, float>)
        value = attribute.as_float();
    else if constexpr (std::is_same_v<T, double>)
        value = attribute.as_double();
    else if constexpr (std::is_same_v<T, int>)
        value = attribute.as_int();
    else
        std::cout << "please implement here" << std::endl;
}

void LoadHairState(
    std::vector<Hair>& hairs,
    float& stiff_bend,
    float gravity[],
    float& stiff_contact,
    const std::string& xml_path) {

    int stroke_id = 0;
    bool in_searchFirst = true;

    pugi::xml_document doc;
    const int MAX_CHAR = 1024;
    char search_string[MAX_CHAR];

    doc.load_file(xml_path.c_str());
    pugi::xml_node root = doc.document_element();
    pugi::xml_node hair_data_node = doc.select_node("/hair_data").node();

    if (hair_data_node) {
        LoadHairParameterNode(stiff_bend, hair_data_node, "/stiff_bend");
        LoadHairParameterNode(gravity[0], hair_data_node, "/gravity_x");
        LoadHairParameterNode(gravity[1], hair_data_node, "/gravity_y");
        LoadHairParameterNode(gravity[2], hair_data_node, "/gravity_z");
        LoadHairParameterNode(stiff_contact, hair_data_node, "/stiff_contact");

        pugi::xml_node hairs_node = hair_data_node.select_node("/hairs").node();
        assert(hairs_node);
        pugi::xml_attribute attribute = hairs_node.attribute("size");
        assert(attribute);
        int hair_size = attribute.as_int();
        hairs.resize(hair_size);
                
        for (int ns = 0; ns < hair_size; ns++) {
            sprintf(search_string, "//hair[@id=\"%d\"]", ns);
            pugi::xml_node hair_node = hairs_node.select_node(search_string).node();

            // fill is_converged
            {
                pugi::xml_node is_converged_node = hair_node.select_node("is_converged").node();
                attribute = is_converged_node.attribute("value");
                assert(attribute);
                hairs[ns].is_converged = attribute.as_bool();
            }

            // fill camera_stroke
            {
                pugi::xml_node camera_stroke_node = hair_node.select_node("camera_stroke").node();
                attribute = camera_stroke_node.attribute("size");
                assert(attribute);
                int camera_stroke_size = attribute.as_int();

                // fill stroke
                for (int ncs = 0; ncs < camera_stroke_size; ncs++) {
                    sprintf(search_string, "//stroke[@id=\"%d\"]", stroke_id);
                    stroke_id++;
                    pugi::xml_node stroke_node = camera_stroke_node.select_node(search_string).node();
                    attribute = stroke_node.attribute("camera_id");
                    assert(attribute);
                    int camera_id = attribute.as_int();
                    attribute = stroke_node.attribute("size");
                    assert(attribute);
                    int stroke_size = attribute.as_int();
                    hairs[ns].camera_stroke[camera_id].xys.resize(stroke_size);

                    attribute = stroke_node.attribute("value");
                    assert(attribute);
                    std::string strand_string = attribute.as_string();

                    int count = 0;
                    std::string item_string;
                    std::stringstream string_stream(strand_string);
                    while (std::getline(string_stream, item_string, ' ')) {
                        assert(count < stroke_size * 2);
                        if (count % 2 == 0)
                            hairs[ns].camera_stroke[camera_id].xys[count / 2].x = std::stod(item_string);
                        else
                            hairs[ns].camera_stroke[camera_id].xys[count / 2].y = std::stod(item_string);
                        count++;
                    }
                }
            }

            // fill vtx_xyz
            {
                pugi::xml_node vtx_xyz_node = hair_node.select_node("vtx_xyz").node();
                assert(vtx_xyz_node);

                attribute = vtx_xyz_node.attribute("size");
                assert(attribute);
                int vtx_xyz_count = attribute.as_int();

                attribute = vtx_xyz_node.attribute("value");
                assert(attribute);
                std::string strand_string = attribute.as_string();
                hairs[ns].vtx_xyz.resize(vtx_xyz_count);

                int count = 0;
                std::string item_string;
                std::stringstream string_stream(strand_string);
                while (std::getline(string_stream, item_string, ' ')) {
                    assert(count < vtx_xyz_count);
                    hairs[ns].vtx_xyz[count] = std::stof(item_string);
                    count++;
                }
            }

            // fill root_on_mesh
            {
                pugi::xml_node root_on_mesh_node = hair_node.select_node("root_on_mesh").node();
                assert(root_on_mesh_node);
                attribute = root_on_mesh_node.attribute("itri");
                assert(attribute);
                hairs[ns].root_on_mesh.itri = attribute.as_uint();

                attribute = root_on_mesh_node.attribute("r0");
                assert(attribute);
                hairs[ns].root_on_mesh.r0 = attribute.as_double();

                attribute = root_on_mesh_node.attribute("r1");
                assert(attribute);
                hairs[ns].root_on_mesh.r1 = attribute.as_double();
            }

            // fill root_pos
            {
                pugi::xml_node root_pos_node = hair_node.select_node("root_pos").node();
                assert(root_pos_node);
                attribute = root_pos_node.attribute("value");
                assert(attribute);
                std::string strand_string = attribute.as_string();

                int count = 0;
                std::string item_string;
                std::stringstream string_stream(strand_string);
                for (int nrp = 0; nrp < 3; nrp++) {
                    std::getline(string_stream, item_string, ' ');
                    hairs[ns].root_pos[nrp] = std::stof(item_string);
                }
            }

            // fill root_normal
            {
                pugi::xml_node root_normal_node = hair_node.select_node("root_normal").node();
                assert(root_normal_node);
                attribute = root_normal_node.attribute("value");
                assert(attribute);
                std::string strand_string = attribute.as_string();

                int count = 0;
                std::string item_string;
                std::stringstream string_stream(strand_string);
                for (int nrn = 0; nrn < 3; nrn++) {
                    std::getline(string_stream, item_string, ' ');
                    hairs[ns].root_normal[nrn] = std::stof(item_string);
                }
            }

            // fill root_rotation
            {
                pugi::xml_node root_rotation_node = hair_node.select_node("root_rotation").node();
                assert(root_rotation_node);
                attribute = root_rotation_node.attribute("value");
                assert(attribute);
                std::string strand_string = attribute.as_string();

                int count = 0;
                std::string item_string;
                std::stringstream string_stream(strand_string);
                std::getline(string_stream, item_string, ' ');
                hairs[ns].root_rotation.x = std::stof(item_string);
                std::getline(string_stream, item_string, ' ');
                hairs[ns].root_rotation.y = std::stof(item_string);
                std::getline(string_stream, item_string, ' ');
                hairs[ns].root_rotation.z = std::stof(item_string);
                std::getline(string_stream, item_string, ' ');
                hairs[ns].root_rotation.w = std::stof(item_string);
            }
        }
    }
}

#endif // HAIR_IO_STATE_H_
