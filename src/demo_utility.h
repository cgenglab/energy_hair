//
// Created by Shinichi Kinuwaki on 2022-01-07.
//

#pragma once

#include "hair_io_state.h"

class DemoUtility {
public:
    // pointers
    SimulationParam *p_sim_param, sim_param_initial;

    imgui_addons::ImGuiFileBrowser imgui_file_browser;
    bool imgui_file_browser_opening;
    bool show_open_dialog;
    bool show_save_dialog;
    bool show_export_dialog;
    bool quit_application_flag;

    DemoUtility(SimulationParam& sim_param, const SimulationParam& _sim_param_initial) {
        p_sim_param = &sim_param;
        sim_param_initial = _sim_param_initial;

        imgui_file_browser_opening = false;
        show_open_dialog = false;
        show_save_dialog = false;
        show_export_dialog = false;
        quit_application_flag = false;
    };

    void PostImGui(MyViewer& viewer, std::vector<Hair>& hairs, const HeadMesh& head) {
        bool run_undo_operation = viewer.is_undo_operation;
        bool run_redo_operation = viewer.is_redo_operation;;

        if (ImGui::BeginMainMenuBar()) {
            if (ImGui::BeginMenu("File")) {
                if (ImGui::MenuItem("New")) { *p_sim_param = sim_param_initial; }
                if (ImGui::MenuItem("Open", "Ctrl+O")) { show_open_dialog = true; }
                if (ImGui::MenuItem("Save", "Ctrl+S")) { show_save_dialog = true; }
                if (ImGui::MenuItem("Export", "Ctrl+E")) { show_export_dialog = true; }
                if (ImGui::MenuItem("Quit", "Ctrl+Q")) { quit_application_flag = true; }
                ImGui::EndMenu();
            }

            if (ImGui::BeginMenu("Edit")) {
                if (ImGui::MenuItem("Undo", "Z")) {
                    run_undo_operation = true;
                }

                bool redo_feasibility = false;
                if (ImGui::MenuItem("Redo", "Y", false, redo_feasibility)) {
                    run_redo_operation = true;
                }
                ImGui::EndMenu();
            }

            if (ImGui::BeginMenu("Edit"))
            {
                if (ImGui::MenuItem("Undo", "Z")) {
                    run_undo_operation = true;
                }

                bool redo_feasibility = false;
                if (ImGui::MenuItem("Redo", "Y", false, redo_feasibility)) {
                    run_redo_operation = true;
                }
                ImGui::EndMenu();
            }

            ImGui::EndMainMenuBar();
        }

        // undo operation
        if (run_undo_operation) {
            // todo : implement
            viewer.is_undo_operation = false;
        }

        // redo operation
        if (run_redo_operation) {
            // todo : implement
            viewer.is_redo_operation = false;
        }

        if (imgui_file_browser_opening == true) {
            viewer.ResetOperation();
        }

        if (show_save_dialog || viewer.is_save_operation) {
            ImGui::OpenPopup("Save File");
            show_save_dialog = false;
            imgui_file_browser_opening = true;
        }
        else if (show_open_dialog || viewer.is_open_operation) {
            ImGui::OpenPopup("Open File");
            show_open_dialog = false;
            imgui_file_browser_opening = true;
        }
        else if (show_export_dialog || viewer.is_export_operation) {
            ImGui::OpenPopup("Export File");
            show_export_dialog = false;
            imgui_file_browser_opening = true;
        }

        if (imgui_file_browser.showFileDialog("Open File", imgui_addons::ImGuiFileBrowser::DialogMode::OPEN, ImVec2(700, 310), ".xml")) {
            LoadHairState(hairs, p_sim_param->stiff_bend, p_sim_param->gravity_times_rho, p_sim_param->stiff_contact, imgui_file_browser.selected_path);
            imgui_file_browser_opening = false;
        } else if (imgui_file_browser.showFileDialog("Save File", imgui_addons::ImGuiFileBrowser::DialogMode::SAVE, ImVec2(700, 310), ".xml")) {
            SaveHairState(hairs, p_sim_param->stiff_bend, p_sim_param->gravity_times_rho, p_sim_param->stiff_contact, imgui_file_browser.selected_path);
            imgui_file_browser_opening = false;
        } else if (imgui_file_browser.showFileDialog("Export File", imgui_addons::ImGuiFileBrowser::DialogMode::SAVE, ImVec2(700, 310), ".abc")) {
            const auto path_head = "head.obj";
            auto path_hair = std::filesystem::path(imgui_file_browser.selected_path);
            SaveScaledHeadHair(path_head, path_hair,
                head, hairs, viewer.cam_config.region_center);
        }
    }
};


