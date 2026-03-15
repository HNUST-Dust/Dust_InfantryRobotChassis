//
// Created by RM UI Designer
// Dynamic Edition
//

#include "string.h"
#include "ui_interface.h"
#include "ui_booster_off.h"

#define TOTAL_FIGURE 3
#define TOTAL_STRING 3

ui_interface_figure_t ui_booster_off_now_figures[TOTAL_FIGURE];
uint8_t ui_booster_off_dirty_figure[TOTAL_FIGURE];
ui_interface_string_t ui_booster_off_now_strings[TOTAL_STRING];
uint8_t ui_booster_off_dirty_string[TOTAL_STRING];

#ifndef MANUAL_DIRTY
ui_interface_figure_t ui_booster_off_last_figures[TOTAL_FIGURE];
ui_interface_string_t ui_booster_off_last_strings[TOTAL_STRING];
#endif

#define SCAN_AND_SEND() ui_scan_and_send(ui_booster_off_now_figures, ui_booster_off_dirty_figure, ui_booster_off_now_strings, ui_booster_off_dirty_string, TOTAL_FIGURE, TOTAL_STRING)

void ui_init_booster_off() {
    ui_booster_off_static_group_preaim_line->figure_type = 0;
    ui_booster_off_static_group_preaim_line->operate_type = 1;
    ui_booster_off_static_group_preaim_line->layer = 0;
    ui_booster_off_static_group_preaim_line->color = 1;
    ui_booster_off_static_group_preaim_line->start_x = 900;
    ui_booster_off_static_group_preaim_line->start_y = 400;
    ui_booster_off_static_group_preaim_line->width = 1;
    ui_booster_off_static_group_preaim_line->end_x = 1025;
    ui_booster_off_static_group_preaim_line->end_y = 400;

    ui_booster_off_static_group_autoaim_arc_left->figure_type = 4;
    ui_booster_off_static_group_autoaim_arc_left->operate_type = 1;
    ui_booster_off_static_group_autoaim_arc_left->layer = 0;
    ui_booster_off_static_group_autoaim_arc_left->color = 3;
    ui_booster_off_static_group_autoaim_arc_left->start_x = 720;
    ui_booster_off_static_group_autoaim_arc_left->start_y = 540;
    ui_booster_off_static_group_autoaim_arc_left->width = 2;
    ui_booster_off_static_group_autoaim_arc_left->start_angle = 225;
    ui_booster_off_static_group_autoaim_arc_left->end_angle = 315;
    ui_booster_off_static_group_autoaim_arc_left->rx = 150;
    ui_booster_off_static_group_autoaim_arc_left->ry = 200;

    ui_booster_off_static_group_autoaim_arc_right->figure_type = 4;
    ui_booster_off_static_group_autoaim_arc_right->operate_type = 1;
    ui_booster_off_static_group_autoaim_arc_right->layer = 0;
    ui_booster_off_static_group_autoaim_arc_right->color = 3;
    ui_booster_off_static_group_autoaim_arc_right->start_x = 1200;
    ui_booster_off_static_group_autoaim_arc_right->start_y = 540;
    ui_booster_off_static_group_autoaim_arc_right->width = 2;
    ui_booster_off_static_group_autoaim_arc_right->start_angle = 45;
    ui_booster_off_static_group_autoaim_arc_right->end_angle = 135;
    ui_booster_off_static_group_autoaim_arc_right->rx = 150;
    ui_booster_off_static_group_autoaim_arc_right->ry = 200;

    ui_booster_off_dynamic_group_booster_off_text->figure_type = 7;
    ui_booster_off_dynamic_group_booster_off_text->operate_type = 1;
    ui_booster_off_dynamic_group_booster_off_text->layer = 1;
    ui_booster_off_dynamic_group_booster_off_text->color = 5;
    ui_booster_off_dynamic_group_booster_off_text->start_x = 120;
    ui_booster_off_dynamic_group_booster_off_text->start_y = 540;//260
    ui_booster_off_dynamic_group_booster_off_text->width = 2;
    ui_booster_off_dynamic_group_booster_off_text->font_size = 20;
    ui_booster_off_dynamic_group_booster_off_text->str_length = 11;
    strcpy(ui_booster_off_dynamic_group_booster_off_text->string, "booster_off");

    ui_booster_off_dynamic__group_spin_off_text->figure_type = 7;
    ui_booster_off_dynamic__group_spin_off_text->operate_type = 1;
    ui_booster_off_dynamic__group_spin_off_text->layer = 1;
    ui_booster_off_dynamic__group_spin_off_text->color = 5;
    ui_booster_off_dynamic__group_spin_off_text->start_x = 120;
    ui_booster_off_dynamic__group_spin_off_text->start_y = 590;//302
    ui_booster_off_dynamic__group_spin_off_text->width = 2;
    ui_booster_off_dynamic__group_spin_off_text->font_size = 20;
    ui_booster_off_dynamic__group_spin_off_text->str_length = 8;
    strcpy(ui_booster_off_dynamic__group_spin_off_text->string, "spin_off");

    ui_booster_off_dynamic__group_cap_charge_text->figure_type = 7;
    ui_booster_off_dynamic__group_cap_charge_text->operate_type = 1;
    ui_booster_off_dynamic__group_cap_charge_text->layer = 1;
    ui_booster_off_dynamic__group_cap_charge_text->color = 5;
    ui_booster_off_dynamic__group_cap_charge_text->start_x = 120;
    ui_booster_off_dynamic__group_cap_charge_text->start_y = 640;//352
    ui_booster_off_dynamic__group_cap_charge_text->width = 2;
    ui_booster_off_dynamic__group_cap_charge_text->font_size = 20;
    ui_booster_off_dynamic__group_cap_charge_text->str_length = 10;
    strcpy(ui_booster_off_dynamic__group_cap_charge_text->string, "cap_charge");

    uint32_t idx = 0;
    for (int i = 0; i < TOTAL_FIGURE; i++) {
        ui_booster_off_now_figures[i].figure_name[2] = idx & 0xFF;
        ui_booster_off_now_figures[i].figure_name[1] = (idx >> 8) & 0xFF;
        ui_booster_off_now_figures[i].figure_name[0] = (idx >> 16) & 0xFF;
        ui_booster_off_now_figures[i].operate_type = 1;
#ifndef MANUAL_DIRTY
        ui_booster_off_last_figures[i] = ui_booster_off_now_figures[i];
#endif
        ui_booster_off_dirty_figure[i] = 1;
        idx++;
    }
    for (int i = 0; i < TOTAL_STRING; i++) {
        ui_booster_off_now_strings[i].figure_name[2] = idx & 0xFF;
        ui_booster_off_now_strings[i].figure_name[1] = (idx >> 8) & 0xFF;
        ui_booster_off_now_strings[i].figure_name[0] = (idx >> 16) & 0xFF;
        ui_booster_off_now_strings[i].operate_type = 1;
#ifndef MANUAL_DIRTY
        ui_booster_off_last_strings[i] = ui_booster_off_now_strings[i];
#endif
        ui_booster_off_dirty_string[i] = 1;
        idx++;
    }

    SCAN_AND_SEND();

    for (int i = 0; i < TOTAL_FIGURE; i++) {
        ui_booster_off_now_figures[i].operate_type = 2;
    }
    for (int i = 0; i < TOTAL_STRING; i++) {
        ui_booster_off_now_strings[i].operate_type = 2;
    }
}

void ui_update_booster_off() {
#ifndef MANUAL_DIRTY
    for (int i = 0; i < TOTAL_FIGURE; i++) {
        if (memcmp(&ui_booster_off_now_figures[i], &ui_booster_off_last_figures[i], sizeof(ui_booster_off_now_figures[i])) != 0) {
            ui_booster_off_dirty_figure[i] = 1;
            ui_booster_off_last_figures[i] = ui_booster_off_now_figures[i];
        }
    }
    for (int i = 0; i < TOTAL_STRING; i++) {
        if (memcmp(&ui_booster_off_now_strings[i], &ui_booster_off_last_strings[i], sizeof(ui_booster_off_now_strings[i])) != 0) {
            ui_booster_off_dirty_string[i] = 1;
            ui_booster_off_last_strings[i] = ui_booster_off_now_strings[i];
        }
    }
#endif
    SCAN_AND_SEND();
}
