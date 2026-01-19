//
// Created by RM UI Designer
// Dynamic Edition
//

#ifndef UI_booster_on_H
#define UI_booster_on_H

#include "ui_interface.h"

extern ui_interface_figure_t ui_booster_on_now_figures[3];
extern uint8_t ui_booster_on_dirty_figure[3];
extern ui_interface_string_t ui_booster_on_now_strings[3];
extern uint8_t ui_booster_on_dirty_string[3];

#define ui_booster_on_static_group_autoaim_arc_right ((ui_interface_arc_t*)&(ui_booster_on_now_figures[0]))
#define ui_booster_on_static_group_autoaim_arc_left ((ui_interface_arc_t*)&(ui_booster_on_now_figures[1]))
#define ui_booster_on_static_group_preaim_line ((ui_interface_line_t*)&(ui_booster_on_now_figures[2]))

#define ui_booster_on_dynamic_group_booster_on_text (&(ui_booster_on_now_strings[0]))
#define ui_booster_on_dynamic__group_spin_on_text (&(ui_booster_on_now_strings[1]))
#define ui_booster_on_dynamic__group_cap_discharge_text (&(ui_booster_on_now_strings[2]))

#ifdef MANUAL_DIRTY
#define ui_booster_on_static_group_autoaim_arc_right_dirty (ui_booster_on_dirty_figure[0])
#define ui_booster_on_static_group_autoaim_arc_left_dirty (ui_booster_on_dirty_figure[1])
#define ui_booster_on_static_group_preaim_line_dirty (ui_booster_on_dirty_figure[2])

#define ui_booster_on_dynamic_group_booster_on_text_dirty (ui_booster_on_dirty_string[0])
#define ui_booster_on_dynamic__group_spin_on_text_dirty (ui_booster_on_dirty_string[1])
#define ui_booster_on_dynamic__group_cap_discharge_text_dirty (ui_booster_on_dirty_string[2])
#endif

void ui_init_booster_on();
void ui_update_booster_on();

#endif // UI_booster_on_H
