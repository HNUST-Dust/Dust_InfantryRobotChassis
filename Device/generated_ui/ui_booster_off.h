//
// Created by RM UI Designer
// Dynamic Edition
//

#ifndef UI_booster_off_H
#define UI_booster_off_H

#include "ui_interface.h"

extern ui_interface_figure_t ui_booster_off_now_figures[3];
extern uint8_t ui_booster_off_dirty_figure[3];
extern ui_interface_string_t ui_booster_off_now_strings[3];
extern uint8_t ui_booster_off_dirty_string[3];

#define ui_booster_off_static_group_preaim_line ((ui_interface_line_t*)&(ui_booster_off_now_figures[0]))
#define ui_booster_off_static_group_autoaim_arc_left ((ui_interface_arc_t*)&(ui_booster_off_now_figures[1]))
#define ui_booster_off_static_group_autoaim_arc_right ((ui_interface_arc_t*)&(ui_booster_off_now_figures[2]))

#define ui_booster_off_dynamic_group_booster_off_text (&(ui_booster_off_now_strings[0]))
#define ui_booster_off_dynamic__group_spin_off_text (&(ui_booster_off_now_strings[1]))
#define ui_booster_off_dynamic__group_cap_charge_text (&(ui_booster_off_now_strings[2]))

#ifdef MANUAL_DIRTY
#define ui_booster_off_static_group_preaim_line_dirty (ui_booster_off_dirty_figure[0])
#define ui_booster_off_static_group_autoaim_arc_left_dirty (ui_booster_off_dirty_figure[1])
#define ui_booster_off_static_group_autoaim_arc_right_dirty (ui_booster_off_dirty_figure[2])

#define ui_booster_off_dynamic_group_booster_off_text_dirty (ui_booster_off_dirty_string[0])
#define ui_booster_off_dynamic__group_spin_off_text_dirty (ui_booster_off_dirty_string[1])
#define ui_booster_off_dynamic__group_cap_charge_text_dirty (ui_booster_off_dirty_string[2])
#endif

void ui_init_booster_off();
void ui_update_booster_off();

#endif // UI_booster_off_H
