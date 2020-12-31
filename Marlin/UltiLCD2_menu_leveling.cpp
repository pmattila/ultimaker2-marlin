#include <avr/pgmspace.h>

#include "Configuration.h"

#ifdef ENABLE_ULTILCD2

#include "Marlin.h"
#include "cardreader.h"
#include "temperature.h"
#include "ConfigurationStore.h"
#include "machinesettings.h"
#include "UltiLCD2.h"
#include "UltiLCD2_hi_lib.h"
#include "UltiLCD2_menu_material.h"
#include "UltiLCD2_menu_maintenance.h"
#include "UltiLCD2_menu_leveling.h"
#include "UltiLCD2_menu_print.h"
#include "UltiLCD2_menu_utils.h"

static void lcd_menu_bed_level_center_adjust();
static void lcd_menu_bed_level_left_back_adjust();
static void lcd_menu_bed_level_left_front_adjust();
static void lcd_menu_bed_level_right_back_adjust();
static void lcd_menu_bed_level_right_front_adjust();

static void lcd_menu_bed_level_paper();
static void lcd_menu_bed_level_paper_center();
static void lcd_menu_bed_level_paper_left_back();
static void lcd_menu_bed_level_paper_left_front();
static void lcd_menu_bed_level_paper_right_back();
static void lcd_menu_bed_level_paper_right_front();

static void lcd_menu_bed_level_done();


static void resetAndHomeAndParkHeadForCenterAdjustment()
{
    add_homing[Z_AXIS] = 0;

    enquecommand_P(PSTR("G28 Z0 X0 Y0"));

    char buffer[32] = {0};
    sprintf_P(buffer, PSTR("G1 F%i Z%i X%i Y%i"), int(homing_feedrate[0]), 35, int(AXIS_CENTER_POS(X_AXIS)), int(AXIS_CENTER_POS(Y_AXIS)));
    enquecommand(buffer);

    menu.return_to_previous(false);
}

static void homeAndParkHeadForCenterAdjustment()
{
    enquecommand_P(PSTR("G28 Z0 X0 Y0"));

    char buffer[32] = {0};
    sprintf_P(buffer, PSTR("G1 F%i Z%i X%i Y%i"), int(homing_feedrate[0]), 10, int(AXIS_CENTER_POS(X_AXIS)), int(AXIS_CENTER_POS(Y_AXIS)));
    enquecommand(buffer);

    menu.return_to_previous(false);
}

static void parkHeadForCenterAdjustment()
{
    char buffer[32] = {0};
    sprintf_P(buffer, PSTR("G1 F%i Z5"), int(homing_feedrate[Z_AXIS]));
    enquecommand(buffer);
    sprintf_P(buffer, PSTR("G1 F%i X%i Y%i"), int(homing_feedrate[X_AXIS]), int(AXIS_CENTER_POS(X_AXIS)), int(AXIS_CENTER_POS(Y_AXIS)));
    enquecommand(buffer);
    sprintf_P(buffer, PSTR("G1 F%i Z0"), int(homing_feedrate[Z_AXIS]));
    enquecommand(buffer);
}

static void parkHeadForLeftBackAdjustment()
{
    add_homing[Z_AXIS] -= current_position[Z_AXIS];
    current_position[Z_AXIS] = 0;

    plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], active_extruder, true);

    char buffer[32] = {0};
    sprintf_P(buffer, PSTR("G1 F%i Z5"), int(homing_feedrate[Z_AXIS]));
    enquecommand(buffer);
    sprintf_P(buffer, PSTR("G1 F%i X%i Y%i"), int(homing_feedrate[X_AXIS]), max(int(min_pos[X_AXIS]),0)+10, int(max_pos[Y_AXIS])-15);
    enquecommand(buffer);
    sprintf_P(buffer, PSTR("G1 F%i Z0"), int(homing_feedrate[Z_AXIS]));
    enquecommand(buffer);
}

static void parkHeadForRightBackAdjustment()
{
    char buffer[32] = {0};
    sprintf_P(buffer, PSTR("G1 F%i Z5"), int(homing_feedrate[Z_AXIS]));
    enquecommand(buffer);
    sprintf_P(buffer, PSTR("G1 F%i X%i Y%i"), int(homing_feedrate[X_AXIS]), int(max_pos[X_AXIS])-10, int(max_pos[Y_AXIS])-15);
    enquecommand(buffer);
    sprintf_P(buffer, PSTR("G1 F%i Z0"), int(homing_feedrate[Z_AXIS]));
    enquecommand(buffer);
}

static void parkHeadForRightFrontAdjustment()
{
    char buffer[32] = {0};
    sprintf_P(buffer, PSTR("G1 F%i Z5"), int(homing_feedrate[Z_AXIS]));
    enquecommand(buffer);
    sprintf_P(buffer, PSTR("G1 F%i X%i Y%i"), int(homing_feedrate[X_AXIS]), int(max_pos[X_AXIS])-10, max(int(min_pos[Y_AXIS]),0)+15);
    enquecommand(buffer);
    sprintf_P(buffer, PSTR("G1 F%i Z0"), int(homing_feedrate[Z_AXIS]));
    enquecommand(buffer);
}

static void parkHeadForLeftFrontAdjustment()
{
    char buffer[32] = {0};
    sprintf_P(buffer, PSTR("G1 F%i Z5"), int(homing_feedrate[Z_AXIS]));
    enquecommand(buffer);
    sprintf_P(buffer, PSTR("G1 F%i X%i Y%i"), int(homing_feedrate[X_AXIS]), max(int(min_pos[X_AXIS]),0)+10, max(int(min_pos[Y_AXIS]),0)+15);
    enquecommand(buffer);
    sprintf_P(buffer, PSTR("G1 F%i Z0"), int(homing_feedrate[Z_AXIS]));
    enquecommand(buffer);
}

void lcd_menu_start_bed_leveling()
{
    lcd_question_screen(lcd_menu_bed_level_center_adjust, resetAndHomeAndParkHeadForCenterAdjustment, PSTR("CONTINUE"), NULL, lcd_change_to_previous_menu, PSTR("CANCEL"));
    lcd_lib_draw_string_centerP(10, PSTR("Place the leveling"));
    lcd_lib_draw_string_centerP(20, PSTR("card at the center"));
    lcd_lib_draw_string_centerP(30, PSTR("of the buildplate."));
    lcd_lib_update_screen();
}

void lcd_menu_start_bed_adjustment()
{
    lcd_question_screen(lcd_menu_bed_level_paper_center, homeAndParkHeadForCenterAdjustment, PSTR("CONTINUE"), NULL, lcd_change_to_previous_menu, PSTR("CANCEL"));
    lcd_lib_draw_string_centerP(10, PSTR("Place a sheet of"));
    lcd_lib_draw_string_centerP(20, PSTR("paper at the center"));
    lcd_lib_draw_string_centerP(30, PSTR("of the buildplate."));
    lcd_lib_update_screen();
}

static void lcd_menu_bed_level_center_adjust()
{
    LED_GLOW

    if (lcd_lib_encoder_pos == ENCODER_NO_SELECTION)
        lcd_lib_encoder_pos = 0;

    if (printing_state == PRINT_STATE_NORMAL && lcd_lib_encoder_pos != 0 && movesplanned() < 4)
    {
        current_position[Z_AXIS] -= float(lcd_lib_encoder_pos) * 0.05;
        plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], 60, 0);
    }
    lcd_lib_encoder_pos = 0;

    if (blocks_queued())
        lcd_info_screen(NULL, NULL, PSTR("CONTINUE"));
    else
        lcd_info_screen(lcd_menu_bed_level_left_back_adjust, parkHeadForLeftBackAdjustment, PSTR("CONTINUE"));

    lcd_lib_draw_string_centerP(10, PSTR("Rotate the button"));
    lcd_lib_draw_string_centerP(20, PSTR("until the nozzle is"));
    lcd_lib_draw_string_centerP(30, PSTR("touching the card."));
    lcd_lib_update_screen();
}

static void lcd_menu_bed_level_left_back_adjust()
{
    LED_GLOW
    SELECT_MAIN_MENU_ITEM(0);

    lcd_info_screen(lcd_menu_bed_level_right_back_adjust, parkHeadForRightBackAdjustment, PSTR("CONTINUE"));

    lcd_lib_draw_string_centerP(10, PSTR("Turn the buildplate"));
    lcd_lib_draw_string_centerP(20, PSTR("screw till the nozzle"));
    lcd_lib_draw_string_centerP(30, PSTR("is touching the card"));
    lcd_lib_update_screen();
}

static void lcd_menu_bed_level_right_back_adjust()
{
    LED_GLOW
    SELECT_MAIN_MENU_ITEM(0);

    lcd_info_screen(lcd_menu_bed_level_right_front_adjust, parkHeadForRightFrontAdjustment, PSTR("CONTINUE"));

    lcd_lib_draw_string_centerP(10, PSTR("Turn the buildplate"));
    lcd_lib_draw_string_centerP(20, PSTR("screw till the nozzle"));
    lcd_lib_draw_string_centerP(30, PSTR("is touching the card"));
    lcd_lib_update_screen();
}

static void lcd_menu_bed_level_right_front_adjust()
{
    LED_GLOW
    SELECT_MAIN_MENU_ITEM(0);

    lcd_info_screen(lcd_menu_bed_level_left_front_adjust, parkHeadForLeftFrontAdjustment, PSTR("CONTINUE"));

    lcd_lib_draw_string_centerP(10, PSTR("Turn the buildplate"));
    lcd_lib_draw_string_centerP(20, PSTR("screw till the nozzle"));
    lcd_lib_draw_string_centerP(30, PSTR("is touching the card"));
    lcd_lib_update_screen();
}

static void lcd_menu_bed_level_left_front_adjust()
{
    LED_GLOW
    SELECT_MAIN_MENU_ITEM(0);

    lcd_info_screen(lcd_menu_bed_level_paper, NULL, PSTR("CONTINUE"));

    lcd_lib_draw_string_centerP(10, PSTR("Turn the buildplate"));
    lcd_lib_draw_string_centerP(20, PSTR("screw till the nozzle"));
    lcd_lib_draw_string_centerP(30, PSTR("is touching the card"));
    lcd_lib_update_screen();
}

static void lcd_menu_bed_level_paper()
{
    LED_GLOW
    SELECT_MAIN_MENU_ITEM(0);

    lcd_info_screen(lcd_menu_bed_level_paper_center, parkHeadForCenterAdjustment, PSTR("CONTINUE"));

    lcd_lib_draw_string_centerP(10, PSTR("Now place a sheet of"));
    lcd_lib_draw_string_centerP(20, PSTR("paper at the center"));
    lcd_lib_draw_string_centerP(30, PSTR("of the buildplate."));
    lcd_lib_update_screen();
}

static void lcd_menu_bed_level_paper_center()
{
    LED_GLOW

    if (lcd_lib_encoder_pos == ENCODER_NO_SELECTION)
        lcd_lib_encoder_pos = 0;

    if (printing_state == PRINT_STATE_NORMAL && lcd_lib_encoder_pos != 0 && movesplanned() < 4)
    {
        current_position[Z_AXIS] -= float(lcd_lib_encoder_pos) * 0.05;
        lcd_lib_encoder_pos = 0;
        plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], 60, 0);
    }

    if (blocks_queued())
        lcd_info_screen(NULL, NULL, PSTR("CONTINUE"));
    else
        lcd_info_screen(lcd_menu_bed_level_paper_left_back, parkHeadForLeftBackAdjustment, PSTR("CONTINUE"));

    lcd_lib_draw_string_centerP(10, PSTR("Rotate the button"));
    lcd_lib_draw_string_centerP(20, PSTR("until you feel a"));
    lcd_lib_draw_string_centerP(30, PSTR("bit resistance"));
    lcd_lib_draw_string_centerP(40, PSTR("moving the paper."));
    lcd_lib_update_screen();
}

static void lcd_menu_bed_level_paper_left_back()
{
    LED_GLOW
    SELECT_MAIN_MENU_ITEM(0);

    lcd_info_screen(lcd_menu_bed_level_paper_right_back, parkHeadForRightBackAdjustment, PSTR("CONTINUE"));

    lcd_lib_draw_string_centerP(10, PSTR("Adjust the leveling"));
    lcd_lib_draw_string_centerP(20, PSTR("knob until you feel"));
    lcd_lib_draw_string_centerP(30, PSTR("a bit resistance"));
    lcd_lib_draw_string_centerP(40, PSTR("moving the paper."));
    lcd_lib_update_screen();
}

static void lcd_menu_bed_level_paper_right_back()
{
    LED_GLOW
    SELECT_MAIN_MENU_ITEM(0);

    lcd_info_screen(lcd_menu_bed_level_paper_right_front, parkHeadForRightFrontAdjustment, PSTR("CONTINUE"));

    lcd_lib_draw_string_centerP(10, PSTR("Adjust the leveling"));
    lcd_lib_draw_string_centerP(20, PSTR("knob until you feel"));
    lcd_lib_draw_string_centerP(30, PSTR("a bit resistance"));
    lcd_lib_draw_string_centerP(40, PSTR("moving the paper."));
    lcd_lib_update_screen();
}

static void lcd_menu_bed_level_paper_right_front()
{
    LED_GLOW
    SELECT_MAIN_MENU_ITEM(0);

    lcd_info_screen(lcd_menu_bed_level_paper_left_front, parkHeadForLeftFrontAdjustment, PSTR("CONTINUE"));

    lcd_lib_draw_string_centerP(10, PSTR("Adjust the leveling"));
    lcd_lib_draw_string_centerP(20, PSTR("knob until you feel"));
    lcd_lib_draw_string_centerP(30, PSTR("a bit resistance"));
    lcd_lib_draw_string_centerP(40, PSTR("moving the paper."));
    lcd_lib_update_screen();
}

static void storeBedLeveling()
{
    // Adjust the Z homing position to account for the thickness of the paper.
    add_homing[Z_AXIS] += LEVELING_OFFSET;

    // now that we are finished, save the settings to EEPROM
    Config_StoreSettings();

    homeAll();
}

static void lcd_menu_bed_level_paper_left_front()
{
    LED_GLOW
    SELECT_MAIN_MENU_ITEM(0);

    lcd_info_screen(lcd_menu_bed_level_done, storeBedLeveling, PSTR("DONE"));

    lcd_lib_draw_string_centerP(10, PSTR("Adjust the leveling"));
    lcd_lib_draw_string_centerP(20, PSTR("knob until you feel"));
    lcd_lib_draw_string_centerP(30, PSTR("a bit resistance"));
    lcd_lib_draw_string_centerP(40, PSTR("moving the paper."));
    lcd_lib_update_screen();
}

static void lcd_menu_bed_level_done()
{
    menu.return_to_previous(true);
}

#endif//ENABLE_ULTILCD2
