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

static void lcd_menu_first_run_init_2();
static void lcd_menu_first_run_init_3();

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
static void lcd_menu_first_run_done();


#define DRAW_PROGRESS_NR_IF_NOT_DONE(nr) do { if (!IS_FIRST_RUN_DONE()) { lcd_lib_draw_stringP((nr < 10) ? 100 : 94, 0, PSTR( #nr "/15")); } } while(0)
#define DRAW_PROGRESS_NR(nr) do { lcd_lib_draw_stringP((nr < 10) ? 100 : 94, 0, PSTR( #nr "/15")); } while(0)
#define CLEAR_PROGRESS_NR(nr) do { lcd_lib_clear_stringP((nr < 10) ? 100 : 94, 0, PSTR( #nr "/15")); } while(0)

static void homeAndRaiseBed()
{
    homeBed();
    char buffer[32] = {0};
    sprintf_P(buffer, PSTR("G1 F%i Z%i"), int(homing_feedrate[0]), 35);
    enquecommand(buffer);
}

static void homeAndParkHeadForCenterAdjustment()
{
    homeHead();
    char buffer[32] = {0};
    sprintf_P(buffer, PSTR("G1 F%i Z%i X%i Y%i"), int(homing_feedrate[0]), 35, int(AXIS_CENTER_POS(X_AXIS)), int(AXIS_CENTER_POS(Y_AXIS)));
    enquecommand(buffer);
}

static void homeAndParkHeadForCenterAdjustment2()
{
    add_homing[Z_AXIS] = 0;
    enquecommand_P(PSTR("G28 Z0 X0 Y0"));

    char buffer[32] = {0};
    sprintf_P(buffer, PSTR("G1 F%i Z%i X%i Y%i"), int(homing_feedrate[0]), 35, int(AXIS_CENTER_POS(X_AXIS)), int(AXIS_CENTER_POS(Y_AXIS)));
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

//Run the first time you start-up the machine or after a factory reset.
void lcd_menu_first_run_init()
{
    SELECT_MAIN_MENU_ITEM(0);
    lcd_info_screen(lcd_menu_first_run_init_2, NULL, PSTR("CONTINUE"));
    DRAW_PROGRESS_NR_IF_NOT_DONE(1);
    lcd_lib_draw_string_centerP(10, PSTR("Welcome to the first"));
    lcd_lib_draw_string_centerP(20, PSTR("startup of your"));
    lcd_lib_draw_string_centerP(30, PSTR("Ultimaker! Press the"));
    lcd_lib_draw_string_centerP(40, PSTR("button to continue"));
    lcd_lib_update_screen();
}

static void lcd_menu_first_run_init_2()
{
    SELECT_MAIN_MENU_ITEM(0);
    lcd_info_screen(lcd_menu_first_run_init_3, homeAndRaiseBed, PSTR("CONTINUE"));
    DRAW_PROGRESS_NR_IF_NOT_DONE(2);
    lcd_lib_draw_string_centerP(10, PSTR("Because this is the"));
    lcd_lib_draw_string_centerP(20, PSTR("first startup I will"));
    lcd_lib_draw_string_centerP(30, PSTR("walk you through"));
    lcd_lib_draw_string_centerP(40, PSTR("a first run wizard."));
    lcd_lib_update_screen();
}

static void lcd_menu_first_run_init_3()
{
    SELECT_MAIN_MENU_ITEM(0);
    lcd_info_screen(lcd_menu_bed_level_center_adjust, homeAndParkHeadForCenterAdjustment, PSTR("CONTINUE"));
    DRAW_PROGRESS_NR_IF_NOT_DONE(3);
    lcd_lib_draw_string_centerP(10, PSTR("After transportation"));
    lcd_lib_draw_string_centerP(20, PSTR("we need to do some"));
    lcd_lib_draw_string_centerP(30, PSTR("adjustments, we are"));
    lcd_lib_draw_string_centerP(40, PSTR("going to do that now."));
    lcd_lib_update_screen();
}

//Started bed leveling from the calibration menu
void lcd_menu_start_bed_leveling()
{
    lcd_question_screen(lcd_menu_bed_level_center_adjust, homeAndParkHeadForCenterAdjustment2, PSTR("CONTINUE"), NULL, lcd_change_to_previous_menu, PSTR("CANCEL"));
    lcd_lib_draw_string_centerP(10, PSTR("I will guide you"));
    lcd_lib_draw_string_centerP(20, PSTR("through the process"));
    lcd_lib_draw_string_centerP(30, PSTR("of leveling your"));
    lcd_lib_draw_string_centerP(40, PSTR("buildplate."));
    lcd_lib_update_screen();
}

void lcd_menu_start_bed_adjustment()
{
    lcd_question_screen(lcd_menu_bed_level_paper_center, parkHeadForCenterAdjustment, PSTR("CONTINUE"), NULL, lcd_change_to_previous_menu, PSTR("CANCEL"));
    lcd_lib_draw_string_centerP(10, PSTR("I will guide you"));
    lcd_lib_draw_string_centerP(20, PSTR("through the process"));
    lcd_lib_draw_string_centerP(30, PSTR("of fine tuning your"));
    lcd_lib_draw_string_centerP(40, PSTR("buildplate."));
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

    DRAW_PROGRESS_NR_IF_NOT_DONE(4);
    lcd_lib_draw_string_centerP(10, PSTR("Rotate the button"));
    lcd_lib_draw_string_centerP(20, PSTR("until the nozzle is"));
    lcd_lib_draw_string_centerP(30, PSTR("a millimeter away"));
    lcd_lib_draw_string_centerP(40, PSTR("from the buildplate."));
    lcd_lib_update_screen();
}

static void lcd_menu_bed_level_left_back_adjust()
{
    LED_GLOW
    SELECT_MAIN_MENU_ITEM(0);

    lcd_info_screen(lcd_menu_bed_level_right_back_adjust, parkHeadForRightBackAdjustment, PSTR("CONTINUE"));
    DRAW_PROGRESS_NR_IF_NOT_DONE(5);
    lcd_lib_draw_string_centerP(10, PSTR("Turn the buildplate"));
    lcd_lib_draw_string_centerP(20, PSTR("screw till the nozzle"));
    lcd_lib_draw_string_centerP(30, PSTR("is a millimeter away"));
    lcd_lib_draw_string_centerP(40, PSTR("from the buildplate."));

    lcd_lib_update_screen();
}

static void lcd_menu_bed_level_right_back_adjust()
{
    LED_GLOW
    SELECT_MAIN_MENU_ITEM(0);

    lcd_info_screen(lcd_menu_bed_level_right_front_adjust, parkHeadForRightFrontAdjustment, PSTR("CONTINUE"));
    DRAW_PROGRESS_NR_IF_NOT_DONE(6);
    lcd_lib_draw_string_centerP(10, PSTR("Turn the buildplate"));
    lcd_lib_draw_string_centerP(20, PSTR("screw till the nozzle"));
    lcd_lib_draw_string_centerP(30, PSTR("is a millimeter away"));
    lcd_lib_draw_string_centerP(40, PSTR("from the buildplate."));

    lcd_lib_update_screen();
}

static void lcd_menu_bed_level_right_front_adjust()
{
    LED_GLOW
    SELECT_MAIN_MENU_ITEM(0);

    lcd_info_screen(lcd_menu_bed_level_left_front_adjust, parkHeadForLeftFrontAdjustment, PSTR("CONTINUE"));
    DRAW_PROGRESS_NR_IF_NOT_DONE(7);
    lcd_lib_draw_string_centerP(10, PSTR("Turn the buildplate"));
    lcd_lib_draw_string_centerP(20, PSTR("screw till the nozzle"));
    lcd_lib_draw_string_centerP(30, PSTR("is a millimeter away"));
    lcd_lib_draw_string_centerP(40, PSTR("from the buildplate."));

    lcd_lib_update_screen();
}

static void lcd_menu_bed_level_left_front_adjust()
{
    LED_GLOW
    SELECT_MAIN_MENU_ITEM(0);

    lcd_info_screen(lcd_menu_bed_level_paper, NULL, PSTR("CONTINUE"));
    DRAW_PROGRESS_NR_IF_NOT_DONE(8);
    lcd_lib_draw_string_centerP(10, PSTR("Turn the buildplate"));
    lcd_lib_draw_string_centerP(20, PSTR("screw till the nozzle"));
    lcd_lib_draw_string_centerP(30, PSTR("is a millimeter away"));
    lcd_lib_draw_string_centerP(40, PSTR("from the buildplate."));

    lcd_lib_update_screen();
}

static void lcd_menu_bed_level_paper()
{
    SELECT_MAIN_MENU_ITEM(0);

    lcd_info_screen(lcd_menu_bed_level_paper_center, parkHeadForCenterAdjustment, PSTR("CONTINUE"));
    DRAW_PROGRESS_NR_IF_NOT_DONE(9);
    lcd_lib_draw_string_centerP(10, PSTR("Repeat this step, but"));
    lcd_lib_draw_string_centerP(20, PSTR("now use a sheet of"));
    lcd_lib_draw_string_centerP(30, PSTR("paper to fine-tune"));
    lcd_lib_draw_string_centerP(40, PSTR("the buildplate level."));
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

    DRAW_PROGRESS_NR_IF_NOT_DONE(10);
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
    DRAW_PROGRESS_NR_IF_NOT_DONE(11);
    lcd_lib_draw_string_centerP(20, PSTR("Repeat this for the"));
    lcd_lib_draw_string_centerP(30, PSTR("back left corner..."));
    lcd_lib_update_screen();
}

static void lcd_menu_bed_level_paper_right_back()
{
    LED_GLOW
    SELECT_MAIN_MENU_ITEM(0);

    lcd_info_screen(lcd_menu_bed_level_paper_right_front, parkHeadForRightFrontAdjustment, PSTR("CONTINUE"));
    DRAW_PROGRESS_NR_IF_NOT_DONE(12);
    lcd_lib_draw_string_centerP(20, PSTR("Repeat this for the"));
    lcd_lib_draw_string_centerP(30, PSTR("back right corner..."));
    lcd_lib_update_screen();
}

static void lcd_menu_bed_level_paper_right_front()
{
    LED_GLOW
    SELECT_MAIN_MENU_ITEM(0);

    lcd_info_screen(lcd_menu_bed_level_paper_left_front, parkHeadForLeftFrontAdjustment, PSTR("CONTINUE"));
    DRAW_PROGRESS_NR_IF_NOT_DONE(13);
    lcd_lib_draw_string_centerP(20, PSTR("Repeat this for the"));
    lcd_lib_draw_string_centerP(30, PSTR("front right corner..."));
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

    if (IS_FIRST_RUN_DONE())
        lcd_info_screen(lcd_menu_bed_level_done, storeBedLeveling, PSTR("DONE"));
    else
        lcd_info_screen(lcd_menu_first_run_done, storeBedLeveling, PSTR("CONTINUE"));

    DRAW_PROGRESS_NR_IF_NOT_DONE(14);
    lcd_lib_draw_string_centerP(20, PSTR("Repeat this for the"));
    lcd_lib_draw_string_centerP(30, PSTR("front left corner..."));
    lcd_lib_update_screen();
}

static void lcd_menu_bed_level_done()
{
    menu.return_to_previous(true);
}

static void lcd_menu_first_run_done()
{
    SET_FIRST_RUN_DONE();

    SELECT_MAIN_MENU_ITEM(0);
    lcd_info_screen(NULL, lcd_return_to_main_menu);
    DRAW_PROGRESS_NR(15);
    lcd_lib_draw_string_centerP(10, PSTR("Your Ultimaker2 is"));
    lcd_lib_draw_string_centerP(20, PSTR("now calibrated."));
    lcd_lib_draw_string_centerP(30, PSTR("Enjoy printing"));
    lcd_lib_draw_string_centerP(40, PSTR("with it!"));
    lcd_lib_update_screen();
}

#endif//ENABLE_ULTILCD2
