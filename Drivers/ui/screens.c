#include <string.h>

#include "screens.h"
#include "images.h"
#include "fonts.h"
#include "actions.h"
#include "vars.h"
#include "styles.h"
#include "ui.h"

#include <string.h>

objects_t objects;
lv_obj_t *tick_value_change_obj;

static lv_meter_scale_t * scale0;
 lv_meter_indicator_t * indicator1;

void create_screen_main() {
    lv_obj_t *obj = lv_obj_create(0);
    objects.main = obj;
    lv_obj_set_pos(obj, 0, 0);
    lv_obj_set_size(obj, 320, 240);
    {
        lv_obj_t *parent_obj = obj;
        {
            lv_obj_t *obj = lv_label_create(parent_obj);
            lv_obj_set_pos(obj, 206, 3);
            lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
            lv_label_set_text(obj, "Hello, world!");
        }
        {
            lv_obj_t *obj = lv_btn_create(parent_obj);
            objects.obj0 = obj;
            lv_obj_set_pos(obj, 208, 28);
            lv_obj_set_size(obj, 100, 50);
            lv_obj_set_style_bg_color(obj, lv_color_hex(0xff2196f3), LV_PART_MAIN | LV_STATE_DEFAULT);
            {
                lv_obj_t *parent_obj = obj;
                {
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    lv_obj_set_pos(obj, 0, 0);
                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                    lv_label_set_text(obj, "Button");
                    lv_obj_set_style_align(obj, LV_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);
                }
            }
        }
        {
            lv_obj_t *obj = lv_checkbox_create(parent_obj);
            lv_obj_set_pos(obj, 206, 91);
            lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
            lv_checkbox_set_text(obj, "Checkbox");
        }
        {
            lv_obj_t *obj = lv_roller_create(parent_obj);
            lv_obj_set_pos(obj, 218, 120);
            lv_obj_set_size(obj, 80, 100);
            lv_roller_set_options(obj, "Option 1\nOption 2\nOption 3", LV_ROLLER_MODE_NORMAL);
        }
        {
            // chart1
            lv_obj_t *obj = lv_chart_create(parent_obj);
            objects.chart1 = obj;
            lv_obj_set_pos(obj, 18, 120);
            lv_obj_set_size(obj, 180, 100);
            lv_obj_set_style_border_color(obj, lv_color_hex(0xff3361ae), LV_PART_MAIN | LV_STATE_DEFAULT);
        }
        {
            // arc1
            lv_obj_t *obj = lv_arc_create(parent_obj);
            objects.arc1 = obj;
            lv_obj_set_pos(obj, 52, 12);
            lv_obj_set_size(obj, 112, 101);
            lv_arc_set_value(obj, 25);
            lv_arc_set_bg_end_angle(obj, 60);
        }
        {
            // val1
            lv_obj_t *obj = lv_label_create(parent_obj);
            objects.val1 = obj;
            lv_obj_set_pos(obj, 90, 54);
            lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
            lv_label_set_text(obj, "000");
        }
    }
}

void tick_screen_main() {
}

void create_screen_speed() {
    lv_obj_t *obj = lv_obj_create(0);
    objects.speed = obj;
    lv_obj_set_pos(obj, 0, 0);
    lv_obj_set_size(obj, 320, 240);
    {
        lv_obj_t *parent_obj = obj;
        {
            // meter1
            lv_obj_t *obj = lv_meter_create(parent_obj);
            objects.meter1 = obj;
            lv_obj_set_pos(obj, 68, 37);
            lv_obj_set_size(obj, 180, 180);
            {
                lv_meter_scale_t *scale = lv_meter_add_scale(obj);
                scale0 = scale;
                lv_meter_set_scale_ticks(obj, scale, 41, 1, 5, lv_color_hex(0xffa49a9a));
                lv_meter_set_scale_major_ticks(obj, scale, 8, 3, 5, lv_color_hex(0xffcf4444), 16);
                lv_meter_set_scale_range(obj, scale, 0, 300, 270, 90);
                {
                    lv_meter_indicator_t *indicator = lv_meter_add_needle_line(obj, scale, 3.5, lv_color_hex(0xfffe1313), -11);
                    indicator1 = indicator;
                    lv_meter_set_indicator_value(obj, indicator, 245);
                }
            }
            lv_obj_set_style_bg_color(obj, lv_color_hex(0xff000000), LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_obj_set_style_bg_color(obj, lv_color_hex(0xff4a4343), LV_PART_INDICATOR);
        }
        {
            // text2
            lv_obj_t *obj = lv_label_create(parent_obj);
            objects.text2 = obj;
            lv_obj_set_pos(obj, 88, 10);
            lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
            lv_label_set_long_mode(obj, LV_LABEL_LONG_SCROLL_CIRCULAR);
            lv_label_set_text(obj, "     Speedtest");
            //lv_obj_set_style_text_font(obj, &lv_font_montserrat_30, LV_PART_MAIN | LV_STATE_DEFAULT);
        }
    }
}

void tick_screen_speed() {
}

void create_screen_calendar() {
    lv_obj_t *obj = lv_obj_create(0);
    objects.calendar = obj;
    lv_obj_set_pos(obj, 0, 0);
    lv_obj_set_size(obj, 320, 240);
    {
        lv_obj_t *parent_obj = obj;
        {
            lv_obj_t *obj = lv_calendar_create(parent_obj);
            lv_obj_set_pos(obj, 45, 11);
            lv_obj_set_size(obj, 230, 202);
            lv_calendar_header_arrow_create(obj);
            lv_calendar_set_today_date(obj, 2022, 11, 1);
            lv_calendar_set_showed_date(obj, 2022, 11);
        }
    }
}

void tick_screen_calendar() {
}


void create_screens() {
    lv_disp_t *dispp = lv_disp_get_default();
    lv_theme_t *theme = lv_theme_default_init(dispp, lv_palette_main(LV_PALETTE_BLUE), lv_palette_main(LV_PALETTE_RED), true, LV_FONT_DEFAULT);
    lv_disp_set_theme(dispp, theme);
    
    create_screen_main();
    create_screen_speed();
    create_screen_calendar();
}

typedef void (*tick_screen_func_t)();

tick_screen_func_t tick_screen_funcs[] = {
    tick_screen_main,
    tick_screen_speed,
    tick_screen_calendar,
};

void tick_screen(int screen_index) {
    tick_screen_funcs[screen_index]();
}
