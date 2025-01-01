#ifndef EEZ_LVGL_UI_SCREENS_H
#define EEZ_LVGL_UI_SCREENS_H

#include <lvgl.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct _objects_t {
    lv_obj_t *main;
    lv_obj_t *speed;
    lv_obj_t *calendar;
    lv_obj_t *obj0;
    lv_obj_t *chart1;
    lv_obj_t *arc1;
    lv_obj_t *val1;
    lv_obj_t *meter1;
    lv_obj_t *text2;
} objects_t;

extern objects_t objects;

enum ScreensEnum {
    SCREEN_ID_MAIN = 1,
    SCREEN_ID_SPEED = 2,
    SCREEN_ID_CALENDAR = 3,
};

void create_screen_main();
void tick_screen_main();

void create_screen_speed();
void tick_screen_speed();

void create_screen_calendar();
void tick_screen_calendar();

void create_screens();
void tick_screen(int screen_index);


#ifdef __cplusplus
}
#endif

#endif /*EEZ_LVGL_UI_SCREENS_H*/