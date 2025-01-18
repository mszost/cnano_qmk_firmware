/**
 * Copyright 2021 Charly Delay <charly@codesink.dev> (@0xcharly)
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include QMK_KEYBOARD_H


#ifdef CHARYBDIS_AUTO_POINTER_LAYER_TRIGGER_ENABLE
#    include "timer.h"
#endif // CHARYBDIS_AUTO_POINTER_LAYER_TRIGGER_ENABLE

// Automatically enable sniping-mode on the pointer layer.
#define CHARYBDIS_AUTO_SNIPING_ON_LAYER LAYER_PTR

#ifdef CHARYBDIS_AUTO_POINTER_LAYER_TRIGGER_ENABLE
static uint16_t auto_pointer_layer_timer = 0;

#    ifndef CHARYBDIS_AUTO_POINTER_LAYER_TRIGGER_TIMEOUT_MS
#        define CHARYBDIS_AUTO_POINTER_LAYER_TRIGGER_TIMEOUT_MS 1000
#    endif 

#    ifndef CHARYBDIS_AUTO_POINTER_LAYER_TRIGGER_THRESHOLD
#        define CHARYBDIS_AUTO_POINTER_LAYER_TRIGGER_THRESHOLD 8
#    endif 
#endif     

#ifndef POINTING_DEVICE_ENABLE
#    define DRGSCRL KC_NO
#    define DPI_MOD KC_NO
#    define S_D_MOD KC_NO
#    define SNIPING KC_NO
#endif // !POINTING_DEVICE_ENABLE


#ifdef MACCEL_ENABLE
    #include "maccel/maccel.h"
#endif


enum charybdis_keymap_layers {
    LAYER_BASE = 0,
    LAYER_NAV,
    LAYER_NUM,
    LAYER_SYM,
    LAYER_FN,
    LAYER_PTR,
};


#define SPC_NAV LT(LAYER_NAV, KC_SPC)
#define TAB_FN LT(LAYER_FN, KC_TAB)
#define ENT_NUM LT(LAYER_NUM, KC_ENT)
#define BCK_SYM LT(LAYER_SYM, KC_BSPC)
#define SLASH_PTR LT(LAYER_PTR, KC_SLASH)
#define _L_PTR(KC) LT(LAYER_PTR, KC)


/** Convenience row shorthands. 
#define _______________DEAD_HALF_ROW_______________ XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX
#define ______________HOME_ROW_GACS_L______________ KC_LGUI, KC_LALT, KC_LCTL, KC_LSFT, XXXXXXX
#define ______________HOME_ROW_GACS_R______________ XXXXXXX, KC_LSFT, KC_LCTL, KC_LALT, KC_LGUI
*/

// Base layer: Colemak-DH 3x10 Matrix
#define LAYOUT_LAYER_BASE                                                                             \
  /*--------+--------+--------+--------+--------+    +--------|--------|--------|--------|--------|*/ \
      KC_Q  ,  KC_W  ,  KC_F  ,  KC_P  ,  KC_B  ,       KC_J  ,  KC_L  ,  KC_U  ,  KC_Y  , KC_MINS,   \
  /*--------|--------|--------|--------|--------|    |--------|--------|--------|--------|--------|*/ \
      KC_A  ,  KC_R  ,  KC_S   ,  KC_T ,  KC_G  ,       KC_M  ,  KC_N  ,  KC_E  ,  KC_I  ,  KC_O  ,   \
  /*--------|--------|--------|--------|--------|    |--------|--------|--------|--------|--------|*/ \
      KC_Z  ,  KC_X  ,  KC_C  ,  KC_D  ,  KC_V  ,       KC_K  ,  KC_H  ,KC_COMMA, KC_DOT , KC_SLASH,  \
  /*--------+--------|--------|--------|--------|    |--------|--------|--------|--------|--------|*/ \
                LGUI_T(KC_ESC),   SPC_NAV, TAB_FN,     KC_RETURN, KC_BSPC  
  /*                 +--------|--------|--------+    +--------|--------+                           */



#define LAYOUT_LAYER_PTR                                                                              \
  /*--------+--------+--------+--------+--------+    +--------|--------|--------|--------|--------|*/ \
    XXXXXXX , XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX,     XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX,  XXXXXXX,   \
  /*--------|--------|--------|--------|--------|    |--------|--------|--------|--------|--------|*/ \
    XXXXXXX , XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX,     XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX,    \
  /*--------|--------|--------|--------|--------|    |--------|--------|--------|--------|--------|*/ \
    XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX ,     XXXXXXX , XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX,   \
  /*--------+--------|--------|--------|--------|    |--------|--------|--------|--------|--------|*/ \
                      XXXXXXX, XXXXXXX,  XXXXXXX,      XXXXXXX, XXXXXXX
  /*                 +--------|--------|--------+    +--------|--------+                           */

/*
 * Add Home Row mod to a layout.
 *
 * Expects a 10-key per row layout.  Adds support for GACS (Gui, Alt, Ctl, Shift)
 * home row.  The layout passed in parameter must contain at least 20 keycodes.
 *
 * This is meant to be used with `LAYER_ALPHAS_QWERTY` defined above, eg.:
 *
 *     HOME_ROW_MOD_GACS(LAYER_ALPHAS_QWERTY)

#define _HOME_ROW_MOD_GACS(                                            \
    L00, L01, L02, L03, L04, R05, R06, R07, R08, R09,                  \
    L10, L11, L12, L13, L14, R15, R16, R17, R18, R19,                  \
    ...)                                                               \
             L00,         L01,         L02,         L03,         L04,  \
             R05,         R06,         R07,         R08,         R09,  \
      LGUI_T(L10), LALT_T(L11), LCTL_T(L12), LSFT_T(L13),        L14,  \
             R15,  RSFT_T(R16), RCTL_T(R17), LALT_T(R18), RGUI_T(R19), \
      __VA_ARGS__
#define HOME_ROW_MOD_GACS(...) _HOME_ROW_MOD_GACS(__VA_ARGS__)
*/

/*
 * Add pointer layer keys to a layout.
 *
 * Expects a 10-key per row layout.  The layout passed in parameter must contain
 * at least 30 keycodes.
 *
 * This is meant to be used with `LAYER_ALPHAS_QWERTY` defined above, eg.:
 *  POINTER_MOD(LAYER_ALPHAS_QWERTY)
 */
#define _POINTER_MOD(                                                  \
    L00, L01, L02, L03, L04, R05, R06, R07, R08, R09,                  \
    L10, L11, L12, L13, L14, R15, R16, R17, R18, R19,                  \
    L20, L21, L22, L23, L24, R25, R26, R27, R28, R29,                  \
    ...)                                                               \
             L00,         L01,         L02,         L03,         L04,  \
             R05,         R06,         R07,         R08,         R09,  \
             L10,         L11,         L12,         L13,         L14,  \
             R15,         R16,         R17,         R18,         R19,  \
      _L_PTR(L20),        L21,         L22,         L23,         L24,  \
             R25,         R26,         R27,         R28,  _L_PTR(R29), \
      __VA_ARGS__

#define POINTER_MOD(...) _POINTER_MOD(__VA_ARGS__)
#define LAYOUT_wrapper(...) LAYOUT(__VA_ARGS__)


const uint16_t PROGMEM keymaps[][MATRIX_ROWS][MATRIX_COLS] = {
  [LAYER_BASE] = LAYOUT_wrapper(
    //POINTER_MOD(HOME_ROW_MOD_GACS(LAYOUT_LAYER_BASE))
    POINTER_MOD(LAYOUT_LAYER_BASE)
  )//,
  //[LAYER_POINTER] = LAYOUT_wrapper(LAYOUT_LAYER_POINTER),
};


#ifdef POINTING_DEVICE_ENABLE
#    ifdef CHARYBDIS_AUTO_POINTER_LAYER_TRIGGER_ENABLE
report_mouse_t pointing_device_task_user(report_mouse_t mouse_report) {
    if (abs(mouse_report.x) > CHARYBDIS_AUTO_POINTER_LAYER_TRIGGER_THRESHOLD || abs(mouse_report.y) > CHARYBDIS_AUTO_POINTER_LAYER_TRIGGER_THRESHOLD) {
        if (auto_pointer_layer_timer == 0) {
            layer_on(LAYER_POINTER);
#        ifdef RGB_MATRIX_ENABLE
            rgb_matrix_mode_noeeprom(RGB_MATRIX_NONE);
            rgb_matrix_sethsv_noeeprom(HSV_GREEN);
#        endif // RGB_MATRIX_ENABLE
        }
        auto_pointer_layer_timer = timer_read();
    }

    #ifdef MACCEL_ENABLE
        return pointing_device_task_maccel(mouse_report);
    #endif

    return mouse_report;
}

void matrix_scan_user(void) {
    if (auto_pointer_layer_timer != 0 && TIMER_DIFF_16(timer_read(), auto_pointer_layer_timer) >= CHARYBDIS_AUTO_POINTER_LAYER_TRIGGER_TIMEOUT_MS) {
        auto_pointer_layer_timer = 0;
        layer_off(LAYER_PTR);
#        ifdef RGB_MATRIX_ENABLE
        rgb_matrix_mode_noeeprom(RGB_MATRIX_DEFAULT_MODE);
#        endif // RGB_MATRIX_ENABLE
    }
}
#    endif // CHARYBDIS_AUTO_POINTER_LAYER_TRIGGER_ENABLE

#    ifdef CHARYBDIS_AUTO_SNIPING_ON_LAYER
layer_state_t layer_state_set_user(layer_state_t state) {
    charybdis_set_pointer_sniping_enabled(layer_state_cmp(state, CHARYBDIS_AUTO_SNIPING_ON_LAYER));
    return state;
}
#    endif // CHARYBDIS_AUTO_SNIPING_ON_LAYER
#endif     // POINTING_DEVICE_ENABLE

#ifdef RGB_MATRIX_ENABLE
// Forward-declare this helper function since it is defined in
// rgb_matrix.c.
void rgb_matrix_update_pwm_buffers(void);
#endif
