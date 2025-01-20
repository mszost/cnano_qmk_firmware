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
    LAYER_BASE,
    LAYER_NAV,
    LAYER_NUM,
    LAYER_SYM,
    LAYER_FN,
    LAYER_PTR,
};


#define SPC_NAV LT(LAYER_NAV, KC_SPC)
#define BCK_NAV LT(LAYER_NAV, KC_BSPC)
#define RET_NUM LT(LAYER_NUM, KC_ENT)
#define ESC_SYM LT(LAYER_SYM, KC_ESC)
#define TAB_FN LT(LAYER_FN, KC_TAB)
#define SLSH_PTR LT(LAYER_PTR, KC_SLASH)
#define _L_PTR(KC) LT(LAYER_PTR, KC)


// Convenience row shorthands. 
#define _______________DEAD_HALF_ROW_______________ XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX
#define ___________TRANSPARENT_HALF_ROW____________ _______, _______, _______, _______, _______
#define ______________HOME_ROW_GACS_R______________ XXXXXXX, KC_LSFT, KC_LCTL, KC_LALT, KC_LGUI


// Colemak-DH with GACS home-row mods
#define LAYOUT_LAYER_BASE \
    KC_Q,         KC_W,         KC_F,         KC_P,         KC_B,        KC_J,  KC_L,         KC_U,         KC_Y,         KC_MINS,      \
    LGUI_T(KC_A), LALT_T(KC_R), LCTL_T(KC_S), LSFT_T(KC_T), KC_G,        KC_M,  RSFT_T(KC_N), RCTL_T(KC_E), RALT_T(KC_I), RGUI_T(KC_O), \
    KC_Z,         KC_X,         KC_C,         KC_D,         KC_V,        KC_K,  KC_H,         KC_COMMA,     KC_DOT,       SLSH_PTR,     \
                             ESC_SYM,      SPC_NAV,         TAB_FN,      RET_NUM,  BCK_NAV


#define LAYOUT_LAYER_NAV \
    ___________TRANSPARENT_HALF_ROW____________,    KC_NO,  KC_HOME,  KC_UP,   KC_END,  KC_PGUP, \
     KC_LGUI, KC_LALT, KC_LCTL, KC_LSFT, KC_NO,     KC_NO,  KC_LEFT,  KC_DOWN, KC_RGHT, KC_PGDN, \
    ___________TRANSPARENT_HALF_ROW____________,    KC_LCAP,  KC_NO,  KC_NO,   KC_NO,   KC_NO,   \
                      _______, _______, _______,    _______, _______


#define LAYOUT_LAYER_NUM \
    KC_NO,  KC_7,  KC_9,  KC_9,  KC_NO,    KC_NO,   KC_NO,   KC_VOLU,  KC_NO,   KC_NO,    \
    KC_NO,  KC_4,  KC_5,  KC_6,  KC_NO,    KC_MPLY, KC_MPRV, KC_VOLD,  KC_MNXT, KC_NO,    \
    KC_NO,  KC_1,  KC_2,  KC_3,  KC_NO,    C(KC_0), C(KC_MINS), C(KC_PLUS), KC_NO, KC_NO, \
             _______, _______, _______,    _______, _______


#define LAYOUT_LAYER_SYM \
    KC_TILD, KC_AMPR, KC_ASTR,  KC_CIRC, KC_PIPE,    KC_NO,   KC_LCBR, KC_RCBR, KC_NO,   KC_NO,   \
    KC_GRV,  KC_DLR,  KC_PERC,  KC_EQL,  KC_PLUS,    KC_COLN, KC_LPRN, KC_RPRN, KC_QUOT, KC_DQUO, \
    KC_NO,   KC_EXLM, KC_AT,    KC_HASH, KC_BSLS,    KC_SCLN, KC_LBRC, KC_RBRC, KC_NO,   KC_NO,   \
                       _______, _______, _______,    _______, _______


#define LAYOUT_LAYER_FN \
    KC_F15,  KC_F7,  KC_F8,  KC_F9,  KC_F12,    _______________DEAD_HALF_ROW_______________, \
    KC_F14,  KC_F4,  KC_F5,  KC_F6,  KC_F11,    _______________DEAD_HALF_ROW_______________, \
    KC_F13,  KC_F1,  KC_F2,  KC_F3,  KC_F10,    _______________DEAD_HALF_ROW_______________, \
                  _______, _______, _______,    _______, _______


#define LAYOUT_LAYER_PTR \
    _______________DEAD_HALF_ROW_______________,    _______________DEAD_HALF_ROW_______________, \
    _______________DEAD_HALF_ROW_______________,    _______________DEAD_HALF_ROW_______________, \
    _______________DEAD_HALF_ROW_______________,    _______________DEAD_HALF_ROW_______________, \
             _______, _______, _______,    _______, _______



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
  [LAYER_BASE] = LAYOUT_wrapper(POINTER_MOD(LAYOUT_LAYER_BASE)),
  [LAYER_NAV] = LAYOUT_wrapper(LAYOUT_LAYER_NAV),
  [LAYER_NUM] = LAYOUT_wrapper(LAYOUT_LAYER_NUM),
  [LAYER_SYM] = LAYOUT_wrapper(LAYOUT_LAYER_SYM),
  [LAYER_FN] = LAYOUT_wrapper(LAYOUT_LAYER_FN),
  [LAYER_PTR] = LAYOUT_wrapper(LAYOUT_LAYER_PTR),
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
