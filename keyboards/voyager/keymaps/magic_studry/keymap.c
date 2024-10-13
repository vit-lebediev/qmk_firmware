#include QMK_KEYBOARD_H
#include "version.h"
//#include "stdio.h"

#define MOON_LED_LEVEL LED_LEVEL
#define ML_SAFE_RANGE SAFE_RANGE

// Taken from magic sturdy repo
// @see https://github.com/Ikcelaks/keyboard_layouts/blob/main/magic_sturdy/QMK_Layout/Moonlander/magic_sturdy/keymap.c#L324
#define KC_MAC_UNDO LGUI(KC_Z)
#define KC_MAC_CUT LGUI(KC_X)
#define KC_MAC_COPY LGUI(KC_C)
#define KC_MAC_PASTE LGUI(KC_V)
#define KC_PC_UNDO LCTL(KC_Z)
#define KC_PC_CUT LCTL(KC_X)
#define KC_PC_COPY LCTL(KC_C)
#define KC_PC_PASTE LCTL(KC_V)
#define ES_LESS_MAC KC_GRV
#define ES_GRTR_MAC LSFT(KC_GRV)
#define ES_BSLS_MAC ALGR(KC_6)
#define NO_PIPE_ALT KC_GRV
#define NO_BSLS_ALT KC_EQUAL
#define LSA_T(kc) MT(MOD_LSFT | MOD_LALT, kc)
#define BP_NDSH_MAC ALGR(KC_8)
#define SE_SECT_MAC ALGR(KC_6)
#define MOON_LED_LEVEL LED_LEVEL

#define C_MAGIC QK_AREP
#define C_GUI_ESC LGUI_T(KC_ESC)
#define C_RSFT_ENT RSFT_T(KC_ENT)
#define C_LALT_ENT LALT_T(KC_ENT)
#define C_RCTL_MINS RCTL_T(KC_MINS)
#define C_LCTL_BSPC LCTL(KC_BSPC)

enum layers {
    BASE,
    NUMS,
    ARRWS,
};

enum custom_keycodes {
    RGB_SLD = ML_SAFE_RANGE,
    HSV_0_245_245,
    HSV_74_255_255,
    HSV_188_255_255,
    C_MAG_2 = SAFE_RANGE,
    C_MAG_3,
    MK_DUND,
    MG_ENT,
    MG_MENT,
    MG_ER,
    MG_ES,
    MG_UST,
    MG_ON,
    MG_ION,
    MG_OA,
    MG_SP_BUT,
    MG_THE,
    MG_EFORE,
    MG_HICH,
    MG_MLATIV,
    MG_QUOT_S,
};

enum tap_dance_codes {
  DANCE_0,
  DANCE_1,
};

const uint16_t PROGMEM keymaps[][MATRIX_ROWS][MATRIX_COLS] = {
  [BASE] = LAYOUT_voyager(
    TD(DANCE_0),        KC_F1,              KC_F2,          KC_F3,           KC_F4,          KC_F5,                                          KC_F6,          KC_F7,          KC_F8,          KC_F9,          KC_F10,                 KC_MINUS,
    KC_V,               KC_M,               KC_L,           KC_C,            KC_P,           KC_SCLN,                                        KC_B,           C_MAGIC,        KC_U,           KC_O,           KC_TRANSPARENT,         KC_BSLS,
    MT(MOD_LSFT, KC_S), KC_T,               KC_R,           KC_D,            KC_Y,           KC_Q,                                           KC_F,           KC_N,           KC_E,           KC_A,           KC_I,                   MT(MOD_RSFT, KC_QUOTE),
    MT(MOD_LCTL, KC_X), MT(MOD_LALT, KC_K), KC_J,           KC_G,            KC_W,           KC_TAB,                                         KC_Z,           KC_H,           KC_COMMA,       KC_DOT,         MT(MOD_RALT, KC_SLASH), MT(MOD_RCTL, KC_CAPS),
                                                              HYPR_T(KC_ENTER),  MT(MOD_LGUI, KC_BSPC),                          QK_REP,   LT(ARRWS,KC_SPC)
  ),
  [NUMS] = LAYOUT_voyager(
    KC_ESCAPE,        KC_F1,          KC_F2,          KC_F3,          KC_F4,          KC_F5,                                          KC_F6,          KC_F7,          KC_F8,          KC_F9,          KC_F10,         KC_F11,
    KC_GRAVE,         KC_EXLM,        KC_AT,          KC_HASH,        KC_DLR,         KC_PERC,                                        KC_MINUS,       KC_7,           KC_8,           KC_9,           KC_SLASH,       KC_F12,
    TD(DANCE_1),      KC_CIRC,        KC_AMPR,        KC_ASTR,        KC_LPRN,        KC_RPRN,                                        KC_PLUS,        KC_4,           KC_5,           KC_6,           KC_ASTR,        MT(MOD_RSFT, KC_BSPC),
    KC_TRANSPARENT,   KC_TRANSPARENT, KC_RBRC,        KC_LBRC,        KC_LCBR,        KC_RCBR,                                        KC_0,           KC_1,           KC_2,           KC_3,           KC_EQUAL,       KC_ENTER,
                                                              KC_TRANSPARENT,   KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_DOT
  ),
  [ARRWS] = LAYOUT_voyager(
    RGB_TOG,        TOGGLE_LAYER_COLOR,  RGB_MODE_FORWARD,    RGB_SLD,         RGB_VAD,             RGB_VAI,                                        KC_TRANSPARENT, KC_TRANSPARENT,     KC_TRANSPARENT,  KC_TRANSPARENT, KC_TRANSPARENT, QK_BOOT,
    KC_TRANSPARENT, KC_TRANSPARENT,      KC_AUDIO_VOL_DOWN,   KC_AUDIO_VOL_UP, KC_AUDIO_MUTE,       KC_TRANSPARENT,                                 LGUI(KC_LEFT),  LALT(KC_LEFT),      KC_UP,           LALT(KC_RIGHT), LGUI(KC_RIGHT), KC_TRANSPARENT,
    KC_TRANSPARENT, KC_MEDIA_PREV_TRACK, KC_MEDIA_NEXT_TRACK, KC_MEDIA_STOP,   KC_MEDIA_PLAY_PAUSE, KC_TRANSPARENT,                                 KC_PAGE_UP,     KC_LEFT,            KC_DOWN,         KC_RIGHT,       KC_TRANSPARENT, KC_TRANSPARENT,
    HSV_0_245_245,  HSV_74_255_255,      HSV_188_255_255,     KC_TRANSPARENT,  KC_TRANSPARENT,      KC_TRANSPARENT,                                 KC_PGDN,        LCTL(LSFT(KC_TAB)), LCTL(KC_TAB),    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,
                                                                                 KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_TRANSPARENT
  ),
};

// Custom configuration for magic sturdy setup: copied from magic sturdy repo
const uint16_t PROGMEM combo_LTEST_12[] = { KC_1, KC_2, COMBO_END};
const uint16_t PROGMEM combo_RT_12[] = { QK_REP, OSM(MOD_LSFT), COMBO_END};
const uint16_t PROGMEM combo_LB_IM[] = { KC_J, KC_G, COMBO_END};
const uint16_t PROGMEM combo_LB_MR[] = { KC_R, KC_D, COMBO_END};
const uint16_t PROGMEM combo_LB_RP[] = { KC_E, KC_A, COMBO_END};
const uint16_t PROGMEM combo_LB_IR[] = { KC_K, KC_G, COMBO_END};
const uint16_t PROGMEM combo_RB_IM[] = { KC_H, KC_QUOT, COMBO_END};
const uint16_t PROGMEM combo_RB_MR[] = { KC_QUOT, KC_QUES, COMBO_END};
const uint16_t PROGMEM combo_RB_RP[] = { KC_QUES, KC_DOT, COMBO_END};
const uint16_t PROGMEM combo_RB_IR[] = { KC_H, KC_QUES, COMBO_END};

combo_t key_combos[COMBO_COUNT] = {
//    COMBO(combo_LTEST_12, OSL(SYMB)),
    COMBO(combo_RT_12, KC_E),
    COMBO(combo_LB_IM, KC_COLN),
    COMBO(combo_LB_MR, C_MAG_2),
    COMBO(combo_LB_RP, C_MAG_3),
    COMBO(combo_LB_IR, MG_QUOT_S),
    COMBO(combo_RB_IM, KC_SCLN),
    COMBO(combo_RB_MR, C_MAG_2),
    COMBO(combo_RB_RP, C_MAG_3),
    COMBO(combo_RB_IR, MG_QUOT_S),
};

extern rgb_config_t rgb_matrix_config;

void keyboard_post_init_user(void) {
  rgb_matrix_enable();
}

bool remember_last_key_user(uint16_t keycode, keyrecord_t* record, uint8_t* remembered_mods) {
    switch (keycode) {
        // Ignore Custom Magic Keys
        case C_MAG_2:
        case C_MAG_3:
            return false;
        case KC_A ... KC_Z:
            if ((*remembered_mods & ~(MOD_MASK_SHIFT)) == 0) {
                *remembered_mods &= ~MOD_MASK_SHIFT;
            }
            break;
    }

    return true;
}

uint16_t get_alt_repeat_key_keycode_user(uint16_t keycode, uint8_t mods) {
    // Debugging
//    char buffer[128];
//    sprintf(buffer, "Keycode: %d\n", keycode);
//    SEND_STRING(buffer);
//
//    sprintf(buffer, "KC_X: %d, KC_K: %d, KC_S: %d\n", KC_X, KC_K, KC_S);
//    SEND_STRING(buffer);
//
//    sprintf(buffer, "MT(MOD_LSFT, KC_S): %d, MT(MOD_LCTL, KC_X): %d, MT(MOD_LALT, KC_K): %d\n", MT(MOD_LSFT, KC_S), MT(MOD_LCTL, KC_X), MT(MOD_LALT, KC_K));
//    SEND_STRING(buffer);
    switch (keycode) {
        case KC_C:
        case KC_P:
        case KC_D: // should be ignored - it's a roll
        case KC_G:
        case KC_Z: return KC_Y; // should be ignored - diff parts of keyboard, z->y is easier than z->*
        case KC_Y: return KC_P;
        case KC_R: return KC_L;
        case KC_K:
        case MT(MOD_LALT, KC_K): return KC_S;
        case KC_L:
        case KC_S:
        case MT(MOD_LSFT, KC_S): return KC_K;
        case KC_U: return KC_E;
        case KC_E: return KC_U;
        case KC_O: return KC_A;
        case KC_A: return KC_O;
        case KC_DOT:
            if (mods & MOD_MASK_SHIFT) {
                return KC_EQL;
            } else {
                return KC_BSLS;
            }
        case KC_COMM:
            if (mods & MOD_MASK_SHIFT) {
                return KC_EQL;
            } else {
                return MG_SP_BUT;
            }
        case KC_EQL:
        case C_RCTL_MINS:
        case KC_MINS: return KC_RABK;
        case KC_Q: return MG_MLATIV;
        case KC_H: return MG_OA;
        case KC_I: return MG_ON;
        case KC_N: return MG_ION;
        case KC_V: return MG_ER;
        case KC_X:
        case MT(MOD_LCTL, KC_X): return MG_ES;
        case KC_M: return MG_ENT;
        case KC_T: return MG_MENT;
        case KC_J: return MG_UST;
        case KC_B: return MG_EFORE;
        case KC_W: return MG_HICH;
        case KC_1 ... KC_0: return KC_DOT;
    }

    return MG_THE;
}

bool process_magic_key_2(uint16_t prev_keycode, uint8_t prev_mods) {
    // Debugging
//    char buffer[64];
//    sprintf(buffer, "Prev Keycode: %d\n", prev_keycode);
//    SEND_STRING(buffer);
//
//    sprintf(buffer, "KC_N: %d, C_MAG_2: %d, KC_DOT: %d", KC_N, C_MAG_2, KC_DOT);
//    SEND_STRING(buffer);

    switch (prev_keycode) {
        case KC_B:
            SEND_STRING("ecome");
            return false;
        case KC_F:
            SEND_STRING("ollow");
            return false;
        case KC_N:
            SEND_STRING("eighbor");
            return false;
        case KC_H:
            SEND_STRING("owever");
            return false;
        case KC_U:
            SEND_STRING("pgrade");
            return false;
        case KC_O:
            SEND_STRING("ther");
            return false;
        case KC_A:
            SEND_STRING("lready");
            return false;
        case KC_P:
            SEND_STRING("sych");
            return false;
        case KC_I:
            SEND_STRING("'ll");
            return false;
        case MT(MOD_LALT, KC_K):
        case KC_K:
            SEND_STRING("now");
            return false;
        case KC_T:
            SEND_STRING("hough");
            return false;
        case KC_L:
            SEND_STRING("ittle");
            return false;
        case KC_M:
        case KC_R:
            SEND_STRING("ight");
            return false;
        case KC_J:
            SEND_STRING("udge");
            return false;
        case KC_C:
            SEND_STRING("ould");
            return false;
        case KC_D:
            SEND_STRING("evelop");
            return false;
        case KC_G:
            SEND_STRING("eneral");
            return false;
        case KC_W:
            SEND_STRING("here");
            return false;
        case MT(MOD_LSFT, KC_S):
        case KC_S:
            SEND_STRING("hould");
            return false;
        case KC_DOT:
            SEND_STRING("org");
            return false;
        case KC_COMM:
            SEND_STRING(" however");
            return false;
        default:
            SEND_STRING("'ll"); //
            return false;
    }
}

bool process_magic_key_3(uint16_t prev_keycode, uint8_t prev_mods) {
    // Debugging
//    char buffer[64];
//    sprintf(buffer, "Prev Keycode: %d\n", prev_keycode);
//    SEND_STRING(buffer);
//
//    sprintf(buffer, "KC_N: %d, C_MAG_2: %d, KC_DOT: %d", KC_N, C_MAG_2, KC_DOT);
//    SEND_STRING(buffer);

    switch (prev_keycode) {
        case KC_B:
            SEND_STRING("etween");
            return false;
        case KC_N:
            SEND_STRING("umber");
            return false;
        case KC_U:
            SEND_STRING("pdate");
            return false;
        case KC_A:
            SEND_STRING("bout");
            return false;
        case KC_W:
            SEND_STRING("orld");
            return false;
        case KC_G:
            SEND_STRING("overn");
            return false;
        case KC_P:
            SEND_STRING("rogram");
            return false;
        case KC_Q:
            SEND_STRING("uestion");
            return false;
        case KC_C:
            SEND_STRING("rowd");
            return false;
        case MT(MOD_LSFT, KC_S):
        case KC_S:
            SEND_STRING("chool");
            return false;
        case KC_T:
            SEND_STRING("hrough");
            return false;
        case KC_M:
            SEND_STRING("anage");
            return false;
        case KC_O:
            SEND_STRING("xygen");
            return false;
        case KC_I:
            SEND_STRING("'m");
            return false;
        case KC_E:
            SEND_STRING("'re");
            return false;
        case KC_DOT:
            SEND_STRING("com");
            return false;
        case KC_COMM:
            SEND_STRING(" since");
            return false;
        default:
            return false;
    }
}

/**
* Original code modified with updates from magic sturdy repo
*/
bool process_record_user(uint16_t keycode, keyrecord_t *record) {

    if (record->event.pressed) {
        int rep_count = get_repeat_key_count();
        if (rep_count < -1 && keycode != MG_UST) {
            send_char('n');
            return false;
        }
        switch (keycode) {
            case C_MAG_2:
                return process_magic_key_2(get_last_keycode(), get_last_mods());
            case C_MAG_3:
                return process_magic_key_3(get_last_keycode(), get_last_mods());
            case MK_DUND:
                SEND_STRING(SS_LSFT(SS_TAP(X_4)) SS_DELAY(100) SS_LSFT(SS_TAP(X_MINUS)));
                return false;
            case MG_ENT:
                SEND_STRING("ent");
                return false;
            case MG_MENT:
                SEND_STRING("ment");
                return false;
            case MG_ER:
                SEND_STRING("er");
                return false;
            case MG_ES:
                SEND_STRING("es");
                return false;
            case MG_UST:
                if (rep_count < -1) {
                    SEND_STRING("ment");
                } else {
                    SEND_STRING("ust");
                }
                return false;
            case MG_OA:
                SEND_STRING("oa");
                return false;
            case MG_ON:
                SEND_STRING("on");
                return false;
            case MG_ION:
                SEND_STRING("ion");
                return false;
            case MG_SP_BUT:
                SEND_STRING(" but");
                return false;
            case MG_THE:
                SEND_STRING("the");
                return false;
            case MG_EFORE:
                SEND_STRING("efore");
                return false;
            case MG_HICH:
                SEND_STRING("hich");
                return false;
            case MG_MLATIV:
                SEND_STRING("mlativ");
                return false;
            case MG_QUOT_S:
                SEND_STRING("'s");
                return false;

            // this also is defined by C_MAG_2 and gives duplicate case value error - for now let's test new magic keys
//            case RGB_SLD:
//                if (record->event.pressed) {
//                    rgblight_mode(1);
//                }
//                return false;
//            case HSV_0_245_245:
//                if (record->event.pressed) {
//                    rgblight_mode(1);
//                    rgblight_sethsv(0,245,245);
//                }
//                return false;
//            case HSV_74_255_255:
//                if (record->event.pressed) {
//                    rgblight_mode(1);
//                    rgblight_sethsv(74,255,255);
//                }
//                return false;
//            case HSV_188_255_255:
//                if (record->event.pressed) {
//                    rgblight_mode(1);
//                    rgblight_sethsv(188,255,255);
//                }
//                return false;
        }

        // Debugging
//        char buffer[64];
//        sprintf(buffer, "Keycode: %d, Mod: %d, Rep: %d", keycode, record->event.pressed, rep_count);
//        SEND_STRING(buffer);
//
//        sprintf(buffer, "KC_DQUO: %d, KC_BSPC: %d, KC_LPRN: %d", KC_DQUO, KC_BSPC, KC_LPRN);
//        SEND_STRING(buffer);

        if (rep_count > 0) { // repeat key used
            switch (keycode) {
                case C_LCTL_BSPC: // ???
                case C_LALT_ENT: // ???
                case C_RSFT_ENT: // ???

                case KC_DQUO: // Doulbe quote. Keycode: 564
                case KC_LPRN: // Left parenthesis. Keycode: 55

                case KC_BSPC: // Backspace. Keycode: 42
                case MT(MOD_LGUI, KC_BSPC): // weird (backspace) keycode which only procuded when using MT(MOD_LGUI, KC_BSPC)

                case KC_SPC: // Space. Keycode: 44
                case LT(ARRWS,KC_SPC): // weird (space) keycode which only procuded when using LT(ARRWS,KC_SPC)

                case KC_ENT: // Enter. Keycode: 40
                case HYPR_T(KC_ENTER): // weird (enter) keycode which only procuded when using HYPR_T(KC_ENTER)
                    unregister_weak_mods(MOD_MASK_CSAG);
                    SEND_STRING("for");
                    return false;
                case KC_I:
                    unregister_weak_mods(MOD_MASK_CSAG);
                    SEND_STRING("ng");
                    return false;
                case KC_DOT:
                case KC_QUES:
                case KC_EXLM:
                case KC_COLN:
                case KC_SCLN:
                    unregister_weak_mods(MOD_MASK_CSAG);
                    send_char(' ');
                    add_oneshot_mods(MOD_MASK_SHIFT);
                    set_last_keycode(KC_SPC);
                    return false;
                case KC_COMM:
                    unregister_weak_mods(MOD_MASK_CSAG);
                    SEND_STRING(" and");
                    return false;
                case KC_A:
                    unregister_weak_mods(MOD_MASK_CSAG);
                    SEND_STRING("nd");
                    return false;
                case KC_N:
                    unregister_weak_mods(MOD_MASK_CSAG);
                    send_char('f');
                    return false;
                case KC_B:
                    unregister_weak_mods(MOD_MASK_CSAG);
                    SEND_STRING("ecause");
                    return false;
                case KC_W:
                    unregister_weak_mods(MOD_MASK_CSAG);
                    SEND_STRING("ould");
                    return false;
                case KC_Y:
                    unregister_weak_mods(MOD_MASK_CSAG);
                    if (rep_count > 2) {
                        SEND_STRING("ll");
                        return false;
                    }
                    if (rep_count > 1) {
                        send_char('\'');
                        return false;
                    }
                    SEND_STRING("ou");
                    return false;
            }
        }
    }

  return true;
}

// Pre-custom magic setup: exported from Oryx configurator

typedef struct {
    bool is_press_action;
    uint8_t step;
} tap;

enum {
    SINGLE_TAP = 1,
    SINGLE_HOLD,
    DOUBLE_TAP,
    DOUBLE_HOLD,
    DOUBLE_SINGLE_TAP,
    MORE_TAPS
};

static tap dance_state[2];

uint8_t dance_step(tap_dance_state_t *state);

uint8_t dance_step(tap_dance_state_t *state) {
    if (state->count == 1) {
        if (state->interrupted || !state->pressed) return SINGLE_TAP;
        else return SINGLE_HOLD;
    } else if (state->count == 2) {
        if (state->interrupted) return DOUBLE_SINGLE_TAP;
        else if (state->pressed) return DOUBLE_HOLD;
        else return DOUBLE_TAP;
    }
    return MORE_TAPS;
}


void on_dance_0(tap_dance_state_t *state, void *user_data);
void dance_0_finished(tap_dance_state_t *state, void *user_data);
void dance_0_reset(tap_dance_state_t *state, void *user_data);

void on_dance_0(tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(KC_GRAVE);
        tap_code16(KC_GRAVE);
        tap_code16(KC_GRAVE);
    }
    if(state->count > 3) {
        tap_code16(KC_GRAVE);
    }
}

void dance_0_finished(tap_dance_state_t *state, void *user_data) {
    dance_state[0].step = dance_step(state);
    switch (dance_state[0].step) {
        case SINGLE_TAP: register_code16(KC_GRAVE); break;
        case SINGLE_HOLD: register_code16(KC_ESCAPE); break;
        case DOUBLE_TAP: register_code16(KC_GRAVE); register_code16(KC_GRAVE); break;
        case DOUBLE_SINGLE_TAP: tap_code16(KC_GRAVE); register_code16(KC_GRAVE);
    }
}

void dance_0_reset(tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[0].step) {
        case SINGLE_TAP: unregister_code16(KC_GRAVE); break;
        case SINGLE_HOLD: unregister_code16(KC_ESCAPE); break;
        case DOUBLE_TAP: unregister_code16(KC_GRAVE); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(KC_GRAVE); break;
    }
    dance_state[0].step = 0;
}
void on_dance_1(tap_dance_state_t *state, void *user_data);
void dance_1_finished(tap_dance_state_t *state, void *user_data);
void dance_1_reset(tap_dance_state_t *state, void *user_data);

void on_dance_1(tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(KC_TILD);
        tap_code16(KC_TILD);
        tap_code16(KC_TILD);
    }
    if(state->count > 3) {
        tap_code16(KC_TILD);
    }
}

void dance_1_finished(tap_dance_state_t *state, void *user_data) {
    dance_state[1].step = dance_step(state);
    switch (dance_state[1].step) {
        case SINGLE_TAP: register_code16(KC_TILD); break;
        case SINGLE_HOLD: register_code16(KC_LEFT_SHIFT); break;
        case DOUBLE_TAP: register_code16(KC_TILD); register_code16(KC_TILD); break;
        case DOUBLE_SINGLE_TAP: tap_code16(KC_TILD); register_code16(KC_TILD);
    }
}

void dance_1_reset(tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[1].step) {
        case SINGLE_TAP: unregister_code16(KC_TILD); break;
        case SINGLE_HOLD: unregister_code16(KC_LEFT_SHIFT); break;
        case DOUBLE_TAP: unregister_code16(KC_TILD); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(KC_TILD); break;
    }
    dance_state[1].step = 0;
}

tap_dance_action_t tap_dance_actions[] = {
        [DANCE_0] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_0, dance_0_finished, dance_0_reset),
        [DANCE_1] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_1, dance_1_finished, dance_1_reset),
};
