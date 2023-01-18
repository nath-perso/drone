#ifndef __PS2_CONTROLLER_H__
#define __PS2_CONTROLLER_H__

/* ================================================================
 * ===                        INCLUDES                          ===
 * ================================================================ */
#include <PS2X_lib.h>


/* ================================================================
 * ===                         DEFINE                           ===
 * ================================================================ */

/* Configuration */
#define PS2_PRESSURE  true
#define PS2_RUMBLE    false

/* ================================================================
 * ===                        VARIABLES                         ===
 * ================================================================ */
 /* Controller connection error codes */
enum PS2_connection_error_code_list : byte {
    no_error_code_k,
    not_found_code_k,
    no_command_code_k,
    no_pressure_code_k
};


/* Controller type codes */
enum PS2_controller_type_code_list : byte {
    unknown_k,
    dualshock_k,
    guitar_hero_k
};

/* ================================================================
 * ===                        FUNCTIONS                         ===
 * ================================================================ */
int ps2ControllerSetup();
void handlePS2Controller();

#endif /* __PS2_CONTROLLER_H__ */