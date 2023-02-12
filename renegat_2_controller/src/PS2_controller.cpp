/* =========================================================================
   NOTE: The library does NOT support hot pluggable controllers, meaning
   you must always either restart your Arduino after you connect the controller,
   or call config_gamepad(pins) again after connecting the controller.
   ========================================================================= */

/* ================================================================
 * ===                        INCLUDES                          ===
 * ================================================================ */
#include "PS2_controller.h"
#include "pins.h"

/* ================================================================
 * ===                        VARIABLES                         ===
 * ================================================================ */
/* Configuration */
#define PS2_PRESSURE  true
#define PS2_RUMBLE    false


PS2X ps2x; /* create PS2 Controller Class */

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

/*
 * @brief   Setup PS2 controller and check connection
 * @returns 0 if successful, error code if not
 */
int ps2ControllerSetup()
{
  /* Setup pins and settings:  config_gamepad(clock, command, attention, data, Pressures?, Rumble?) check for error */
  byte error = ps2x.config_gamepad(PS2_CLK_PIN, PS2_COMMAND_PIN, PS2_ATTENTION_PIN, PS2_DATA_PIN, PS2_PRESSURE, PS2_RUMBLE);

  switch (error)
  {
    /* Everything is fine */
    case no_error_code_k:
    {
      Serial.println("Found Controller, configured successful");

      /* Detect the controller type */
      byte type = ps2x.readType();

      switch (type)
      {
      case unknown_k:
        Serial.println("Unknown Controller type");
        break;

      case dualshock_k:
        Serial.println("DualShock Controller Found");
        break;

      case guitar_hero_k:
        Serial.println("GuitarHero Controller Found");
        break;
      }
    }
    break;

    /* Something went wrong */
    case not_found_code_k:
    {
      Serial.println("No controller found, check wiring, see readme.txt to enable debug. visit www.billporter.info for troubleshooting tips");
    }
    break;

    case no_command_code_k:
    {
      Serial.println("Controller found but not accepting commands. see readme.txt to enable debug. Visit www.billporter.info for troubleshooting tips");
    }
    break;

    case no_pressure_code_k:
    {
      Serial.println("Controller refusing to enter Pressures mode, may not support it. ");
    }
    break;

    default:
    {
      Serial.println("Controller connection error. Code not defined.");
    }
  }

  return (int)error;
}

/*
 * @brief   Read new controller data and handle useful information
 * @help    Button(button_address) :  returns true as long as the button is being held
            Analog(button_address) :  returns pressure reading on button (byte)
            NewButtonState() :        returns true if any button changes state (ON -> OFF / OFF -> ON)
            NewButtonState(button_address) :  returns true if specified button changes state (ON -> OFF / OFF -> ON)
            ButtonPressed(button_address) :   returns true (once) when specified button has been pressed
            ButtonReleased(button_address) :  returns true (once) when specified button has been released
 */
void handlePS2Controller()
{
  /* Read from controller to get buttons information */
  ps2x.read_gamepad();

  if (ps2x.Button(PSB_START))
    Serial.println("Start is being held");
  if (ps2x.Button(PSB_SELECT))
    Serial.println("Select is being held");

  if (ps2x.Button(PSB_PAD_UP))
  {
    Serial.print("Up held this hard: ");
    Serial.println(ps2x.Analog(PSAB_PAD_UP), DEC);
  }
  if (ps2x.Button(PSB_PAD_RIGHT))
  {
    Serial.print("Right held this hard: ");
    Serial.println(ps2x.Analog(PSAB_PAD_RIGHT), DEC);
  }
  if (ps2x.Button(PSB_PAD_LEFT))
  {
    Serial.print("LEFT held this hard: ");
    Serial.println(ps2x.Analog(PSAB_PAD_LEFT), DEC);
  }
  if (ps2x.Button(PSB_PAD_DOWN))
  {
    Serial.print("DOWN held this hard: ");
    Serial.println(ps2x.Analog(PSAB_PAD_DOWN), DEC);
  }

  if (ps2x.NewButtonState())
  {
    if (ps2x.Button(PSB_L3))
      Serial.println("L3 pressed");
    if (ps2x.Button(PSB_R3))
      Serial.println("R3 pressed");
    if (ps2x.Button(PSB_L2))
      Serial.println("L2 pressed");
    if (ps2x.Button(PSB_R2))
      Serial.println("R2 pressed");
    if (ps2x.Button(PSB_GREEN))
      Serial.println("Triangle pressed");
  }

  if (ps2x.ButtonPressed(PSB_RED))
    Serial.println("Circle just pressed");

  if (ps2x.ButtonReleased(PSB_PINK))
    Serial.println("Square just released");

  if (ps2x.NewButtonState(PSB_BLUE))
    Serial.println("X just changed");

  if (ps2x.Button(PSB_L1) || ps2x.Button(PSB_R1))
  {
    Serial.print("Stick Values:");
    Serial.print(ps2x.Analog(PSS_LY), DEC);
    Serial.print(",");
    Serial.print(ps2x.Analog(PSS_LX), DEC);
    Serial.print(",");
    Serial.print(ps2x.Analog(PSS_RY), DEC);
    Serial.print(",");
    Serial.println(ps2x.Analog(PSS_RX), DEC);
  }
}
