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
int toggleButton(unsigned int mask, keys *keys);

/**
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

/**
 * @brief   Read new controller data and handle useful information
 * @note    Button(button_address) :  returns true as long as the button is being held
 * @note    Analog(button_address) :  returns pressure reading on button (byte)
 * @note    NewButtonState() :        returns true if any button changes state (ON -> OFF / OFF -> ON)
 * @note    NewButtonState(button_address) :  returns true if specified button changes state (ON -> OFF / OFF -> ON)
 * @note    ButtonPressed(button_address) :   returns true (once) when specified button has been pressed
 * @note    ButtonReleased(button_address) :  returns true (once) when specified button has been released
 */
void handlePS2Controller(keys *keys)
{
  /* Read from controller to get buttons information */
  ps2x.read_gamepad();  //read controller but set rumble motor to off

	//analog readings
	keys->analogKeys.leftX = ps2x.Analog(PSS_LX);	// left analog stick
	keys->analogKeys.leftY = ps2x.Analog(PSS_LY);
	keys->analogKeys.rghtX = ps2x.Analog(PSS_RX);	// right analog stick
	keys->analogKeys.rghtY = ps2x.Analog(PSS_RY);

	// digital readings
	if (ps2x.NewButtonState()) {
		// use masking to toggle the value
		if (ps2x.ButtonPressed(PSB_L1))			keys->digitalKeys.bits.l1 = toggleButton(0x0001, keys);
		if (ps2x.ButtonPressed(PSB_L2))			keys->digitalKeys.bits.l2 = toggleButton(0x0002, keys);
		if (ps2x.ButtonPressed(PSB_L3))			keys->digitalKeys.bits.l3 = toggleButton(0x0004, keys);

		if (ps2x.ButtonPressed(PSB_R1))			keys->digitalKeys.bits.r1 = toggleButton(0x0008, keys);
		if (ps2x.ButtonPressed(PSB_R2))			keys->digitalKeys.bits.r2 = toggleButton(0x0010, keys);
		if (ps2x.ButtonPressed(PSB_R3))			keys->digitalKeys.bits.r3 = toggleButton(0x0020, keys);

		if (ps2x.ButtonPressed(PSB_PAD_UP))		keys->digitalKeys.bits.up = toggleButton(0x0040, keys);
		if (ps2x.ButtonPressed(PSB_PAD_DOWN))	keys->digitalKeys.bits.dn = toggleButton(0x0080, keys);
		if (ps2x.ButtonPressed(PSB_PAD_LEFT))	keys->digitalKeys.bits.lf = toggleButton(0x0100, keys);
		if (ps2x.ButtonPressed(PSB_PAD_RIGHT))	keys->digitalKeys.bits.rt = toggleButton(0x0200, keys);

		if (ps2x.ButtonPressed(PSB_TRIANGLE))	keys->digitalKeys.bits.tr = toggleButton(0x0400, keys);
		
		if (ps2x.ButtonPressed(PSB_CROSS))		keys->digitalKeys.bits.cr = toggleButton(0x0800, keys);
		if (ps2x.ButtonReleased(PSB_CROSS))		keys->digitalKeys.bits.cr = toggleButton(0x0800, keys);

		if (ps2x.ButtonPressed(PSB_SQUARE))		keys->digitalKeys.bits.sq = toggleButton(0x1000, keys);
		if (ps2x.ButtonPressed(PSB_CIRCLE))		keys->digitalKeys.bits.cl = toggleButton(0x2000, keys);

		if (ps2x.ButtonPressed(PSB_SELECT))		keys->digitalKeys.bits.sl = toggleButton(0x4000, keys);
		if (ps2x.ButtonPressed(PSB_START))		keys->digitalKeys.bits.st = toggleButton(0x8000, keys);
	}
}

int toggleButton(unsigned int mask, keys *keys) {
	if ((keys->digitalKeys.onoff & mask) == 0) {
		return 1;
	}
	else {
		return 0;
	}
}


/**
 * @brief   Check if the specified button is pressed
 * @returns True if pressed
 */
bool ps2IsButtonPressed(uint16_t button)
{
  /* Read from controller to get buttons information */
  ps2x.read_gamepad();
  return ps2x.Button(button);
}