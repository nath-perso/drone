#ifndef __PS2_CONTROLLER_H__
#define __PS2_CONTROLLER_H__

/* ================================================================
 * ===                        INCLUDES                          ===
 * ================================================================ */
#include <PS2X_lib.h>


/* ================================================================
 * ===                         DEFINE                           ===
 * ================================================================ */


/* ================================================================
 * ===                        VARIABLES                         ===
 * ================================================================ */
//analogs
typedef struct analogs {
	unsigned int leftX;
	unsigned int leftY;
	unsigned int rghtX;
	unsigned int rghtY;
};

//digitals
typedef union digitals{
	struct {
		unsigned int l1 : 1;
		unsigned int l2 : 1;
		unsigned int l3 : 1;
		unsigned int r1 : 1;
		unsigned int r2 : 1;
		unsigned int r3 : 1;
		unsigned int up : 1;
		unsigned int dn : 1;
		unsigned int lf : 1;
		unsigned int rt : 1;
		unsigned int tr : 1;
		unsigned int cr : 1;
		unsigned int sq : 1;
		unsigned int cl : 1;
		unsigned int sl : 1;
		unsigned int st : 1;
	}bits;
	unsigned int onoff;
} ;

struct keys {
	analogs analogKeys;
	digitals digitalKeys;
};


/* ================================================================
 * ===                        FUNCTIONS                         ===
 * ================================================================ */
int ps2ControllerSetup();
void handlePS2Controller(keys *keys);
bool ps2IsButtonPressed(uint16_t button);

#endif /* __PS2_CONTROLLER_H__ */