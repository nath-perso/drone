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
/* Analogs */
struct analogs {
	unsigned int leftX;
	unsigned int leftY;
	unsigned int rghtX;
	unsigned int rghtY;
};

/* Digitals */
union digitals{
	struct {
		unsigned int l1 : 1;
		unsigned int l2 : 1;
		unsigned int l3 : 1;
		unsigned int r1 : 1;
		unsigned int r2 : 1;
		unsigned int r3 : 1;
		unsigned int up 	: 1;
		unsigned int down 	: 1;
		unsigned int left 	: 1;
		unsigned int right 	: 1;
		unsigned int triangle : 1;
		unsigned int cross 	: 1;
		unsigned int square : 1;
		unsigned int circle : 1;
		unsigned int select : 1;
		unsigned int start 	: 1;
	}bits;
	unsigned int onoff;
} ;

/* Key structure */
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