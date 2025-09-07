//------------------------------------------------------------------------------
//  file Led_driver.h - For ws2812 led driver / Everlights
//
//  brief
//      Structure defs, #defs, programming enums and lookup tables
//------------------------------------------------------------------------------
//  Disclosure
//  Copyright (c) 2016 Sigma Design
//
//  All information contained herein is the property of Sigma Design, Inc.
//  And are protected trade secrets and copyrights, and may be covered by U.S.
//  and/or foreign patents or patents pending and/or mask works.
//
//  Any reproduction, dissemination or use of any portion of this
//  document or of software or other works derived from it is strictly
//  forbidden unless prior written permission is obtained from
//  Sigma Design.
//------------------------------------------------------------------------------

/************************************************************************/
/* Includes																*/
/************************************************************************/

#include <stdint.h>

/************************************************************************/
/* Static structures, constants and lookup tables						*/
/************************************************************************/

#define N_COLORS		256		// number of colors in LUT
#define N_LEDS			640		// # of leds being driven, This is a requirement
#define LEDS			639		// # of C leds
#define N_BUFFERS		4		// four working display buffers, one byte / pixel, translated through the LUT
#define BYTES_PER_LED	3		// RGB
#define N_DMA_BUFS		2		// # of dma buffers for double buffer operation
#define N_DMA_BYTES		48		// 48 bytes for each DMA buffer
#define N_DMA_PIXELS	16		// 16 pixels in each DMA buffer


typedef struct
{
	uint8_t g;
	uint8_t r;
	uint8_t b;
} ws2812drv_led_t;


enum							// indices into any RGB structure (LUT...)
{
	RED	=	0,		
	GREEN =	1,
	BLUE =	2
};

enum							// indices into any RGB structure (LUT...)
{
	DMA_GREEN =	0,
	DMA_RED	=	1,		
	DMA_BLUE =	2
};

enum {							// indices for working display buffers
	BUF0 =	0,		
	BUF1 =	1,
	BUF2 =	2,
	BUF3 =	3
};

// Display instruction-----------------------------

// Define the color look up table - ROYGBIV settings, 24 intensities / Color
#define RED_LUT			0x0		// Red lookup ( 24 colors )
#define ORANGE_LUT		0x18	// Orange ...
#define YELLOW_LUT		0x30
#define GREEN_LUT		0x48
#define BLUEGREEN_LUT	0x60
#define BLUE_LUT		0x78
#define VIOLET_LUT		0x90
#define LIB_LUT			0xa8	// Library lookup ( 88 selected colors )

// Define Mid-range colors - ROYGBIV settings, 24 intensities 
#define RED_MID			0x10		// Red lookup ( 24 colors )
#define ORANGE_MID		0x28		// Orange ...
#define YELLOW_MID		0x40
#define GREEN_MID		0x58
#define BLUEGREEN_MID	0x70
#define BLUE_MID		0x88
#define VIOLET_MID		0xa0

const uint8_t LUT_INDEX[] PROGMEM = {
	RED_LUT,
	ORANGE_LUT,
	YELLOW_LUT,
	GREEN_LUT,
	BLUEGREEN_LUT,
	BLUE_LUT,
	VIOLET_LUT,
	LIB_LUT,
};

enum LUT_INDEX {
	RED_IDX,
	ORANGE_IDX,
	YELLOW_IDX,
	GREEN_IDX,
	BLUEGREEN_IDX,
	BLUE_IDX,
	VIOLET_IDX,
	LIB_IDX,
};

// Demo patterns that can be selected from the application
#define NONE	0
#define BLINK	1
#define CHASE	2
#define FADE	3
#define STROBE	4
#define TWINK	5
#define RANDOM	6

// ---------------------------------------------------------------------------
//Sequencer structure definitions and opcodes
//----------------------------------------------------------------------------

// define the pixel programming language ( sprite ) --------------------------

enum PIX_PGM  {
	MOVE		= 0x01,
	BOUNCE		= 0x02,
	EXPLODE		= 0x04,
	DECAY		= 0x08,
	SPAWN		= 0x10,
	SPEW		= 0x20,
	GLOBAL		= 0x40,
	DIRECTION	= 0x80,
};



// define the display list sequencing instruction format ---------------------

typedef struct seq {
	uint8_t		opr;		// operator
	uint8_t		br;			// branch control / collision / timer delay address
	uint16_t	opa;		// operand
	uint16_t	start;		// general purpose data ( start address )
	uint16_t	end;		// general purpose data ( end address )
}seq;


// Define the display list macro --------------------------------------------

#define ins(addr,opra,opaa,braddr,starta,enda,p0,p1,p2,p3,p4,p5 )\
		s[addr].opr = (uint8_t)opra;\
		s[addr].br = (uint8_t)braddr;\
		s[addr].opa = (uint16_t)opaa;\
		s[addr].start = ( (uint16_t)( ( starta ) | ( p0 << 10 ) | (p1 << 12 ) | (p2 << 14 ) ) );\
		s[addr].end = ( (uint16_t)( ( enda )| ( p3 << 10 ) | (p4 << 12 ) | (p5 << 14 ) ) ); 			// general purpose data ( end address )

/*
#define INS(s,addr,opra,opaa,bra,starta,enda,p0,p1,p2,p3,p4,p5 )\
		s[addr].br = (const)(bra | ADDR(starta));\
		s[addr].opr = opra;\
		s[addr].opa = opaa;\
		s[addr].start = ( ( starta ) | ( p0 << 10 ) | (p1 << 12 ) | (p2 << 14 ) );\
		s[addr].end = ( ( enda )| ( p3 << 10 ) | (p4 << 12 ) | (p5 << 14 ) ); 			// general purpose data ( end address )		
*/
// ---------------------------------------------------------------------------
// Effects, Logical and math raster operations. 
// Add new operations (OPR) codes here.
//----------------------------------------------------------------------------

// Raster operations - can be performed in parallel --------------------------

enum RASTER_OPR {
	RASOP_NOP			= 0x0000,
	RASOP_AND			= 0x0001,
	RASOP_OR			= 0x0002,
	RASOP_XOR			= 0x0004,
	RASOP_ADD			= 0x0008,
	RASOP_SUB			= 0x0010,
	RASOP_DECREMENT		= 0x0020,
	RASOP_INCREMENT		= 0x0040,
	RASOP_USER			= 0x0080
};

	
// bitplane manipulation - paint, shifts, rotates - 256 total available opcodes

enum OPR {
	NOP = 0,
	ROTATE,
	SHIFT,
	REVERSE_PAT,
	BLIT,
	SWAP,
	LOAD_DELAY,		// load delay register for delay operation
	LOAD_COUNT,		// load count register
	SPARKLE,
	CANNED_EFFECTS,
	
	// Paint operations ------------------------------------
	SOLID,			// solid color
	ALTERNATE,		// alternate color
	GRADIENT,		// gradient ramp of single color
	SIN_GRAD,		// Sin(x) gradient
	RAND,			// random colors ( all )
	RAND_SHADE,		// random shades of a selected color
	RAINBOW,		// raindow pattern	
	RAMP_INTENSITY,	// ramp intensity of buffer

	// Utility operations, set operating modes and enables
	CONTROL,		// control flags
	COLLISION_ENA,	// plane / led pixel collision enables
	BUF_ENA,		// plane / led pixel display enable 
	ANI_SPEED,		// animation speed
	DEMO_MODE,		// reload display list with new demo mode
	RGB_INTENS,		// global intensity controls ( RGB )
	SET_SPARKLE,	// Maximum 32 opcodes!
	
// Branch operations ( high bits of opa[5:7] )
	BR		= 1<<5,		// Unconditional branch
	BRCOL	= 2<<5,		// Branch on collision
	BRCNT	= 3<<5,		// branch on count == 0
	BRDLY	= 4<<5,		// branch on delay != 0 ( until )
	BRREG	= 5<<5,		// branch on register var
	BRMWAY	= 6<<5,		// multiway branch 
	HALT	= 7<<5		// Stop for refresh operation
};

// Comm decoder state machine control --------------------------

#define NIL 0		// bypass flag for data already acquired in post cmd decode

enum {
IDLE,				// Waiting for command
CMD,				// Have command - In command mode
CMD_COLOR,			// command is load color
CMD_SPEED,			// command is set speed 
CMD_INTENS,			// command is set intensity
CMD_PAT,			// command is set pattern
CMD_MODE,			// command is set mode ( global mode flags )
CMD_DLIST,			// load display list - suspend display list execution
CMD_UPDATE,			// update individual LED and refresh
LOW_NIB,			// low nibble of byte
HI_NIB,				// Hi nibble of byte
STO_RED,			// store RED
STO_GREEN,			// store GREEN
STO_BLUE,			// store BLUE
STO_SPEED,			// set dlist execution speed
STO_INTENS,			// set intensity ( global )
STO_PAT,			// set pattern
STO_MODE,			// set mode
STO_CMD,			// 
STO_DLIST,			// store display list instructions ***Be careful***
};

// Color map / LUT initialization
// Let's distribute the below map and interpolate 
// between colors to fully fill the LUT. The following was 
// sorted and hand adjusted to give ROYGBIV normalized
// with 24 levels of intensity, gamma and observational metrics
// to correct. 

#define MAX_LIB_LUT (sizeof(lib_LUT)/BYTES_PER_LED)

// The available stock library colors ------------------------------

const uint8_t lib_LUT[][BYTES_PER_LED] PROGMEM = {
	{ 0xCD,0x5C,0x5C },    //   IndianRed                
	{ 0xF0,0x80,0x80 },    //   LightCoral               
	{ 0xFA,0x80,0x72 },    //   Salmon                   
	{ 0xE9,0x96,0x7A },    //   DarkSalmon               
	{ 0xFF,0xA0,0x7A },    //   LightSalmon              
	{ 0xDC,0x14,0x3C },    //   Crimson                  
	{ 0xB2,0x22,0x22 },    //   FireBrick                
	{ 0xFF,0xC0,0xCB },    //   Pink                     
	{ 0xFF,0xB6,0xC1 },    //   LightPink                
	{ 0xFF,0x69,0xB4 },    //   HotPink                  
	{ 0xFF,0x14,0x93 },    //   DeepPink                 
	{ 0xC7,0x15,0x85 },    //   MediumVioletRed          
	{ 0xDB,0x70,0x93 },    //   PaleVioletRed            
	{ 0xFF,0x7F,0x50 },    //   Coral                    
	{ 0xFF,0x63,0x47 },    //   Tomato                   
	{ 0xFF,0xFF,0xE0 },    //   LightYellow              
	{ 0xFF,0xFA,0xCD },    //   LemonChiffon             
	{ 0xFA,0xFA,0xD2 },    //   LightGoldenrodYellow     
	{ 0xFF,0xEF,0xD5 },    //   PapayaWhip               
	{ 0xFF,0xE4,0xB5 },    //   Moccasin                 
	{ 0xFF,0xDA,0xB9 },    //   PeachPuff                
	{ 0xEE,0xE8,0xAA },    //   PaleGoldenrod            
	{ 0xF0,0xE6,0x8C },    //   Khaki                    
	{ 0xBD,0xB7,0x6B },    //   DarkKhaki                
	{ 0xE6,0xE6,0xFA },    //   Lavender                 
	{ 0xD8,0xBF,0xD8 },    //   Thistle                  
	{ 0xDD,0xA0,0xDD },    //   Plum                     
	{ 0xDA,0x70,0xD6 },    //   Orchid                   
	{ 0xBA,0x55,0xD3 },    //   MediumOrchid             
	{ 0x93,0x70,0xDB },    //   MediumPurple             
	{ 0x99,0x66,0xCC },    //   Amethyst                 
	{ 0x8A,0x2B,0xE2 },    //   BlueViolet               
	{ 0x99,0x32,0xCC },    //   DarkOrchid               
	{ 0x6A,0x5A,0xCD },    //   SlateBlue                
	{ 0x48,0x3D,0x8B },    //   DarkSlateBlue            
	{ 0x7B,0x68,0xEE },    //   MediumSlateBlue          
	{ 0xAD,0xFF,0x2F },    //   GreenYellow              
	{ 0x32,0xCD,0x32 },    //   LimeGreen                
	{ 0x98,0xFB,0x98 },    //   PaleGreen                
	{ 0x90,0xEE,0x90 },    //   LightGreen               
	{ 0x3C,0xB3,0x71 },    //   MediumSeaGreen           
	{ 0x2E,0x8B,0x57 },    //   SeaGreen                 
	{ 0x22,0x8B,0x22 },    //   ForestGreen              
	{ 0x9A,0xCD,0x32 },    //   YellowGreen              
	{ 0x6B,0x8E,0x23 },    //   OliveDrab                
	{ 0x55,0x6B,0x2F },    //   DarkOliveGreen           
	{ 0x66,0xCD,0xAA },    //   MediumAquamarine         
	{ 0x8F,0xBC,0x8F },    //   DarkSeaGreen             
	{ 0x20,0xB2,0xAA },    //   LightSeaGreen            
	{ 0xE0,0xFF,0xFF },    //   LightCyan                
	{ 0xAF,0xEE,0xEE },    //   PaleTurquoise            
	{ 0x7F,0xFF,0xD4 },    //   Aquamarine               
	{ 0x40,0xE0,0xD0 },    //   Turquoise                
	{ 0x48,0xD1,0xCC },    //   MediumTurquoise          
	{ 0x5F,0x9E,0xA0 },    //   CadetBlue                
	{ 0x46,0x82,0xB4 },    //   SteelBlue                
	{ 0xB0,0xC4,0xDE },    //   LightSteelBlue           
	{ 0xB0,0xE0,0xE6 },    //   PowderBlue               
	{ 0xAD,0xD8,0xE6 },    //   LightBlue                
	{ 0x87,0xCE,0xEB },    //   SkyBlue                  
	{ 0x87,0xCE,0xFA },    //   LightSkyBlue             
	{ 0x1E,0x90,0xFF },    //   DodgerBlue               
	{ 0x64,0x95,0xED },    //   CornflowerBlue           
	{ 0x7B,0x68,0xEE },    //   MediumSlateBlue          
	{ 0x41,0x69,0xE1 },    //   RoyalBlue                
	{ 0x19,0x19,0x70 },    //   MidnightBlue             
	{ 0xFF,0xF0,0xF5 },    //   LavenderBlush
	{ 0xFF,0xE4,0xE1 },    //   MistyRose
	{ 0xFF,0xF8,0xDC },    //   Cornsilk                 
	{ 0xFF,0xEB,0xCD },    //   BlanchedAlmond           
	{ 0xFF,0xE4,0xC4 },    //   Bisque                   
	{ 0xFF,0xDE,0xAD },    //   NavajoWhite              
	{ 0xF5,0xDE,0xB3 },    //   Wheat                    
	{ 0xDE,0xB8,0x87 },    //   BurlyWood                
	{ 0xD2,0xB4,0x8C },    //   Tan                      
	{ 0xBC,0x8F,0x8F },    //   RosyBrown                
	{ 0xF4,0xA4,0x60 },    //   SandyBrown               
	{ 0xDA,0xA5,0x20 },    //   Goldenrod                
	{ 0xB8,0x86,0x0B },    //   DarkGoldenrod            
	{ 0xCD,0x85,0x3F },    //   Peru                     
	{ 0xD2,0x69,0x1E },    //   Chocolate                
	{ 0x8B,0x45,0x13 },    //   SaddleBrown              
	{ 0xA0,0x52,0x2D },    //   Sienna                   
	{ 0xA5,0x2A,0x2A },    //   Brown                    
	{ 0xFF,0xFF,0xFF },    //   White                    
	{ 0xFF,0xFA,0xFA },    //   Snow                     
	{ 0xF0,0xFF,0xF0 },    //   Honeydew                 
	{ 0xF5,0xFF,0xFA },    //   MintCream                
	{ 0xF0,0xFF,0xFF },    //   Azure                    
	{ 0xF0,0xF8,0xFF },    //   AliceBlue                
	{ 0xF8,0xF8,0xFF },    //   GhostWhite               
	{ 0xF5,0xF5,0xF5 },    //   WhiteSmoke               
	{ 0xFF,0xF5,0xEE },    //   Seashell                 
	{ 0xF5,0xF5,0xDC },    //   Beige                    
	{ 0xFD,0xF5,0xE6 },    //   OldLace                  
	{ 0xFF,0xFA,0xF0 },    //   FloralWhite              
	{ 0xFF,0xFF,0xF0 },    //   Ivory                    
	{ 0xFA,0xEB,0xD7 },    //   AntiqueWhite             
	{ 0xFA,0xF0,0xE6 },    //   Linen                    
	{ 0x2F,0x4F,0x4F },    //   DarkSlateGray
	{ 0x69,0x69,0x69 },    //   DimGray
	{ 0x77,0x88,0x99 },    //   LightSlateGray
	{ 0x70,0x80,0x90 },    //   SlateGray
	{ 0x80,0x80,0x80 },    //   Gray
	{ 0xA9,0xA9,0xA9 },    //   DarkGray
	{ 0xC0,0xC0,0xC0 },    //   Silver
	{ 0xD3,0xD3,0xD3 },    //   LightGrey                
	{ 0xDC,0xDC,0xDC },    //   Gainsboro
	{ 0xff,0xff,0xff }     //   White          - Test
};

/*
typedef struct rainbow_pallet {
	unsigned char red;
	unsigned char green;
	unsigned char blue;
} rainbow_pallet;

#define MAX_RAINBOW_PALLET 12

rainbow_pallet rainbow[MAX_RAINBOW_PALLET] = {
	{ 0x20, 0x0, 0x0 },		// Red
	{ 0xff, 0x0, 0x0 },
	{ 0x20, 0x10, 0x0 },	// Orange
	{ 0xff, 0x80, 0x0 },
	{ 0x20, 0x20, 0x0 },	// Yellow
	{ 0xff, 0xff, 0x0 },
	{ 0x0, 0x20, 0x0 },		// Green
	{ 0x0, 0xff, 0x0 },
	{ 0x0, 0x0, 0x20 },		// Blue
	{ 0x0, 0x00, 0xff },
	{ 0x20, 0x0, 0x20 },	// violet
	{ 0xff, 0x0, 0xff }
};
*/

/* gamma / intensity correction table for WS2812 LEDS -- Thanks to Idafruit ----------*/

const uint8_t gamma_tbl[] PROGMEM = {
        0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
        0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  1,  1,  1,
        1,  1,  1,  1,  1,  1,  1,  1,  1,  2,  2,  2,  2,  2,  2,  2,
        2,  3,  3,  3,  3,  3,  3,  3,  4,  4,  4,  4,  4,  5,  5,  5,
        5,  6,  6,  6,  6,  7,  7,  7,  7,  8,  8,  8,  9,  9,  9, 10,
       10, 10, 11, 11, 11, 12, 12, 13, 13, 13, 14, 14, 15, 15, 16, 16,
       17, 17, 18, 18, 19, 19, 20, 20, 21, 21, 22, 22, 23, 24, 24, 25,
       25, 26, 27, 27, 28, 29, 29, 30, 31, 32, 32, 33, 34, 35, 35, 36,
       37, 38, 39, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 50,
       51, 52, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 66, 67, 68,
       69, 70, 72, 73, 74, 75, 77, 78, 79, 81, 82, 83, 85, 86, 87, 89,
       90, 92, 93, 95, 96, 98, 99,101,102,104,105,107,109,110,112,114,
      115,117,119,120,122,124,126,127,129,131,133,135,137,138,140,142,
      144,146,148,150,152,154,156,158,160,162,164,167,169,171,173,175,
      177,180,182,184,186,189,191,193,196,198,200,203,205,208,210,213,
      215,218,220,223,225,228,231,233,236,239,241,244,247,249,252,255 };
	  
// Sin tables for wave like patterns ----------------------------

const uint8_t sin_tbl[] PROGMEM = {
	0x00, 0x01, 0x03, 0x04, 
	0x06, 0x07, 0x09, 0x0a, 
	0x0c, 0x0d, 0x0e, 0x10, 
	0x11, 0x12, 0x13, 0x14, 
	0x15, 0x15, 0x16, 0x16, 
	0x17, 0x17, 0x17, 0x17, 
	0x17, 0x17, 0x17, 0x17, 
	0x16, 0x16, 0x15, 0x15, 
	0x14, 0x13, 0x12, 0x11, 
	0x10, 0x0e, 0x0d, 0x0c, 
	0x0a, 0x09, 0x07, 0x06, 
	0x04, 0x03, 0x01, 0x00 };

