/*
 * led_driver.c
 *
 * Created: 3/30/2016 11:43:35 AM
 * Author : jim.davis
 *
 */ 

#include <avr/io.h>
#define F_CPU 32000000
#include <util/delay.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h> 
#include <stdlib.h>
#include <math.h>
#include "led_driver.h"

//*************************************************************************
// Globals ----------------------------------------------------------------
//*************************************************************************


// buffer selection masks applied on start and end Hi bits [15:10]

#define BUFMASK		0xfc00		// used on high byte of seq start / end for buffer selection (2)[15:14] (1)[13:12] (0)[11:10]
#define ADRMASK		0x03ff		// start and end
#define BUFMASK0	0x0c00		// used on high byte of seq start / end for buffer selection [11:10] - dest
#define BUFMASK1	0x3000		// used on high byte of seq start / end for buffer selection [13:12] - src1
#define BUFMASK2	0xc000		// used on high byte of seq start / end for buffer selection [15:14] - src2
#define AUX(start,end) ( ( (start & BUFMASK) >> 13 ) | ( ( end & BUFMASK >> 10 ) ) ) 

// #define CANARY	2		//	canary bytes at end of pix buffer to track down the DMA problem

// Define Pixel buffers - 4 X 600 led buffers for overlay and special effects
#ifdef CANARY
volatile uint8_t pixbuf[N_BUFFERS][N_LEDS+CANARY];	// pixel buffers ( 640 X 8 bit LUT mapped leds X 4 or RGBI )
#else
volatile uint8_t pixbuf[N_BUFFERS][N_LEDS];			// pixel buffers ( 640 X 8 bit LUT mapped leds X 4 or RGBI )
#endif

// Double buffer for DMA
volatile uint8_t dma_buf[N_DMA_BUFS][N_DMA_PIXELS][BYTES_PER_LED];

// LED color lookup table - 256 X 3 (RGB...)
static uint8_t lut[BYTES_PER_LED][N_COLORS];
#define CUSTOM_COLORS	87

#define max(x,y) (x>y?x:y)
#define min(x,y) (x>y?y:x)
#define dir(x,y) ((int8_t)((x<y)?1:-1))

// MISC ENABLE / DISABLE MACROS

#define TOGBUF(buf_ena) (buf_ena & ( 0x0f ^ (1 << sbuf1)))	// toggle buffer enable flag


//************************************************************************
// Display control list --------------------------------------------------
//************************************************************************ 

#define N_INS 64				// # of display list instruction program slots (2 extra bits in branch address!)
seq s[N_INS];					// define the pattern sequencing program buffer


// Global control registers  ---------------------------------------------------------------
uint8_t		collision_ena = 0xff;							// Collision detection enables for each plane
uint8_t		buf_ena = 0x01;									// buffer enable / / display = 1 / don't display = 0
uint16_t	count[N_BUFFERS] = { 0, 0, 0, 0};				// cycle count for programming
uint16_t	delay[N_BUFFERS] = {0, 0, 0, 0};				// sequence delay for programming
// uint16_t	gp_reg[16];										// general purpose registers for display list programming
uint8_t		sparkle_ena = 0;								// sparkle enable 
uint16_t	sparkle_density = 0;							// density of sparkle pattern
uint16_t	sparkle_decay = 0;								// decay rate of sparkle pattern
uint16_t	sparkle_color = 0;								// pick 2 colors, random selection
uint8_t		quenched = 0;									// set when all sparkle == 0
uint8_t		reset_pgm = 0;									// restart display list program
uint8_t		speed = 0;										// animation speed
int16_t		fade = 0;										// fade control
uint16_t	frame_count = 0;								// total # of frames displayed ( wraps at 0 )
	
volatile uint8_t		animation_speed = 0x0;				// global animation speed
volatile uint8_t		twinkle_rate = 0x00;				// rate of twinkle ( sticky )
volatile uint8_t		fade_rate = 0x00;					// rate of fade ( sticky )
volatile uint8_t		demo_mode = NONE;					// changed in interrupt handler
volatile uint8_t		last_demo_mode = 99;				// make it change on startup
volatile uint8_t		pattern_len = 0;					// pattern length - Max default
volatile uint8_t		busy = 0;							// updates in progress, don't refresh
volatile uint8_t		dlist_len = 0;						// display list loading if non-zero
volatile uint8_t		refresh = 0;						// set by interrupt handler ( update display )  TOGGLES to update parameters

// Control bits
#define  LUT_ENA		0x0001								// bitfield for control ( LUT or raw RGBI mode )
#define	 DLIST_ENA		0x0002								// enable display list operations
#define	 MIX_SEL		0x0004								// Select mix or priority operation
#define  MIX_XOR		0x0008								// use XOR in mix operation
#define  MIX_OR			0x0010								// use BOOL OR in mix operation
#define  MIX_ADD		0x0020								// use Arith ADD in mix operation
#define  COLL_ENA		0x0040								// Global Collision enable
#define  PRIORITY		0x0080								// Color or plane priority display 
#define  GAMMA			0x0100								// Enable gamma LUT ( Adafruit and others )
#define  MIX_SEQ		0x0200								// Mix with sequencing / priority ( take turns )
#define  PIX_PGM		0x0400								// enable pixel programming ( per pixed action/sprites )
#define  TIE_POS		0x0800								// Tie color to position ( X-pos + pixel ) % 255
#define  TWINKLE		0x1000								// enable twinkle ( refresh operation )
#define  SPARK_ENA		0x2000								// enable sparkle ( refresh operation )
#define  FADE_ENA		0x4000								// enable fade ( refresh operation )

// test control selects ----------------------------------------------------------------
volatile uint16_t		control = DLIST_ENA | LUT_ENA;		// global control bits
//volatile uint16_t		control = DLIST_ENA | MIX_XOR | MIX_OR | MIX_ADD;		// global control bits
//volatile uint16_t		control = DLIST_ENA | MIX_XOR ;		// global control bits
//volatile uint16_t		control = DLIST_ENA | MIX_XOR;		// global control bits
//volatile uint16_t		control = DLIST_ENA | PRIORITY;		// global control bits

// intensity controls, subtracted with floor at 0 at display time

int16_t red_intensity = 0x080;		// set for default maximum
int16_t blue_intensity = 0x080;		// set for default maximum
int16_t green_intensity = 0x080;		// set for default maximum

;
// prototypes ---------------------------------------------------------

void main(void);
void initialize_clocks(void);
void initialize_timer(void);
void initialize_edma(void);
void initialize_event(void);
void initialize_pwm(void);
void initialize_gpio(void);
void populate_LUT(void);
void ws2812drv_init(void);
void ws2812drv_start_transfer(ws2812drv_led_t *p, uint16_t cnt);
void blit_left(uint16_t start, uint16_t end, uint16_t count, uint8_t sbuf0, uint8_t sbuf1 );
uint8_t ws2812drv_busy(void);
uint8_t do_animation(uint8_t pc);
uint8_t populate_demo(uint8_t demo_mode);
void clear_all_led_buf();
void rotate( uint16_t start, uint16_t end, uint8_t buf);


// XOR polynomial random number generator
// returns 0-255

uint8_t rnd()
{
static 	uint16_t lfsr = 0x3832;
	do {
        /* taps: 16 14 13 11; feedback polynomial: x^16 + x^14 + x^13 + x^11 + 1 */
        uint16_t bit  = ((lfsr >> 0) ^ (lfsr >> 2) ^ (lfsr >> 3) ^ (lfsr >> 5) ) & 1;
        lfsr =  (lfsr >> 1) | (bit << 15);
	} while( ( lfsr == 0 ) || ( lfsr == 0xff ) );
	return lfsr & 0xff;
}


typedef struct RGB_t { unsigned char red, green, blue; } RGB;
typedef struct HSB_t { float hue, saturation, brightness; } HSB;

/*
 * Returns the hue, saturation, and brightness of the color.
 */
void RgbToHsb(struct RGB_t rgb, struct HSB_t* outHsb)
{
    // TODO check arguments

    float r = rgb.red / 255.0f;
    float g = rgb.green / 255.0f;
    float b = rgb.blue / 255.0f;
    float max = fmaxf(fmaxf(r, g), b);
    float min = fminf(fminf(r, g), b);
    float delta = max - min;
    if (delta != 0)
    {
        float hue;
        if (r == max)
        {
            hue = (g - b) / delta;
        }
        else
        {
            if (g == max)
            {
                hue = 2 + (b - r) / delta;
            }
            else
            {
                hue = 4 + (r - g) / delta;
            }
        }
        hue *= 60;
        if (hue < 0) hue += 360;
        outHsb->hue = hue;
    }
    else
    {
        outHsb->hue = 0;
    }
    outHsb->saturation = max == 0 ? 0 : (max - min) / max;
    outHsb->brightness = max;
}

/*
Typical usage and test:

int main()
{
    struct RGB_t rgb = { 132, 34, 255 };
    struct HSB_t hsb;

    RgbToHsb(rgb, &hsb);

    printf("RGB(%u,%u,%u) -> HSB(%f,%f,%f)\n", rgb.red, rgb.green, rgb.blue,
           hsb.hue, hsb.saturation, hsb.brightness);
    // prints: RGB(132,34,255) -> HSB(266.606354,0.866667,1.000000)

    return 0;
}
*/
// Hardware Initialization functions ----------------------------------------------------------------------

void initialize_clocks(void)
{
	CCP = CCP_IOREG_gc;
	OSC.CTRL = OSC_RC32MEN_bm;
	while(!(OSC.STATUS & OSC_RC32MRDY_bm));
	CCP = CCP_IOREG_gc;
	CLK.CTRL = CLK_SCLKSEL_RC32M_gc;
}

void initialize_gpio(void)
{
	PORTC.DIR = PIN0_bm;
}

void initialize_timer(void)
{
//	TC45_CLKSEL_DIV1024_gc
}

void InitUsartD0()
{
	PORTD_DIR = PORTD_DIR | (1<<3); // PE3 (TX) as output
	PORTD_DIR = PORTD_DIR & ~(1<<2); // PE2 (RX) as input.
	
	USARTD0.BAUDCTRLA = 103;	// set baud rate to 19,200 with clk(PER) = 32MHz.
//	USARTD0.BAUDCTRLA = 34;		// set baud rate to 115,200 with clk(PER) = 32MHz.
	USARTD0.BAUDCTRLB = 0;		// BSCALE is zero.
	USARTD0.CTRLB = 0x18;		// enable both transmitter and receiver
	USARTD0.CTRLC = 0x03;		// select asynchronous USART, disable parity, 1 stop bit, 8 data bits.


	USARTD0.CTRLA = (USARTD0.CTRLA & (~(USART_RXCINTLVL_gm | USART_TXCINTLVL_gm | USART_DREINTLVL_gm))) |
					USART_RXCINTLVL_LO_gc | USART_TXCINTLVL_OFF_gc | USART_DREINTLVL_OFF_gc;
	// setup interrupt controller
	PMIC.CTRL = PMIC_LOLVLEN_bm | PMIC_MEDLVLEN_bm | PMIC_HILVLEN_bm;
	sei();
}


// *** interrupt Service Routine *** //
// Build a little state machine to fill in the blanks as the data comes in.


ISR(USARTD0_RXC_vect)	//  void usartd0_rx_isr(void)
{
static	uint8_t count = 0;				// count for color map loading / display list instructions
static  uint8_t index = 1;				// index into the color map (LUT) ( skip 0 == black )
static  uint8_t byte = 0;				// byte received and decoded from Tiva / App
static 	uint8_t state = IDLE;			// Current decode state
static	uint8_t post_state = IDLE;		// post decode state
static	uint8_t data_state = HI_NIB;	// always high nibble first pass
static  uint8_t *dp;					// display list loader pointer ( char )
static uint8_t c;						// for visibility


	PORTD.OUTTGL = PIN5_bm;					// turn off the interrupt noise
	if(USARTD0.STATUS & USART_RXCIF_bm)		// character in fifo (uart)?
	{
	// get character, start to decode and gather data depending on state info
 		c = USARTD0.DATA;
		if( (c >= '0') && ( c <= 'f') )
		{
			if( state != CMD )
			{		
				switch(c)
				{
					case 'C':		// select color
						state = HI_NIB;
						post_state = CMD_COLOR;
						index = 1;
						count = 0;
						busy = 1;		// stop refresh until we have color map loaded
					break;
		
					case 'D':		// program display list
						state = HI_NIB;
						post_state = CMD_DLIST;
						index = 0;
						count = 0;
						dp = (uint8_t *)s;		// setup display list pointer
						busy = 1;				// stop refresh until we have display list loaded
					break;
					
					case 'S':		// Speed -----------------------
						state = HI_NIB;
						post_state = CMD_SPEED;
					break;
			
					case 'I':		// Intensity -------------------
						state = HI_NIB;
						post_state = CMD_INTENS;
//						control &= 0xffff ^ TWINKLE;
					break;
				
					case 'P':		// Pattern selection -----------------
						state = HI_NIB;
						post_state = CMD_PAT;
					break;
	
					case 'M':	// Display mode selection -----------------
						state = HI_NIB;
						post_state = CMD_MODE;
					break;	
					
					case 'U':	// Update color selection -----------------
						state = HI_NIB;
						post_state = CMD_UPDATE;
					break;
						
					// decode the hex digits to a byte					
					case '0':
					case '1':
					case '2':
					case '3':
					case '4':
					case '5':
					case '6':
					case '7':
					case '8':
					case '9':
					case 'a':
					case 'b':
					case 'c':
					case 'd':
					case 'e':
					case 'f':		// decode hex digits here, change state
					if( state == HI_NIB )
					{
						byte = ((c - '0') >  9 ? ( c - 'a' + 10 ) : ( c - '0' ) ) << 4;
						state = LOW_NIB;
					}
					else
					if( state == LOW_NIB )
					{	
						byte |= (c - '0') >  9 ? ( c - 'a' + 10 ) : c - '0' ;
						state = CMD;
						data_state = HI_NIB;		// fool the post processor
						c = NIL;
					}
					break;
					
					default:
//						state = IDLE;			// Current decode state
//						post_state = IDLE;		// post decode state
//						data_state = HI_NIB;	// always high nibble first pass
//						count = 0;
//						byte = 0;
					break;
				}
			}
			// CMD mode set above in LOW_NIB -------------------
			if( state == CMD )		// first pass, don't process character twice since we have all the data for most commands
			{
				switch(c)		// get characters and process the hex digits expected
				{
					case NIL:	// have data already processed, bypass hex processing
					break;
					// decode the hex digits to a byte
					case '0':
					case '1':
					case '2':
					case '3':
					case '4':
					case '5':
					case '6':
					case '7':
					case '8':
					case '9':
					case 'a':
					case 'b':
					case 'c':
					case 'd':
					case 'e':
					case 'f':		// decode hex digits here, change state
					if( data_state == HI_NIB )
					{
						byte = ((c - '0') >  9 ? ( c - 'a' + 10 ) : ( c - '0' ) ) << 4;
						data_state = LOW_NIB;
					}
					else
					if( data_state == LOW_NIB )
					{	
						byte |= (c - '0') >  9 ? ( c - 'a' + 10 ) : c - '0' ;
						data_state = HI_NIB;		// reset hex processing
					}
					break;
					
					default:	// remain in ready state - funny character over the network
//						byte = 0;
//						state = IDLE;						// Current decode state
//						post_state = IDLE;					// post decode state
//						data_state = HI_NIB;				// always high nibble first pass
//						count = 0;
					break;
				}
			
				if( data_state == HI_NIB )					// have byte data for color
				{	
					switch( post_state )					// write over the Base Library colors 
					{
						case CMD_COLOR:
							index = 1;						// offset from 0 ( 0 == black always )
							count = byte;					// set byte count for color loading
							pattern_len = count;			// save pattern length
							post_state = STO_GREEN;			// on complete byte, store green at index
							data_state = HI_NIB;			// ready to get data
						break;

						case CMD_DLIST:
							index = 0;
							count = byte;					// set byte count for display list loading
							dlist_len = count;				// save pattern length
							post_state = STO_DLIST;			// on complete byte, store display list at index
							data_state = HI_NIB;			// ready to get data
						break;
				
						case CMD_SPEED:
//							animation_speed = (255-byte) >> 2 ;		// global animation speed
/*							if( demo_mode == TWINK )
							{
								twinkle_rate = 255-byte;
							}
							else
							if( demo_mode == FADE )
							{
								fade_rate = 255-byte;
							}
							else
*/
							{
								animation_speed = pgm_read_byte(&gamma_tbl[255-byte]);	// gamma gives good (better) slope for controls
							}
								state = IDLE;
								refresh = post_state;			// tell main loop to update the leds
						break;
				
						case CMD_INTENS:	// intensity controls, subtracted with floor at 0 at display time	
							red_intensity = blue_intensity = green_intensity = 255-byte;
							red_intensity = blue_intensity = green_intensity = pgm_read_byte(&gamma_tbl[255-byte]);
							state = IDLE;
							refresh = post_state;			// tell main loop to update the leds
						break;
				
						case CMD_PAT:						// Set pattern / demo mode
							demo_mode = byte;
							last_demo_mode = 0x99;
							state = IDLE;
							refresh = post_state;			// tell main loop to update the leds
						break;

						case CMD_MODE:						// Set operating mode Raw/LUT ...
							control = byte;					// set the display controller mode
							data_state = HI_NIB;			// ready to get data ( botton 8 bits )
							post_state = STO_MODE;
							refresh = post_state;			// tell main loop to update the leds
						break;
					
						case CMD_UPDATE:
							index = byte;					// index is byte
							count = index;					// count for one byte
//							pattern_len = count;			// save pattern length ( does not change )
							post_state = STO_GREEN;			// on complete byte, store green index
							data_state = HI_NIB;			// ready to get data
						break;
						
						case STO_DLIST:						// store the display list
 							dp[index++] = byte;				// save display list byte
							if( count != index )
							{
								data_state = HI_NIB;		// ready to get data
								post_state = STO_DLIST;
							}
							else
							{
								state = IDLE;				// reset state machine
								post_state = IDLE;
								refresh = !refresh;			// tell main loop to update the leds
							}
						break;
						
						case STO_GREEN:
							if( control && LUT_ENA )		// check for RAW / LUT mode
							{
								lut[GREEN][index] = byte;
							}
							else
							{
								pixbuf[GREEN][index] = byte;
							}
							data_state = HI_NIB;			// ready to get data
							post_state = STO_RED;
						break;
						
						case STO_RED:
							if( control && LUT_ENA )		// check for RAW / LUT mode
							{
	 							lut[RED][index] = byte;
							}
							else
							{
								pixbuf[RED][index] = byte;
							}
							data_state = HI_NIB;			// ready to get data
							post_state = STO_BLUE;
						break;
						
						case STO_BLUE:
							if( control && LUT_ENA )		// check for RAW / LUT mode
							{
								lut[BLUE][index] = byte;
							}
							else
							{
								pixbuf[BLUE][index] = byte;
							}
							if( count != index )
							{
								post_state = STO_GREEN;		// on complete byte, 
								data_state = HI_NIB;		// ready to get data
								index++;
							}
							else
							{
								
								refresh = post_state;		// tell main loop to update the leds
								state = IDLE;				// reset state machine
								post_state = IDLE;
							}
						break;
						
						case STO_MODE:
							control |= (byte << 8);			// get global control
							data_state = IDLE;
							post_state = IDLE;
							refresh = post_state;			// tell main loop to update the leds
						break;
						
						default:
//							index = 0;
//							count = byte;			// set byte count for color loading
//							pattern_len = count;	// save pattern length
//							post_state = STO_RED;	// on complete byte, store red at index
//							data_state = HI_NIB;	// ready to get data
						break;
					}
				}
			}
		}
	}
sei();
}

// DMA ------------------------------------------------------------------- 
/*	Timing requirements --------------------------------------------------
	T0H 0 code ,high voltage time 0.4us ±150ns 
	T1H 1 code ,high voltage time 0.8us ±150ns 
	T0L 0 code , low voltage time 0.85us ±150ns 
	T1L 1 code ,low voltage time 0.45us ±150ns 
	RES low voltage time Above 50us is Sequence/latch
*/
	
	void ws2812drv_init(void)
	{
		// Setup EDMA channel 0(+1)
		EDMA.CTRL = EDMA_ENABLE_bm | EDMA_CHMODE_STD02_gc | EDMA_DBUFMODE_DISABLE_gc | EDMA_PRIMODE_CH0123_gc;
		EDMA.CH0.CTRLB = EDMA_CH_INTLVL_OFF_gc;
		EDMA.CH0.ADDRCTRL = EDMA_CH_RELOAD_NONE_gc | EDMA_CH_DIR_INC_gc;
		EDMA.CH0.DESTADDRCTRL = EDMA_CH_RELOAD_NONE_gc | EDMA_CH_DESTDIR_FIXED_gc;
		EDMA.CH0.DESTADDR = (uint16_t)&USARTC0.DATA;
		EDMA.CH0.TRIGSRC = EDMA_CH_TRIGSRC_USARTC0_DRE_gc;

		// Setup port pins for TxD, XCK and LUT0OUT
		PORTC.PIN0CTRL = PORT_OPC_TOTEM_gc;                         // LUT0OUT (data to WS2812)
		PORTC.PIN1CTRL = PORT_OPC_TOTEM_gc | PORT_ISC_RISING_gc;    // XCK
		PORTC.PIN3CTRL = PORT_OPC_TOTEM_gc | PORT_ISC_LEVEL_gc;     // TxD
		PORTC.DIRSET = PIN0_bm | PIN1_bm | PIN3_bm;

		// Setup Event channel 0 to TxD (async)
		EVSYS.CH0MUX = EVSYS_CHMUX_PORTC_PIN3_gc;
		EVSYS.CH0CTRL = EVSYS_DIGFILT_1SAMPLE_gc;
		// Setup Event channel 6 to XCK rising edge
		EVSYS.CH6MUX = EVSYS_CHMUX_PORTC_PIN1_gc;
		EVSYS.CH6CTRL = EVSYS_DIGFILT_1SAMPLE_gc;

		// Setup USART in master SPI mode 1, MSB first
		USARTC0.BAUDCTRLA = 19;                                     // 800.000 baud (1250 ns @ 32 MHz)
		USARTC0.BAUDCTRLB = 0;
		USARTC0.CTRLA = USART_RXSINTLVL_OFF_gc | USART_RXCINTLVL_OFF_gc | USART_TXCINTLVL_OFF_gc;
		USARTC0.CTRLC = USART_CMODE_MSPI_gc | (1 << 1);             // UDORD=0 UCPHA=1
		USARTC0.CTRLD = USART_DECTYPE_DATA_gc | USART_LUTACT_OFF_gc | USART_PECACT_OFF_gc;
		USARTC0.CTRLB = USART_TXEN_bm;

		// Setup XCL BTC0 timer to 1shot pwm generation
		XCL.CTRLE = XCL_CMDSEL_NONE_gc | XCL_TCSEL_BTC0_gc | XCL_CLKSEL_DIV1_gc;
		XCL.CTRLF = XCL_CMDEN_DISABLE_gc | 0x03;                    // 0x03 : One-shot PWM (missing in iox32e5.h)
		XCL.CTRLG = XCL_EVACTEN_bm | (0x03<<3) | XCL_EVSRC_EVCH6_gc; // 0x03<<3 : EVACT0=RESTART (missing in iox32e5.h)
		XCL.PERCAPTL = 22;                                          // Output high time if data is 1 (from RESTART to falling edge of one-shot)
		XCL.CMPL = 13;                                              // Output high time if data is 0 (from RESTART to rising edge of one-shot)

		// Setup XCL LUT
		XCL.CTRLA = XCL_LUTOUTEN_PIN0_gc | XCL_PORTSEL_PC_gc | XCL_LUTCONF_MUX_gc;  // Setup glue logic for MUX
		XCL.CTRLB = 0x50;                                           // IN3SEL=XCL, IN2SEL=XCL, IN1SEL=EVSYS, IN0SEL=EVSYS (missing in iox32e5.h)
		XCL.CTRLC = XCL_EVASYSEL0_bm | XCL_DLYCONF_DISABLE_gc;      // Async inputs, no delay
		XCL.CTRLD = 0xA0;                                           // LUT truth tables (only LUT1 is used)
	}


	void ws2812drv_start_transfer(ws2812drv_led_t *p, uint16_t cnt)
	{
		EDMA.CH0.ADDR = (uint16_t)p;
		EDMA.CH0.TRFCNT = cnt;
		EDMA.CH0.CTRLA = EDMA_CH_ENABLE_bm | EDMA_CH_SINGLE_bm; // Start DMA transfer to LEDs
	}


	uint8_t ws2812drv_busy(void)
	{
		return (EDMA.STATUS & (EDMA_CH0BUSY_bm | EDMA_CH0PEND_bm)) ? 1 : 0;
	}

// Graphics and animation setup, Initialize color look-up table ----------------------------

/************************************************************************/
/*   populate the color look-up table                                   */
/************************************************************************/

void populate_LUT()
{
	uint8_t t = 15;
	// Set up the color look-up table to reduce memory use and select the best colors for the application
	for( uint16_t i = 0; i < 24; i++ )	// Red ( 24 colors )
	{
	// Red ---------------------
	lut[RED][i + RED_LUT] =	t;    //   Maroon  0x80,0x00,0x00,
	lut[GREEN][i + RED_LUT] = 0;
	lut[BLUE][i + RED_LUT] = 0;
	// Orange -------------------
	lut[RED][i + ORANGE_LUT] = t; 	    //   OrangeRed	0xFF,0x8C,0x00,    //   DarkOrange 	0xFF,0xA5,0x00,    //   Orange 0xFF,0x45,0x00,
	lut[GREEN][i + ORANGE_LUT] = t;
	lut[BLUE][i + ORANGE_LUT] = 0;
	// Yellow --------------------
	lut[RED][i + YELLOW_LUT] = t+50;	  //   Gold 0xFF,0xD7,0x00,  
	lut[GREEN][i + YELLOW_LUT] = t;
	lut[BLUE][i + YELLOW_LUT] = 0;
	// Green ----------------------
	lut[RED][i + GREEN_LUT] = 0;    //   MediumSpringGreen	0x00,0xFF,0x7F,    //   SpringGreen 0x00,0xFA,0x9A,
	lut[GREEN][i + GREEN_LUT] = t;  //   Green	0x00,0x64,0x00,    //   DarkGreen 	0x80,0x80,0x00,    //   Olive 	0x7F,0xFF,0x00,    //   Chartreuse	0x7C,0xFC,0x00,    //   LawnGreen 	0x00,0x80,0x00,  
	lut[BLUE][i + GREEN_LUT] = 0;
	// Blue green -----------------
	lut[RED][i + BLUEGREEN_LUT] = 0;
	lut[GREEN][i + BLUEGREEN_LUT] = t;
	lut[BLUE][i + BLUEGREEN_LUT] = t;
	// Blue -----------------------
	lut[RED][i + BLUE_LUT] = 0;   //   MediumBlue	0x00,0x00,0x8B,    //   DarkBlue	0x00,0x00,0x80,    //   Navy 0x00,0x00,0xCD, 
	lut[GREEN][i + BLUE_LUT] = 0;	    //   DeepSkyBlue 0x00,0xBF,0xFF,  //   DarkCyan 0x00,0x8B,0x8B, 
	lut[BLUE][i + BLUE_LUT] = t;	   //   DarkTurquoise  0x00,0xCE,0xD1, 
	// Violet ---------------------
	lut[RED][i + VIOLET_LUT] = t;
	lut[GREEN][i + VIOLET_LUT] = 0;
	lut[BLUE][i + VIOLET_LUT] = t ;

	t = t + 10;
	}
	
	for( uint16_t i = 0; i < 0x87; i++)
	{
		// Library colors -------------
		lut[RED][i + LIB_LUT] = pgm_read_byte(&lib_LUT[i][RED]);
		lut[GREEN][i + LIB_LUT] = pgm_read_byte(&lib_LUT[i][GREEN]);
		lut[BLUE][i + LIB_LUT] = pgm_read_byte(&lib_LUT[i][BLUE]);
	} 

	if( control & GAMMA )
	{
		// perform gamma correction on lut ------------
		for( uint16_t i = 0; i < N_COLORS; i++)
		{
			lut[RED][i] = pgm_read_byte(&gamma_tbl[lut[RED][i]]);
			lut[GREEN][i] = pgm_read_byte(&gamma_tbl[lut[GREEN][i]]);
			lut[BLUE][i] = pgm_read_byte(&gamma_tbl[lut[BLUE][i]]);
		}
	}
/*		test white LUT -----------------------------
	for( uint8_t i = 0; i < 255; i++ )
	{
		lut[i][RED] = 255;
		lut[i][GREEN] = 255;
		lut[i][BLUE] = 255;
	}
*/
//  Assure 0:0:0 is black
	lut[RED][0]		= 0;
	lut[GREEN][0]	= 0;
	lut[BLUE][0]	= 0;
	
// and FF/FF/FF is white
	lut[RED][N_COLORS-1]		= 255;
	lut[GREEN][N_COLORS-1]		= 255;
	lut[BLUE][N_COLORS-1]		= 255;
}

// ********************************************************************************
// Perform the Led refresh operation ----------------------------------------------
// ********************************************************************************

uint8_t  do_refresh( void )
{
uint8_t color = 0;					// color from selected buffer / processed -> LUT
int16_t red = 0;					// final output color(s) -> DMA -> UART -> XCL -> LEDS
int16_t green = 0;
int16_t blue = 0;
// uint8_t red_ratio = 0;			// Ratio calculations ( per pixel ) - Color intensity correction
// uint8_t green_ratio = 0;
// uint8_t blue_ratio = 0;
uint8_t collision = 0;				// local collision detection mask for special effects
uint8_t quench	;					// quench accumulator == 0 ? sparkle done. Set quenched
// static uint8_t mixer = 0;

// working vars for pixel movement to DMA buffers

uint16_t pix_count = 0;				// Pixel count, correct color and stuff to DMA
uint8_t dma_count;					// DMA buffer pixel index
uint8_t dma_buf_sel = 0;			// buffer selection

uint8_t	spark_plane = sparkle_ena & 0x03;	// get sparkle plane

	frame_count++;					// # frames displayed
	// fade control
// LED output loop - Perform color LUT, intensity control and collision detection
	quench = 0;
	do {	
		for( dma_count = 0; dma_count < N_DMA_PIXELS; dma_count++ )
		{
			
			if( control & PIX_PGM ) // process sprites
			{
				uint8_t dir = pixbuf[BUF3][pix_count] & DIRECTION ? 1 : -1;
				if(	MOVE & pixbuf[BUF3][pix_count] )
				{
					if((pix_count + dir) < N_LEDS )
					{
						pixbuf[BUF3][pix_count+dir] = pixbuf[BUF3][pix_count];
						pixbuf[BUF3][pix_count] = DECAY;
						pixbuf[BUF2][pix_count+dir] = pixbuf[BUF2][pix_count];
					}
				}
				if(	BOUNCE & pixbuf[BUF3][pix_count] )
				{
					uint8_t dir = pixbuf[BUF3][pix_count] & DIRECTION ? 1 : -1;
					if( (pix_count + dir) < N_LEDS )
					{
						if( pixbuf[BUF3][pix_count+dir] )
						{
							pixbuf[BUF3][pix_count] ^= DIRECTION;
						}
					}
					else
					{
						pixbuf[BUF3][pix_count+dir] = pixbuf[BUF3][pix_count];
						pixbuf[BUF3][pix_count] ^= DIRECTION;
					}
				}
				if(	EXPLODE & pixbuf[BUF3][pix_count] )
				{
				}
				if(	DECAY & pixbuf[BUF3][pix_count] )
				{
					pixbuf[BUF2][pix_count+dir] = pixbuf[BUF2][pix_count] - 1;
				}
				if(	SPAWN & pixbuf[BUF3][pix_count] )
				{
				}
				if(	SPEW & pixbuf[BUF3][pix_count] )
				{
				}
				if(	GLOBAL & pixbuf[BUF3][pix_count] )
				{
				}
			}
		// Mixer control and priority --------------------------
			if( control & MIX_SEL )
			{
				if( control & MIX_XOR )	// use XOR in mix operation
				{
					color = (buf_ena & 0x01 ? pixbuf[BUF3][pix_count] : 0) ^
							(buf_ena & 0x02 ? pixbuf[BUF2][pix_count] : 0) ^
							(buf_ena & 0x04 ? pixbuf[BUF1][pix_count] : 0) ^
							(buf_ena & 0x08 ? pixbuf[BUF0][pix_count] : 0);
				}
				if( control & MIX_OR )	// use BOOL OR in mix operation
				{
					color = (buf_ena & 0x01 ? pixbuf[BUF3][pix_count] : 0) |
							(buf_ena & 0x02 ? pixbuf[BUF2][pix_count] : 0) |
							(buf_ena & 0x04 ? pixbuf[BUF1][pix_count] : 0) |
							(buf_ena & 0x08 ? pixbuf[BUF0][pix_count] : 0);
				}
				if( control & MIX_ADD )	// use Arith ADD in mix operation
				{
					color = (buf_ena & 0x01 ? pixbuf[BUF3][pix_count] : 0) +
							(buf_ena & 0x02 ? pixbuf[BUF2][pix_count] : 0) +
							(buf_ena & 0x04 ? pixbuf[BUF1][pix_count] : 0) +
							(buf_ena & 0x08 ? pixbuf[BUF0][pix_count] : 0);
				}
				if( control & MIX_SEQ )	// use sequence ( 0, 1, 2, 3 ) in mix operation
				{
					/* use alternate
					if( buf_ena & 0x01 )
						color = pixbuf[BUF0][pix_count];
					else
						color = 0;
					for( uint8_t i = BUF1; i <= mixer; i++)
					{
						if( ( buf_ena & ( 1 << i ) ) )
							color = max( color, pixbuf[i][pix_count] );
					}
					*/
					// alternate mix - simple mix based on pix_count position
					// exclude non-enabled planes
					if( buf_ena & (1 << (pix_count & 0x03)))
						color = pixbuf[BUF0][pix_count & 0x03];
				}
			}
			else	// No mix - Use color priority or level priority
			{
				color = 0;
				if( control & PRIORITY )	// color priority
				{
					if( buf_ena & 0x01 )
						color = pixbuf[BUF0][pix_count];
					else
						color = 0;
					for( uint8_t i = BUF1; i <= BUF3; i++)
					{
						if( buf_ena & ( 1 << i ) )
							color = max( color, pixbuf[i][pix_count] );
					}
				}
				else
				{	// level priority
					if(  buf_ena & 0x08 )
					{
						color = pixbuf[BUF3][pix_count];
					}
					if(  buf_ena & 0x04 )
					{
						color =  pixbuf[BUF2][pix_count];
					}
					if(  buf_ena & 0x02 )
					{
						color = pixbuf[BUF1][pix_count];
					}
					if(  buf_ena & 0x01 )
					{
						color =  pixbuf[BUF0][pix_count];
					}
				}
			}
			// check if tie color to position
			if( control & TIE_POS )	// tie color to position 
			{
				color = pix_count - color;
			}
			// twinkle the leds, use plane 3 for decay count ( depricated )
			if( control & TWINKLE )
			{
				// on time
				if( color != 0 )
				{
//					if( ( rnd() & 0xff ) < twinkle_rate )
					if( ( rnd() & 0xff ) < animation_speed )
					{
						color = 0;
					}
				}
			}			
			// process sparkle 
			if( ( control & SPARK_ENA ) && (speed == 0) )
			{
				if( pixbuf[spark_plane][pix_count] > 0 )
				{
					quench |=  1;
					if( rnd() < sparkle_density )
					{
						if( rnd() < sparkle_decay )
							pixbuf[spark_plane][pix_count]--;
						
						if( sparkle_color == 0 )
							red = green = blue = 0;	
						else
						{
							red = lut[RED][color] ? pixbuf[spark_plane][pix_count] - lut[RED][color] : 0;
							green = lut[GREEN][color] ? pixbuf[spark_plane][pix_count] - lut[GREEN][color] : 0;
							blue = lut[BLUE][color] ? pixbuf[spark_plane][pix_count] - lut[BLUE][color] : 0;
						}
					}
					else
					{
						red = lut[RED][color];
						green = lut[GREEN][color];
						blue = lut[BLUE][color];
					}
				}
			}
			else
			{
				quench =  1;
				red = lut[RED][color];
				green = lut[GREEN][color];
				blue = lut[BLUE][color];
			}
			
// All RGB below this, Not color --------------------------------------------------------
			
			if( control & GAMMA )		// perform gamma correction if enabled
			{
				red = pgm_read_byte(&gamma_tbl[red]);
				green = pgm_read_byte(&gamma_tbl[green]);
				blue = pgm_read_byte(&gamma_tbl[blue]);
			}

			
			// apply intensity correction to overlay bit-planes with floor at zero..
			// we should subtract a weighted ratio between colors, 
			// so no going over the programmed intensity, Global can be used and offset with intensity
			if( red | green | blue )
			{	
				dma_buf[dma_buf_sel][dma_count][DMA_RED] = min( max(red - red_intensity - fade, 0), 0xff); 
				dma_buf[dma_buf_sel][dma_count][DMA_GREEN] = min( max( green - green_intensity - fade, 0), 0xff);
				dma_buf[dma_buf_sel][dma_count][DMA_BLUE] = min( max( blue - blue_intensity - fade, 0), 0xff ); 
			}
			else
			{
				dma_buf[dma_buf_sel][dma_count][DMA_RED] = 0;
				dma_buf[dma_buf_sel][dma_count][DMA_GREEN] = 0;
				dma_buf[dma_buf_sel][dma_count][DMA_BLUE] = 0;
			}
	
			pix_count++;		// 
			
			// detect collisions ----------------------------------------
			collision = 0;
			if( collision_ena )
			{
				collision |=
				((pixbuf[BUF3][pix_count] != 0) & collision_ena & 0x03) |
				((pixbuf[BUF2][pix_count] != 0) & collision_ena >> 2 & 0x0c) |
				((pixbuf[BUF1][pix_count] != 0) & collision_ena >> 4 & 0x30) |
				((pixbuf[BUF0][pix_count] != 0) & collision_ena >> 6 & 0xc0);
			}
		}
		
		while(ws2812drv_busy());	// wait for DMA
		ws2812drv_start_transfer( (ws2812drv_led_t *) &dma_buf[dma_buf_sel][0][0], N_DMA_BYTES);

		// swap DMA buffers
		dma_buf_sel = dma_buf_sel ? 0 : 1;

		// Done with DMA transfer?, start pixel processing ----------------------------------------
	}	while( pix_count < N_LEDS );
	// Check for sparkle quenched - set global flag
	if( (quench == 0) && sparkle_ena )
	{
		sparkle_ena = 0;
		quenched = 1;
	}
	return ( ( control & COLL_ENA ) ? collision : 0 ); // Globally enable / disable collision detection
}

// ********************************************************************************
// Perform the Led refresh operation RAW mode RGBI --------------------------------
// ********************************************************************************

uint8_t  do_raw_refresh(uint16_t start, uint16_t end)
{

// working vars for pixel movement to DMA buffers

uint16_t pix_count;					// Pixel count, stuff to DMA in raw mode
uint8_t dma_count;					// DMA buffer pixel index
uint8_t dma_buf_sel = 0;			// buffer selection
uint8_t red = 0;			// final output color(s) -> DMA -> UART -> XCL -> LEDS
uint8_t green = 0;
uint8_t blue = 0;

	pix_count = start;	
// LED output loop - RAW mode RGBI
	do {	
		for( dma_count = 0; dma_count < N_DMA_PIXELS; dma_count++ )
		{
			red = pixbuf[BUF0][pix_count] + pixbuf[BUF3][pix_count];
			green =	pixbuf[BUF1][pix_count] + pixbuf[BUF3][pix_count];
			blue = pixbuf[BUF2][pix_count] + pixbuf[BUF3][pix_count];
			
			// apply intensity correction to overlay bit-planes with floor at zero..
			// we should subtract a weighted ratio between colors, 
			// so no going over the programmed intensity, Global can be used and offset with intensity
					
			dma_buf[dma_buf_sel][dma_count][DMA_RED] = ((int16_t) red - red_intensity) > 0 ? red - red_intensity: 0;
			dma_buf[dma_buf_sel][dma_count][DMA_GREEN] = ((int16_t) green - green_intensity) > 0 ? green - green_intensity : 0;
			dma_buf[dma_buf_sel][dma_count][DMA_BLUE] = ((int16_t) blue - blue_intensity) > 0 ? blue - blue_intensity: 0;
			pix_count++;
		}
		
		while(ws2812drv_busy());	// wait for DMA
		ws2812drv_start_transfer( (ws2812drv_led_t *) &dma_buf[dma_buf_sel][0][0], N_DMA_BYTES);

		// swap DMA buffers
		dma_buf_sel = dma_buf_sel ? 0 : 1;

		// Done with DMA transfer?, start pixel processing ----------------------
	}	while( pix_count < end );
	
	return (0);		// no collision detection

}

// Testing ----------------------------------------------------------------------
/*  Paint engine programming --------
	Use: PAINT_SEQUENCE as opr
	available opa:
	SOLID		Paint solid
	ALTERNATE	Paint alternate
	UP_GRAD		Paint increasing linear shade gradient
	DN_GRAD		Paint decreasing linear shade gradient
	SIN_GRAD	Paint 1/2 wave sin(X) shade gradient
	RAND		Paint random of all colors
	RAND_SHADE	Paint random of selected shade 
*/

// REGRESSION TEST LIBRARY ---------------------------------------------

void build_canned_seq(void)
{
		//	ins(addr,opr,opa,bra,start,end,r0,r1,r2,c0,c1,c2 )
// Paint operations ------------------------------------

// Raster operations -----------------------------------

// Utility operations, set operating modes and enables

}

// exercise the system --------------------------------------------

void self_test_seq(void)
{
/*	
// bitplane manipulation - shifts, rotates ...

enum OPR {
	COMPLEX_NOP = 0,
	ROTATE_LEFT,	// rotate with wrap
	ROTATE_RIGHT,	
	SHIFT_LEFT,		// shift with edge color fill
	SHIFT_RIGHT,
	REVERSE_PAT,	// reverse pattern
	BLIT_LEFT,		// blit 
	BLIT_RIGHT,
	SWAP_LEDS,		// swap individual leds
	LOAD_DELAY,		// load delay register for delay operation
	LOAD_COUNT,		// load count register
	SPARKLE,		// setup for sparkle function
	CANNED_EFFECTS,	// not yet defined
	
	// Paint operations ------------------------------------
	SOLID,			// solid color
	ALTERNATE,		// alternate color
	UP_GRAD,		// increasing gradient ramp of single color
	DN_GRAD,		// Decreasing gradient ramp of single color
	SIN_GRAD,		// Sin(x) gradient
	RAND,			// random colors ( all )
	RAND_SHADE,		// random shades of a selected color
	RAINBOW,		// raindow pattern	

	// Utility operations, set operating modes and enables
	CONTROL,		// control flags
	COLLISION_ENA,	// collision enables
	BUF_ENA,		// enable display buffer ( for display )
	COUNT,			// cycle count
	DELAY,			// sequence delay for programming
	ANI_SPEED,		// animation speed
	DEMO_MODE,		// reload display list with new demo mode
	RED_INTENS,		// global intensity controls ( RGB )
	GREEN_INTENS,
	BLUE_INTENS,
	
	// Paint operation tests ------------------------------------
	SOLID,			// solid color
	ALTERNATE,		// alternate color
	UP_GRAD,		// increasing gradient ramp of single color
	DN_GRAD,		// Decreasing gradient ramp of single color
	SIN_GRAD,		// Sin(x) gradient
	RAND,			// random colors ( all )
	RAND_SHADE,		// random shades of a selected color
	RAINBOW,		// raindow pattern
*/

	
/*
Display list testing -------------------------------------

			ins( 0, DELAY, NOP, NOP, 0, 0xff, 0, 0, 0, 0, 0, 0)	
			ins( 1, SOLID, RED_MID, NOP, 0, N_LEDS, 0, 0, 0, 0, 0, 0)
			ins( 3, NOP, NOP, HALT, 2, 0, 0, 0, 0, 0, 0, 0)	
			ins( 2, NOP, NOP, BDLY, 1, 0, 0, 0, 0, 0, 0, 0)

			ins( 2, ALTERNATE, NOP, NOP, 0, N_LEDS, 0, 0, 0, 0, 0, 0)	
			ins( 3, UP_GRAD, NOP, NOP, 0, N_LEDS, 0, 0, 0, 0, 0, 0)	
			ins( 4, DN_GRAD, NOP, NOP, 0, N_LEDS, 0, 0, 0, 0, 0, 0)	
			ins( 5, SIN_GRAD, NOP, NOP, 0, N_LEDS, 0, 0, 0, 0, 0, 0)	
			ins( 6, RAND, NOP, NOP, 0, N_LEDS, 0, 0, 0, 0, 0, 0)	
			ins( 7, RAND_SHADE, NOP, NOP, 0, N_LEDS, 0, 0, 0, 0, 0, 0)	
			ins( 8, RAINBOW, NOP, NOP, 0, N_LEDS, 0, 0, 0, 0, 0, 0)	
			ins( 9, NOP, NOP, BR, 40, 0, 0, 0, 0, 0, 0, 0)	

// Raster operations -----------------------------------

			ins( 10, ROTATE_LEFT, NOP, NOP, 0, N_LEDS, 0, 0, 0, 0, 0, 0)	
			ins( 11, ROTATE_RIGHT, NOP, NOP, 0, N_LEDS, 0, 0, 0, 0, 0, 0)
			ins( 12, SHIFT_LEFT, NOP, NOP, 0, N_LEDS, 0, 0, 0, 0, 0, 0)
			ins( 13, SHIFT_RIGHT, NOP, NOP, 0, N_LEDS, 0, 0, 0, 0, 0, 0)
			ins( 14, REVERSE_PAT, NOP, NOP, 0, N_LEDS, 0, 0, 0, 0, 0, 0)
			ins( 15, BLIT_LEFT, NOP, NOP, 0, N_LEDS, 0, 0, 0, 0, 0, 0)
			ins( 16, BLIT_RIGHT, NOP, NOP, 0, N_LEDS, 0, 0, 0, 0, 0, 0)
			ins( 17, SWAP_LEDS, NOP, NOP, 0, N_LEDS, 0, 0, 0, 0, 0, 0)
			ins( 18, LOAD_DELAY, NOP, NOP, 0, N_LEDS, 0, 0, 0, 0, 0, 0)
			ins( 19, LOAD_COUNT, NOP, NOP, 0, N_LEDS, 0, 0, 0, 0, 0, 0)
			ins( 20, SPARKLE, NOP, NOP, 0, N_LEDS, 0, 0, 0, 0, 0, 0)

// Utility operations, set operating modes and enables
			ins( 21, CONTROL, 0, NOP ,0, 0, 0, 0, 0, 0, 0, 0 )
			ins( 22, COLLISION_ENA, 0, NOP ,0, 0, 0, 0, 0, 0, 0, 0 )
			ins( 23, BUF_ENA, 0, NOP ,0, 0, 0, 0, 0, 0, 0, 0 )
			ins( 24, COUNT, 0, NOP ,0, 0, 0, 0, 0, 0, 0, 0 )
			ins( 25, DELAY, 0, NOP ,0, 0, 0, 0, 0, 0, 0, 0 )
			ins( 26, ANI_SPEED, 0, NOP ,0, 0, 0, 0, 0, 0, 0, 0 )
			ins( 27, DEMO_MODE, 0, NOP ,0, 0, 0, 0, 0, 0, 0, 0 )
			ins( 28, RED_INTENS, 0, NOP ,0, 0, 0, 0, 0, 0, 0, 0 )
			ins( 29, GREEN_INTENS, 0, NOP ,0, 0, 0, 0, 0, 0, 0, 0 )
			ins( 30, BLUE_INTENS, 0, NOP ,0, 0, 0, 0, 0, 0, 0, 0 )
			ins( 40, NOP, NOP, HALT, 0, 0, 0, 0, 0, 0, 0, 0)
*/
// TESTING ----------------------------------------------------------------------------------------
	
	// Instruction format:
	//	ins(addr,opr,opa,bra,start,end,r0,r1,r2,c0,c1,c2 )
	//	Note: Bool "OR" | branch and opcodes Ex: HALT|SOLID
	
	ins( 0, RAINBOW, NOP, 500, N_LEDS-1 , 0, 0, 0, 0, 0, 0, 0)	
	ins( 1, RAINBOW, NOP, 0, 100, N_LEDS-300, 1, 0, 0, 0, 0, 0)	
	ins( 2, RAND_SHADE, NOP, 0, 300, N_LEDS-400, 2, 0, 0, 0, 0, 0)	
	ins( 3, ROTATE, NOP, 0, N_LEDS-91, 0, 0, 0, 0, 0, 0, 0)
	ins( 4, ROTATE, NOP, 0, 330, N_LEDS-111, 1, 0, 0, 0, 0, 0)
	ins( 5, ROTATE | BR, NOP, 3, 0, 399, 2, 0, 0, 0, 0, 0)
//	ins( 6, SET_SPARKLE, 0xffff, 3, 0, N_LEDS, 0, 3, 0, 0, 0, 0)
	ins( 6, BR, NOP, 2, 0, N_LEDS, 3, 0, 0, 0, 0, 0)
		
}

// ******************************************************************************
// Main animation and refresh loop ----------------------------------------------
// ******************************************************************************

void main(void)
{
uint16_t			n_leds = N_LEDS-1;		// Number of active LEDs - testing
uint8_t				collision = 0;			// bit plane collision register
static uint8_t		pc = 0;					// Display list program counter 
uint8_t				tmp_pc;
uint8_t				ras;
uint8_t				fade_dir = 0;


#ifdef CANARY
	pixbuf[0][N_LEDS] = 0xDE;		// deadbeef the fucker ( DMA debug )
	pixbuf[1][N_LEDS] = 0xAD;
	pixbuf[2][N_LEDS] = 0xBE;
	pixbuf[3][N_LEDS] = 0xEF;
#endif

	// cli();/* disable interrupts */
	initialize_clocks();
	initialize_gpio();
	initialize_timer();
	InitUsartD0();
	ws2812drv_init();
	populate_LUT();
	clear_all_led_buf();
	// build_canned_seq();
//	self_test_seq();
//	demo_mode = RANDOM;
	demo_mode = NONE;
//	demo_mode = BLINK;
//	demo_mode = CHASE;
//	demo_mode = FADE;
//	demo_mode = STROBE;
//	demo_mode = TWINKLE;
//	demo_mode = RANDOM;
	populate_demo( demo_mode );
	do 
	{
/*		
#ifdef CANARY	// DMA debug - Camp here when the canary dies
	while(pixbuf[0][N_LEDS] != 0xDE);		// deadbeef the fucker ( DMA debug )
	while(pixbuf[1][N_LEDS] != 0xAD);
	while(pixbuf[2][N_LEDS] != 0xBE);
	while(pixbuf[3][N_LEDS] != 0xEF);
#endif
*/	
		// check for demo mode, color change, all network messages
			switch( refresh )
			{
				case CMD_COLOR:
				break;

				case CMD_DLIST:
				break;
				
				case CMD_SPEED:
					// check for speed update
					if( speed > animation_speed)
						speed = animation_speed;
				break;
				
				case CMD_INTENS:	// intensity controls, subtracted with floor at 0 at display time	
				break;
				
				case CMD_PAT:		// Set pattern / demo mode
					// tell display list processor time to switch to new program
					if( last_demo_mode != demo_mode)
						reset_pgm = populate_demo(demo_mode);
				break;

				case CMD_MODE:				// Set operating mode Raw/LUT ...
				break;
						
				case STO_DLIST:				// store the display list
				break;
						
				case STO_BLUE:
					// check for led update from network
					if( pattern_len != 0 )		// Move to refresh???
					{
						for ( uint16_t i = 0; i <= (LEDS - pattern_len ); i++ )
						{
							pixbuf[0][i] = ( i % pattern_len)+1;		// only plane 0 this version
						}
						// fill in the hole at end of pixel buffer
						for ( uint16_t i = LEDS - pattern_len; i <= LEDS ; i++ )
						{
							pixbuf[0][i] = ( i % pattern_len)+1;		// only plane 0 this version
						}
					}
				break;
						
				case STO_MODE:
				break;
						
				default:
				break;
			}
			busy = 0;	// re-enable refresh, LUT and pixbuf populated
			refresh = 99;
			
// Do programmed dlist operation ---------------------------------------

// decode and execute dlist instructions. Perform branch, loop and collision instructions
// 
		if( (control & DLIST_ENA) && !busy)		// Perform display list operation
		{
			// execute this display list until halt instruction
			if( speed++ >= animation_speed )
			{
				speed = 0;
				do 
				{
					uint8_t	pc_branch_taken = s[pc].br;
					tmp_pc = pc;	// this instruction
					uint8_t next_pc = pc+1;
					uint8_t	buf0 = (s[pc].start & BUFMASK0) >> 10;		// control and select planes (0-2)

					if( speed > animation_speed )
						speed = 0;

					switch(s[pc].opr & 0xe0 )	// remove low opcode bits, isolate branch instructions
					{
						case NOP:			// no branch
							ras = do_animation( pc );
						break;
		
						case BR:			// Unconditional branch
							ras = do_animation( pc );
							next_pc = pc_branch_taken;
						break;
						
						case BRCOL:			// Branch on collision
							if( collision )
							{
								next_pc = pc_branch_taken;
							}
							else
							{
								ras = do_animation( pc );
							}

						break;
		
						case BRCNT:			// branch on count == 0
							ras = do_animation( pc );
							if(--count[buf0] == 0)
							{
								next_pc = pc_branch_taken;
							}

						break;
		
						case BRDLY:			// branch on delay != 0
							if(--delay[buf0])
							{
								next_pc = pc_branch_taken;	// branch until timer == 0
							}
							else
							{
								ras = do_animation( pc );
							}
						break;

						
						case HALT:			// HALT display list operation and branch (refresh)
							ras = do_animation( pc );
							next_pc = pc_branch_taken;
						break;
		
						default:
						break;
					} 
					if( reset_pgm )			// have new display list command ( other resets???)
					{
						collision_ena = 0x00;							// Collision detection disabled for each plane
						buf_ena = 0x01;									// buffer enable / display = 1 / don't display = 0
//						count[] = { 0, 0, 0, 0};						// cycle count for programming
//						delay[] = {0, 0, 0, 0};							// sequence delay for programming
//						sparkle_ena = 3;								// sparkle enable 
//						sparkle_density = 80;							// density of sparkle pattern
//						sparkle_decay = 02;								// decay rate of sparkle pattern
//						quenched = 0;									// set when all sparkle == 0
						reset_pgm = 0;									// restart display list program
						pc = 0;					// Display list program counter 
					}
					else
					{
						pc = next_pc;
					}
				}	while( ( s[pc].opr & 0xe0 ) != HALT);
				
//				if( ( speed >= fade_rate ) && ( control & FADE_ENA ) )
				if( ( speed >= animation_speed ) && ( control & FADE_ENA ) )
				{
					fade += fade_dir ? -8: 8;
					if( (fade > 0xc0) || ( fade < 0x08) )
						fade_dir = !fade_dir;
				}
			}
		}
		// --------------------------------------------------------------------------
		// Perform display refresh, collision, quench and other operations performed
		// on next display list run, Selects between normal LUT and RAW modes.
		//
		
		// Normal refresh mode 
		if( control & LUT_ENA )					// global control bits  ( LUT or raw RGBI mode )
		{
			/* Send the pixels with EDMA, timer3/4 eventsys USART (syncro) and XCL */
			collision = do_refresh();
		}
		else
		{
		// RAW - use bit planes in R G B I mode, No LUT */
			collision = do_raw_refresh( 0, N_LEDS-1);
		}

		// wait for DMA to complete, Sync with the last DMA transfer
		while(ws2812drv_busy());
			
		// do the refresh operation - Light up the leds 
		for( volatile uint16_t i = 0; i < 3000; i++ );
	} while(1);
}

/*********************************************************
 * Graphics functions for animation
 ********************************************************/

// Bit map programmed integrated raster op, runs display list, paints patterns

uint8_t do_animation( uint8_t pc )
{
	// extract all the control vars / constants from the dlist instruction
	uint8_t	sbuf0 = (s[pc].start & BUFMASK0) >> 10;
	uint8_t	sbuf1 = (s[pc].start & BUFMASK1) >> 12;
	uint8_t	sbuf2 = (s[pc].start & BUFMASK2) >> 14;
	
	uint8_t	ebuf0 = (s[pc].end & BUFMASK0) >> 10;
	uint8_t	ebuf1 = (s[pc].end & BUFMASK1) >> 12;
	uint8_t	ebuf2 = (s[pc].end & BUFMASK2) >> 14;

	uint16_t scount = (s[pc].start & BUFMASK) >> 10;
	uint16_t ecount = (s[pc].end & BUFMASK) >> 10;
//	uint8_t colr = (( scount >> 2 ) & 0xc0 ) | ecount;
	uint8_t opr = s[pc].opr & 0x1f;				// operator - strip off branch instruction field
	uint16_t opa = s[pc].opa;					// operand
//	uint8_t bra = s->br;						// branch address or extended opa data
	
	int16_t start = s[pc].start & 0x3ff;		// remove buffer selector / opa2
	int16_t end = s[pc].end & 0x3ff;			// remove control bits / opa2
	// uint8_t tmp;

	if(s[pc].opr != NOP)
	{
	switch ( opr )		// 
		{
		case NOP:
		break;
		
// Paint operations -----------------------------------------------
	
		case SOLID:
			for( int16_t i=start; i != end; i++)
			{	
				pixbuf[sbuf0][i] = opa;
			}
		break;
		
		case ALTERNATE:
			for( int16_t i = start, j = max(start,end);
				 i != end + dir( start, end ); 
				 i = i + dir( start, end ))
			{	
				do {
					pixbuf[sbuf0][i] = opa;
				} while( j-- );
			}

		break;
		
		case GRADIENT:
			for( int16_t i = start, j=max(start,end);
			 i != end + dir( start, end ); i = i + dir( start, end ))
			{	
				do {
					pixbuf[sbuf0][i+j-ebuf0+1] = pgm_read_byte(&LUT_INDEX[opa]) + ( ( i / (ebuf0+1) ) % 24);
				} while( j-- );
			}
		break;
		
		
		case SIN_GRAD:
 				for( int16_t i = start, j = max(start,end);
				 i != end + dir( start, end ); i = i + dir( start, end ))
				{	
				do {
					pixbuf[sbuf0][i+j-ebuf0+1] = 
					((int8_t) pgm_read_byte(&LUT_INDEX[opa]) + (int8_t)(pgm_read_byte(&sin_tbl[j % 48])) - 24);
				} while( j-- );
			}
		break;
		
		case RAND_SHADE:
			for( int16_t i = start; 
			i != end + dir( start, end ); i = i + dir( start, end ))
			{	 
				pixbuf[sbuf0][i] = pgm_read_byte(&LUT_INDEX[opa & 0x07]) + (rnd() % 24);
			}
		break;
		
		case RAND:
			for( int16_t i = start; i != end; i++)
			{ 
				pixbuf[sbuf0][i] = rnd() % (pattern_len+1);
//				pixbuf[sbuf1][i] = rnd();
//				pixbuf[sbuf2][i] = rnd();
//				pixbuf[ebuf0][i] = rnd();
			}
		break;
		
		case RAINBOW:
			for( int16_t i = start; i != (end + dir( start, end )); i = i + dir( start, end ))
			{
				pixbuf[sbuf0][i] = i;
			}
		break;
		
		case RAMP_INTENSITY:
			for( int16_t i = start; i != end + dir( start, end ); i = i + dir( start, end ))
			{
				pixbuf[sbuf0][i] = ((uint16_t)pixbuf[sbuf1][i] - opr) ? (pixbuf[sbuf2][i] - opr) : 0;
			}
		break;
		
// rotate, blit and motion functions --------------------------------
					
		case ROTATE:
			rotate(start, end, sbuf0);
			break;	
			
		case SHIFT:
			shift(start, end, sbuf0, opa);
			break;
			
		case REVERSE_PAT:
			reverse_pattern(start, end, sbuf0);
			break;
			
		case BLIT:
			blit(start, end, count, sbuf0, sbuf1 );
			break;
			
		case SWAP:
			swap_leds(start, end, count, sbuf0, sbuf1);
			break;
			
		// sparkle uses a buffer to handle decay / intensity of the sparkle
		// sparkle is enabled here and handled in the refresh to eliminate
		// using extra planes for temporary storage, 
		// this also allows the application to drag the sparkle with rotate and shift
		// the sparkle plane is disabled ( for display ) and the
		// selected planes to sparkle are enabled. 
		// On "quench==0" ( decay ) the sparkle is disabled for that pixel.
		// The "quenched" variable is set when all sparkle counts == 0
		// The "quenched" var is used by the branch control to change 
		// the display list execution flow (usually)
		// The opa gives the gradient increment across the start to end.
		// this allows for the generation of trailing sparklers
		// an opa == 0 gives a (0xff) linear distribution.
				
		case SPARKLE:
			for( int16_t i = start, j = 0xff; i != end; i = i + dir( start, end ))
			{
				if( opa )
				{
					pixbuf[sbuf0][i] = j;		// distance inverse intensity
					j = ( j < opa ) ? 0 : j - opa;
				}
				else
				{
					pixbuf[sbuf0][i] = 0xff;					// set to max if zero ( linear )
				}
			}
			sparkle_ena = sbuf0 | 0x80;							// Plane to sparkle
			buf_ena = buf_ena & ( 0x0f ^ (1 << sbuf1) );	// turn off display on sparkle plane
			break;
			
		case CANNED_EFFECTS:		// do the game - ROCKETS
			{
			}
			break;
// Register / operation control flags, enables and variable loads			
			
		case CONTROL:				// control flags
			control = opa;			// global control bits
		break;
				
		case COLLISION_ENA:				// collision enables
			collision_ena = start; 	// Collision detection enables for each plane
		break;
				
		case BUF_ENA:					// enable display buffer ( for display )
			buf_ena = opa & 0x0f;	// buffer enable / / display = 1 / don't display = 0
		break;
		
		case ANI_SPEED:					// global animation speed
			animation_speed = opa;
		break;

		case DEMO_MODE:					// set demo mode		
			demo_mode = opa;			
		break;

		// intensity controls, subtracted with floor at 0 at display time
		case RGB_INTENS:				// red global intensity
			red_intensity = opa;	
		// blue global intensity
			blue_intensity = start;
		// green global intensity
			green_intensity = end;
		break;
			
		case LOAD_COUNT:	// Load sequence count
			count[sbuf0] = opa;	// use end for full 16 bits.
			break;
		
		case LOAD_DELAY:	// Load sequence delay
			delay[sbuf0] = opa;
			break;

		case SET_SPARKLE:	
			sparkle_color = opa;					// global color for sparkle ( cound assign another plane??? ) 2 bytes ( rand select )
			sparkle_density = start;				// set density
			sparkle_decay = end;					// set sparkle decay
			break;

		
		default:
			break;
		}
	}
	if( ( opa != 0 ) && ( opr == NOP ) )
	{
		for( uint16_t i = start; i <= end; i++)
		{
			if( opa & RASOP_AND )  
				pixbuf[ebuf0][i] = pixbuf[ebuf1][i] & pixbuf[ebuf2][i];
			if( opa & RASOP_OR )
				pixbuf[ebuf0][i] = pixbuf[ebuf1][i] | pixbuf[ebuf2][i];
			if( opa & RASOP_XOR )
				pixbuf[ebuf0][i] = pixbuf[ebuf1][i] ^ pixbuf[ebuf2][i];
			if( opa & RASOP_ADD && ( (uint16_t)(pixbuf[ebuf1][i] + pixbuf[ebuf2][i] < 256 ) ) ) 
				pixbuf[ebuf0][i] = pixbuf[ebuf1][i] + (int16_t)pixbuf[ebuf2][i];
			if( opa & RASOP_SUB && (int16_t)(pixbuf[ebuf1][i] - pixbuf[ebuf2][i] > 0 ) )
				pixbuf[ebuf0][i] = pixbuf[ebuf1][i] - (int16_t) pixbuf[ebuf2][i];
			if( opa & RASOP_DECREMENT && (pixbuf[ebuf0][i] > 0) )
				pixbuf[ebuf0][i] = --pixbuf[ebuf1][i];
			if( opa & RASOP_INCREMENT && ( pixbuf[ebuf0][i] < 255 ) )
				pixbuf[ebuf0][i] = ++pixbuf[ebuf1][i];
			if( opa & RASOP_USER )
			{
				if( pixbuf[ebuf0][i] > pixbuf[ebuf2][i] )
				{
					  pixbuf[ebuf0][i]--;
					  pixbuf[ebuf1][i]++;
					  
				}
				else
				{
					  pixbuf[ebuf0][i]++;
					  pixbuf[ebuf1][i]--;
					  
				}
			}
		}
	}
	return 0;		// for now
}
void clear_led_buf(uint8_t buf)
{
	for( uint16_t i = 0; i < N_LEDS; i++)
	{
		pixbuf[buf][i] = 0;
	}
}

void clear_all_led_buf(void)
{
	for( uint16_t i = 0; i < N_LEDS; i++)
	{
		pixbuf[0][i] = 0;
		pixbuf[1][i] = 0;
		pixbuf[2][i] = 0;
		pixbuf[3][i] = 0;
	}
}

// Rotate with wrap: Move left if start < end, Right if start > end
void rotate(uint16_t start, uint16_t end, uint8_t buf )
{
	// Rotate left
	if( start < end )
	{
		uint8_t tmp = pixbuf[buf][start];
		for( uint16_t i = start; i < end; i++)	
		{
			pixbuf[buf][i] = pixbuf[buf][i+1];
		}
		pixbuf[buf][end] = tmp;
	}
	else
	{
	// Rotate right with wrap
		uint8_t tmp = pixbuf[buf][start];
		for( uint16_t i = start; i > end; i--)	
		{
			pixbuf[buf][i] = pixbuf[buf][i-1];
		}
		pixbuf[buf][end] = tmp;
	}
}


// blit the leds
void blit(uint16_t source, uint16_t dest, uint16_t count, uint8_t buf1, uint8_t buf2 )
{
	if( source < dest )
	{
		for( uint16_t i = 0; i < count; i++)
		{
			pixbuf[buf1][dest++] = pixbuf[buf2][source++];
		}
	}
	else
	{	// blit the leds right
		source += count;
		dest += count;
		for( uint16_t i = 0; i < count; i++ )
		{
			pixbuf[buf1][dest--] = pixbuf[buf2][source--];
		}
	}
}

// rotate look-up table
void rotate_lut(uint8_t start, uint8_t end )
{
	uint16_t tmpr, tmpg, tmpb;
	tmpr = lut[RED][start];
	tmpg = lut[GREEN][start];
	tmpb = lut[BLUE][start];

	if( start < end )
	{
		for( uint16_t i = start; i < end; i++)
		{
			lut[RED][i] = lut[RED][i+1];
			lut[GREEN][i] = lut[GREEN][i+1];
			lut[BLUE][i] = lut[BLUE][i+1];
		}
	}
	else
	{
		for( uint16_t i = start; i > end; i--)
		{
			lut[RED][i] = lut[RED][i];
			lut[GREEN][i] = lut[GREEN][i];
			lut[BLUE][i] = lut[BLUE][i];
		}
	}
	lut[RED][end] = tmpr;
	lut[GREEN][end] = tmpg;
	lut[BLUE][end] = tmpb;
}

// Shift with color fill
void shift(uint16_t start, uint16_t end, uint8_t buf, uint8_t color)
{
	if( start < end )	// shift left
	{
		for( uint16_t i = start; i < end-1; i++)
		{
			pixbuf[buf][i] = pixbuf[buf][i+1];
		}
		pixbuf[buf][end-1]= color;
	}
	else
	{
	// Shift right with color fill
		for( uint16_t i = end-1; i > start; i--)
		{
			pixbuf[buf][i] = pixbuf[buf][i-1];
		}
		pixbuf[buf][start] = color;
	}
}

// Reverse the order of the pattern in the buffer

void reverse_pattern(uint16_t start, uint16_t end, uint8_t buf)
{
	uint16_t tmp;
	for( uint16_t i = start; i < end-1; i++)
	{
		tmp = pixbuf[buf][i];
		pixbuf[buf][i] = pixbuf[buf][end-i];
		pixbuf[buf][end-i] = tmp;
	}
}

void swap_leds(uint16_t start1, uint16_t start2, uint16_t count, uint8_t buf1, uint8_t buf2)
{
	for( uint16_t i = 0; i < count; i++)
	{	// use the xor swap trick
		pixbuf[buf1][i+start1] ^= pixbuf[buf2][i+start2];
		pixbuf[buf2][i+start2] ^= pixbuf[buf1][i+start1];
		pixbuf[buf1][i+start1] ^= pixbuf[buf2][i+start2];
	}
}

// populate plane 0 with the selected demo bitmap

uint8_t populate_demo( uint8_t demo_mode )
{	// define ins(addr,opra,opaa,braddr,starta,enda,p0,p1,p2,p3,p4,p5 )
static uint8_t chase_dir = 0;
static uint8_t blink_ena = 0;

	switch( demo_mode )
	{
		case NONE:
			ins( 0, LOAD_DELAY, 0xfff, 0, 0, 0, 3, 0, 0, 0, 0, 0)
			ins( 1, HALT, NOP, 2, 0, 0, 0, 0, 0, 0, 0, 0)
			ins( 2, BRDLY, NOP, 1, 0, 0, 3, 0, 0, 0, 0, 0)
			ins( 3, BR, NOP, 0, 0, 0, 0, 0, 0, 0, 0, 0)
	
			sparkle_ena = 0;
			blink_ena = 0;
			control = DLIST_ENA | LUT_ENA;
			fade = 0;		
			
/*			if( sparkle_ena )
			{
				sparkle_ena = 0;
				ins( 0, HALT, NOP, 0, LEDS, 0, 3, 0, 0, 0, 0, 0)
			}
			else
			{
				ins( 0, RAINBOW, NOP, 0, 0, LEDS , 0, 0, 0, 0, 0, 0)	
				ins( 1, SET_SPARKLE, 0x38, 0, 0x40, 0x0, 3, 0, 0, 0, 0, 0)	
				ins( 2, SPARKLE, NOP, 4, 0, LEDS, 3, 3, 0, 0, 0, 0)
				ins( 3, ROTATE, NOP, 0, 0, LEDS, 0, 0, 0, 0, 0, 0)
				ins( 4, HALT, NOP, 3, LEDS, 0, 3, 0, 0, 0, 0, 0)
			}
*/
			break;
			
		case CHASE:
				chase_dir = ( chase_dir == 1 ) ? 2 : 1;		
				switch(chase_dir) 
				{
					case 1:
						ins( 0, ROTATE|HALT, NOP, 0 , 0, LEDS, 0, 0, 0, 0, 0, 0)
					break;				
				
					case 2:
						ins( 0, ROTATE|HALT, NOP, 0, LEDS, 0, 0, 0, 0, 0, 0, 0)
					break;
				
					default:
						ins( 0, LOAD_DELAY, 0xfff, 0, 0, 0, 3, 0, 0, 0, 0, 0)
						ins( 1, HALT, NOP, 2, 0, 0, 0, 0, 0, 0, 0, 0)
						ins( 2, BRDLY, NOP, 1, 0, 0, 3, 0, 0, 0, 0, 0)
						ins( 3, BR, NOP, 0, 0, 0, 0, 0, 0, 0, 0, 0)
					break;
				}
			
			break;
		
		case STROBE:
			blink_ena = 1;

			if( blink_ena)
			{
				ins( 0, BUF_ENA | HALT, 0x1, 1, 0, LEDS, 0, 0, 0, 0, 0, 0)			
				ins( 1, BUF_ENA | HALT, 0x0, 0, 0, LEDS, 0, 0, 0, 0, 0, 0)			
			}
			else
			{
				ins( 0, LOAD_DELAY, 0xfff, 0, 0, 0, 3, 0, 0, 0, 0, 0)
				ins( 1, HALT, NOP, 2, 0, 0, 0, 0, 0, 0, 0, 0)
				ins( 2, BRDLY, NOP, 1, 0, 0, 3, 0, 0, 0, 0, 0)
				ins( 3, BR, NOP, 0, 0, 0, 0, 0, 0, 0, 0, 0)
			}
			break;
			
/*
		case CHASE:
		chase_dir = ( chase_dir == 1 ) ? 2 : 1;		
		
		
		case BLINK:
			if( demo_mode == BLINK )
				blink_ena = !blink_ena;

			if(!blink_ena)
			{
				switch(chase_dir) 
				{
					case 1:
						ins( 0, ROTATE|HALT, NOP, 0 , 0, LEDS, 0, 0, 0, 0, 0, 0)
					break;				
				
					case 2:
						ins( 0, ROTATE|HALT, NOP, 0, LEDS, 0, 0, 0, 0, 0, 0, 0)
					break;
				
					default:
						ins( 0, LOAD_DELAY, 0xfff, 0, 0, 0, 3, 0, 0, 0, 0, 0)
						ins( 1, HALT, NOP, 2, 0, 0, 0, 0, 0, 0, 0, 0)
						ins( 2, BRDLY, NOP, 1, 0, 0, 3, 0, 0, 0, 0, 0)
						ins( 3, BR, NOP, 0, 0, 0, 0, 0, 0, 0, 0, 0)
					break;
				}
			}
			else
			{
				switch(chase_dir)		// chase and blink
				{
					case 1:
						ins( 0, ROTATE, NOP, 0 , 0, LEDS, 0, 0, 0, 0, 0, 0)
						ins( 1, BUF_ENA | HALT, 0x1, 2, 0, LEDS, 0, 0, 0, 0, 0, 0)			
						ins( 2, BUF_ENA | HALT, 0x0, 0, 0, LEDS, 0, 0, 0, 0, 0, 0)			
					break;				
			
					case 2:
						ins( 0, ROTATE, NOP, 0, LEDS, 0, 0, 0, 0, 0, 0, 0)
						ins( 1, BUF_ENA | HALT, 0x1, 2, 0, LEDS, 0, 0, 0, 0, 0, 0)			
						ins( 2, BUF_ENA | HALT, 0x0, 0, 0, LEDS, 0, 0, 0, 0, 0, 0)			
					break;
				
					case 0:
					default:
						ins( 0, BUF_ENA | HALT, 0x1, 1, 0, LEDS, 0, 0, 0, 0, 0, 0)			
						ins( 1, BUF_ENA | HALT, 0x0, 0, 0, LEDS, 0, 0, 0, 0, 0, 0)			
					break;
					}
				}
				break;
*/		
		
		case FADE:

			control ^= FADE_ENA;
			fade = 0x20;
//			ins( 0, HALT, NOP, 0, LEDS, 0, 2, 0, 0, 0, 0, 0)
/*
			ins( 0, CONTROL, DLIST_ENA, 0, 0, 0, 1, 0, 0, 0, 0, 0)
			if( pattern_len)
			{
				for( uint16_t i = 0; i < LEDS; i++ )
				{
					pixbuf[0][i] = lut[RED][i % pattern_len];
					pixbuf[1][i] = lut[GREEN][i % pattern_len];
					pixbuf[2][i] = lut[BLUE][i % pattern_len];
				}
				ins( 1, HALT, NOP, 4, 0, 0, 1, 0, 0, 0, 0, 0)
			}
			else
			{
				ins( 1, RAINBOW, NOP, 0, 0, LEDS, 0, 0, 0, 0, 0, 0)
				ins( 2, RAINBOW, NOP, 0, 0, LEDS, 1, 0, 0, 0, 0, 0)
				ins( 3, RAINBOW, NOP, 0, 0, LEDS, 2, 0, 0, 0, 0, 0)
			}
			ins( 4, ROTATE, NOP, 0, LEDS, 0, 0, 0, 0, 0, 0, 0)
			ins( 5, ROTATE, NOP, 0, 0, LEDS, 1, 0, 0, 0, 0, 0)
			ins( 6, ROTATE|HALT, NOP, 4, LEDS, 0, 2, 0, 0, 0, 0, 0)
//			ins( 7, ROTATE|HALT, NOP, 4 , 0, LEDS, 2, 0, 0, 0, 0, 0)
*/
		break;
		
		case BLINK:
			ins( 0, ROTATE|HALT, NOP, 1, 0, LEDS, 0, 0, 0, 0, 0, 0)
			ins( 1, ROTATE|HALT, NOP, 0, LEDS, 0, 0, 0, 0, 0, 0, 0)
		break;
		
		case TWINK:
//			ins( 0, SET_SPARKLE, 0x38, 0, 0x40, 0x0, 3, 0, 0, 0, 0, 0)	
//			ins( 1, SPARKLE, NOP, 4, 0, LEDS, 3, 3, 0, 0, 0, 0)
//			ins( 2, HALT, NOP, 2, 0, 0, 0 , 0, 0, 0, 0, 0)
			control ^= TWINKLE;
			sparkle_ena = 0x03;
		break;
		
		case RANDOM:
			ins( 0, RAND | HALT, NOP, 0, 0, LEDS, 0, 0, 0, 0, 0, 0)
		break;
		
		default:
		break;
	}
	return(1);
}