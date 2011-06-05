// Source for AVR developed by rwatson: http://www.koders.com/c/fidD231788581D546D9C5CBC2B0E3DCCB64D46957E6.aspx
// Ported to ARM by cherep22

#ifndef ds2450_h
#define ds2450_h

//----- Include Files ---------------------------------------------------------
#include "types.h"

//----- Defines ---------------------------------------------------------------
#define ds2450_rev					5

// the two voltage ranges
#define DS2450_RANGE_2V				0x00    // 0-2.55V
#define DS2450_RANGE_5V				0x01    // 0-5.10V
#define DS2450_RANGE_DO_NOT_CHANGE	0xFF	// don't change range for channel

// the family code
#define DS2450_FAMILY				0x20

// the starting addresses
// of the pages in RAM
#define DS2450_DATA_PAGE			0x00
#define DS2450_SETUP_PAGE			0x08
#define DS2450_ALARM_PAGE			0x10

#define DS2450_VCC_FLAG				0x40
#define DS2450_VCC_ADDR				0x1C

// maximum allowable resolution
#define DS2450_RES_MAX				16
#define DS2450_RES_DO_NOT_CHANGE	0xFF	// don't change resolution for channel

// digital out
#define DS2450_OUTPUT_LOW			0		// enable output control and switch the transistor on (pull down output)
#define DS2450_OUTPUT_HIGH			1		// switch the transistor off and disable output control
#define DS2450_OUTPUT_DO_NOT_CHANGE	0xFF	// don't change resolution for channel

// function commands
#define DS2450_READ_MEMORY			0xAA
#define DS2450_WRITE_MEMORY			0x55
#define DS2450_CONVERT				0x3C
#define DS2450_CONVERT_ALL4_MASK	0x0F
#define DS2450_CLEAR_ALL4_MASK		0x55

//----- Typedefs --------------------------------------------------------------

// enumerated constant for configuring
// and controlling an A2D channel as a digital output
typedef enum {DIGOUT_LOW=0x80, DIGOUT_OC=0xC0, DIGOUT_DISABLE=0x00, DIGOUT_UNKNOWN=-1} dallas_a2d_out_T;

//----- Functions ---------------------------------------------------------------

#ifdef	__cplusplus
extern "C" {
#endif

// ds2450Init()
//     initializes the dallas 1-wire bus
DECLEXPORT void DALLASLIB ds2450Init(void);

//----- Single Channel Functions ----------------------------------------------
// The following 4 functions are used for controlling a single channel on the
// a2d converter.  If you are only using 1 channel, then these functions are
// faster.  If you are using 2 or more channel, it is faster to use the All
// Channel Functions.  This is because to convert all the channel only requires
// one command to the device, and then a read of the all the channel.  To read
// two channel individually requires two commands to the device, and two reads.
// Therefore using the All Channel Functions for even just 2 channels is faster
// and more effificient.

// ds2450Setup()
//     Sets up the given device, for the given channel [A-D],
//     the given resolution [1-16] and the given range 0-2.55 or 0-5.10
//     Returns either the corresponding error or DALLAS_NO_ERROR
DECLEXPORT u08  DALLASLIB ds2450Setup(dallas_rom_id_T* rom_id, u08 channel, u08 resolution, u08 range);

// ds2450Start()
//     Starts the a2d conversion for the given device and the given channel [A-D]
//     Returns either the corresponding error or DALLAS_NO_ERROR
DECLEXPORT u08  DALLASLIB ds2450Start(dallas_rom_id_T* rom_id, u08 channel);

// ds2450Result()
//     Gets the result from the a2d conversion
//     for the given device and the given channel [A-D]
//     Returns either the corresponding error or DALLAS_NO_ERROR
DECLEXPORT u08  DALLASLIB ds2450Result(dallas_rom_id_T* rom_id, u08 channel, u16* result);

// ds2450StartAndResult()
//     Starts the conversion of the given device and the given channel [A-D]
//     Stores the result in the variable result
//     Returns either the corresponding error or DALLAS_NO_ERROR
DECLEXPORT u08  DALLASLIB ds2450StartAndResult(dallas_rom_id_T* rom_id, u08 channel, u16 *result);

//----- All Channel Functions -------------------------------------------------
// The following 4 commands are used to access data from all 4 channels on the
// a2d converter.  These commands should be used if you are using more than one
// channel on the device.  See the Single Channel Functions description for
// more information

// ds2450ReadAllSettings()
//     Gets resolution, range and digital output setings for all channels
//     range, resolution and digital_output are pointers to 4 bytes buffers
//     range[i] will be set to DS2450_RANGE_2V [0-2.55], DS2450_RANGE_5V [0-5.10] or DS2450_RANGE_DO_NOT_CHANGE
//     digital_output[i] will be set to DS2450_OUTPUT_LOW, DS2450_OUTPUT_HIGH
//     resolution[i] will be set from 1 to 16
//     Returns either the corresponding error or DALLAS_NO_ERROR
DECLEXPORT u08 DALLASLIB ds2450ReadAllSettings(dallas_rom_id_T* rom_id, u08 *resolution, u08 *range, u08 *digital_output);

// ds2450WriteAllSettings()
//     Sets up the given device for all channels for the given resultions (1..16 or DS2450_RES_DO_NOT_CHANGE), 
//     ranges [0-2.55, 0-5.10 or DS2450_RANGE_DO_NOT_CHANGE]
//     and digital output states (low, high, DS2450_OUTPUT_DO_NOT_CHANGE)
//     Returns either the corresponding error or DALLAS_NO_ERROR
DECLEXPORT u08 DALLASLIB ds2450WriteAllSettings(dallas_rom_id_T* rom_id, u08 *resolution, u08 *range, u08 *digital_output);

// ds2450SetupAll()
//     Sets up the given device for all channels for the given resolution
//     and the given range [0-2.55 or 0-5.10]
//     Returns either the corresponding error or DALLAS_NO_ERROR
DECLEXPORT u08 DALLASLIB ds2450SetupAll(dallas_rom_id_T* rom_id, u08 resolution, u08 range);

// ds2450StartAll()
//     Starts the conversion for all 4 channels on the given a2d converter
//     Returns either the corresponding error or DALLAS_NO_ERROR
DECLEXPORT u08 DALLASLIB ds2450StartAll(dallas_rom_id_T* rom_id);

// ds2450ResultAll
//     Gets the results from the given device
//     and stores the result in the given array
//     Returns either the corresponding error or DALLAS_NO_ERROR
DECLEXPORT u08  DALLASLIB ds2450ResultAll(dallas_rom_id_T* rom_id, u16 result[4]);

// ds2450StartAndResultAll()
//     1-Step command to start the conversion for the given device,
//     and store the results in the given array
//     Returns either the corresponding error or DALLAS_NO_ERROR
DECLEXPORT u08 DALLASLIB ds2450StartAndResultAll(dallas_rom_id_T* rom_id, u16 result[4]);

// ds2450Print()
//     Does a formatted print on the given result for the given range
//void ds2450Print(u16 result, u08 range);

//----- Digital Out Functions -------------------------------------------------
// ds2450DigitalOut
//     Use the given channel of the given device as a digital out
//     Returns either the corresponding error or DALLAS_NO_ERROR
DECLEXPORT u08 DALLASLIB ds2450DigitalOut(dallas_rom_id_T* rom_id, u08 channel, dallas_a2d_out_T state);

#ifdef	__cplusplus
};
#endif

#endif
