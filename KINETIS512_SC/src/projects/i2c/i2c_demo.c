/*
 * File:        i2c_demo.c
 * Purpose:     I2C example of talking to MMA7660 on the Tower boards.
 *
 *              This demo uses a single byte read, but there is also a
 *                u8MMA7660ReadThreeRegisters() function available to show
 *                an example of doing multiple reads back to back.
 *
 *              TWR-K40X256 uses I2C1
 *              TWR-K60N512 uses I2C0
 *              TWR-K53N512 uses I2C1
 *
 */

#include "common.h"
#include "lptmr.h"
#include <stdio.h>


//Determine which chip is being used to determine if need to configure I2C0 or I2C1
#ifdef TWR_K60N512
  #include "k60_i2c.h"
#elif TWR_K40X256
  #include "k40_i2c.h"
#else
  #include "k53_i2C.h"
#endif

enum {
  Accel_X_Register_Index,
  Accel_Y_Register_Index,
  Accel_Z_Register_Index,
  Accel_Num_Axis_Registers,
  Accel_Tilt_Register_Index = Accel_Num_Axis_Registers,
  Accel_SRST_Register_Index,
  Accel_SPCNT_Register_Index,
  Accel_INTSU_Register_Index,
  Accel_MODE_Register_Index,
  Accel_SR_Register_Index,
  Accel_PDET_Register_Index,
  Accel_PD_Register_Index,
  Accel_Number_Of_Registers
};

typedef unsigned char boolean;
//Function declarations
void report_settings();
void report_axis_values();
void report_tilt_values();

void calibrate();
void delay(void);
boolean alert_generated(signed char register_val);
signed char convert (int axis, signed char input);
/********************************************************************/
/*

 */
void main (void)
{
  printf("Kinetis I2C Demo\n");

  //Generic I2C initialization
  init_I2C();

  //Configure MMA7660 sensor
  MMA7660WriteRegister(0x09,0xE0); //Disable tap detection
  MMA7660WriteRegister(0x07,0x19); //Enable auto-sleep, auto-wake, and put in active mode
  report_settings();
  
  // Zero the accelerometer
  calibrate();

  while(1)
  {    
    report_axis_values();
    report_tilt_values();
  }

}

/*******************************************************************/

/*
 * Report various settings:
 *    Sample rate
 *    Modes
 */

void report_settings()
{
  unsigned char reg_val;
  
  reg_val = u8MMA7660ReadRegister(Accel_Tilt_Register_Index);
  printf("Tilt Register = %0x\n", reg_val);
  
  reg_val = u8MMA7660ReadRegister(Accel_SRST_Register_Index);
  printf("Sample Rate Status Register = %0x\n", reg_val);
  
  reg_val = u8MMA7660ReadRegister(Accel_SPCNT_Register_Index);
  printf("Sleep Count Register = %0x\n", reg_val);
  
  reg_val = u8MMA7660ReadRegister(Accel_INTSU_Register_Index);
  printf("Interrupt Setup Register = %0x\n", reg_val);
 
  reg_val = u8MMA7660ReadRegister(Accel_MODE_Register_Index);
  printf("Mode Register = %0x\n", reg_val);
  
  reg_val = u8MMA7660ReadRegister(Accel_SR_Register_Index);
  printf("Sample Rate Register = %0x\n", reg_val);
  
  reg_val = u8MMA7660ReadRegister(Accel_PDET_Register_Index);
  printf("Tap/Pulse Detection Register = %0x\n", reg_val);
  
  reg_val = u8MMA7660ReadRegister(Accel_PD_Register_Index);
  printf("Tap/Pulse Debounce Register = %0x\n", reg_val);
}

/*******************************************************************/

/*
 * Report current x, y, z values
 */

void report_axis_values()
{
  static boolean initialized = 0;
  
  if (!initialized)
  {
    printf("  X     Y     Z\n");
    initialized = 1;
  }

  signed char resultx, resulty, resultz;
  int tries_remaining;
  unsigned long all_registers;
  
  // Read all three axis registers.  Retry a few times if a read conflict occurs.
  tries_remaining = 3;
  do
  {
    all_registers = u8MMA7660ReadThreeRegisters(Accel_X_Register_Index);
    resultx = (all_registers >> 16) & 0xFF;
    resulty = (all_registers >> 8) & 0xFF;
    resultz = all_registers & 0xFF;
  }while (--tries_remaining > 0
      && (alert_generated(resultx) 
      || alert_generated(resulty) 
      || alert_generated(resultz)
      ));
  
  // Turn the register read into a calibrated value
  resultx = convert(Accel_X_Register_Index, resultx);
  resulty = convert(Accel_Y_Register_Index, resulty);
  resultz = convert(Accel_Z_Register_Index, resultz);
    
  printf("%3d    %3d     %3d\n",resultx,resulty,resultz);
}

/*******************************************************************/

/*
 * Report current tilt, tap, and shake values
 */

void report_tilt_values()
{
}

/*******************************************************************/

/*
 * Support routine - does the register value show an alert?
 */
boolean alert_generated(signed char register_val)
{
  return register_val & 0x40;
}

/*******************************************************************/

/*
 * Calibrate accelerometer offset from an average
 */

static signed char AxisOffsets[Accel_Num_Axis_Registers];
void calibrate()
{
  // Initialize to 0 - don't take chances
  for (int i = 0; i < Accel_Num_Axis_Registers; i++)
  {
    AxisOffsets[i] = 0;
  }
  unsigned long all_registers = 0;
  unsigned char x_axis_reg_val = 0, 
                y_axis_reg_val = 0, 
                z_axis_reg_val = 0;
  
  /* Calculate appropriate offset from average of multiple readings */
  const int nIterations = 64;
  int x_avg=0, x_offset = 0;
  int y_avg=0, y_offset = 0;
  int z_avg=0, z_offset = 0;
  
  for(int i=0;i<nIterations;i++)
  {
    all_registers = u8MMA7660ReadThreeRegisters(Accel_X_Register_Index);
    x_axis_reg_val = (all_registers >> 16) & 0xFF;
    y_axis_reg_val = (all_registers >> 8) & 0xFF;
    z_axis_reg_val = all_registers & 0xFF;
    //printf("%3x    %3x     %3x\n",
        //x_axis_reg_val,y_axis_reg_val,z_axis_reg_val);
    
    // Axis conversion offsets are 0 - gathering actual readings
    x_avg+=convert(Accel_X_Register_Index, x_axis_reg_val);
    y_avg+=convert(Accel_Y_Register_Index, y_axis_reg_val);
    z_avg+=convert(Accel_Z_Register_Index, z_axis_reg_val);
  }
  
  x_avg=(int)x_avg/nIterations;
  y_avg=(int)y_avg/nIterations;
  z_avg=(int)z_avg/nIterations;

  /* Calculate offset */
  x_offset= x_avg*-1;
  y_offset= y_avg*-1;
  z_offset= z_avg*-1;
  //z_offset=(signed char)(z_avg-21)*-1; (Not sure why this was set this way in the demo)

  printf("Offsets are x=%d and y=%d and z=%d\n",x_offset,y_offset,z_offset);
  AxisOffsets[Accel_X_Register_Index] = x_offset;
  AxisOffsets[Accel_Y_Register_Index] = y_offset;
  AxisOffsets[Accel_Z_Register_Index] = z_offset;  
}



/*******************************************************************/
/*
 * Convert 6-bit accelerometer result into an 8-bit signed char
 */
signed char convert(int axis, signed char input)
{
  input &=~0x40;    //Turn off alert bit
  if(input & 0x20)  // update 6-bit signed format to 8-bit signed format
    input |= 0xE0;
  
  input += AxisOffsets[axis];
  return input;
}
