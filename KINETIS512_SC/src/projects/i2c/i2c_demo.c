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

//Function declarations
void calibrate();
void delay(void);
signed char convert (int axis, signed char input);
/********************************************************************/
/*

 */
void main (void)
{
  signed char resultx, resulty, resultz;
  unsigned long all_registers = 0;
  printf("Kinetis I2C Demo\n");

  //Initialize I2C
  init_I2C();

  //Configure MMA7660 sensor
  MMA7660WriteRegister(0x09,0xE0); //Disable tap detection
  MMA7660WriteRegister(0x07,0x19); //Enable auto-sleep, auto-wake, and put in active mode
  
  // Zero the accelerometer in its current state
  calibrate();

  printf("  X     Y     Z\n");

  while(1)
  {    
#if 0    
    //Read x-axis register
    resultx = convert(Accel_X_Register_Index, u8MMA7660ReadRegister(Accel_X_Register_Index)& 0x3F);
    printf("1. %3d", (int) resultx);

    //Read y-axis register
    resulty = convert(Accel_Y_Register_Index, u8MMA7660ReadRegister(Accel_Y_Register_Index)& 0x3F);
    printf("   %3d", (int) resulty);

    //Read z-axis register
    resultz = convert(Accel_Z_Register_Index, u8MMA7660ReadRegister(Accel_Z_Register_Index)& 0x3F);
    printf("   %3d\n", (int) resultz);
#endif
    
    // Or you can do this all at once
    all_registers = u8MMA7660ReadThreeRegisters(Accel_X_Register_Index);
    resultx = convert(Accel_X_Register_Index, (all_registers >> 16) & 0x3F);
    resulty = convert(Accel_Y_Register_Index, (all_registers >> 8) & 0x3F);
    resultz = convert(Accel_Z_Register_Index, all_registers & 0x3F);
    printf("%3d    %3d     %3d\n",resultx,resulty,resultz);
  }

}

/*******************************************************************/

/*
 * Calibrate accelerometer offset
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
    x_axis_reg_val = (all_registers >> 16) & 0x3F;
    y_axis_reg_val = (all_registers >> 8) & 0x3F;
    z_axis_reg_val = all_registers & 0x3F;
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
  //z_offset=(signed char)(z_avg-21)*-1;

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
