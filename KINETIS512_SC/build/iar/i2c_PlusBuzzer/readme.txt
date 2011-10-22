This project is a simple I2C example that communicates with the MMA7660 accelerometer on the tower board. It prints out the acceleromer X, Y, and Z axis data to the terminal.

By default the OS-JTAG is used for the terminal output. The terminal should be configured for 115200 8-N-1.

Supported platforms:
- TWR_K40X256
- TWR_K60N512

The i2c.eww file will open the project for all of the supported platforms. Pick the specific project that corresponds to your hardware.

NOTE: if switching between platforms it is a good idea to do a make clean to make sure the code is properly configured for the new platform.

