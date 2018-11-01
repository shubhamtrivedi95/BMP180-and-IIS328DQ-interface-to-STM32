# BMP180-and-IIS328DQ-interface-to-STM32

The project is designed using STM32CUBEMX and System Workbench for STM32.
STM32F746ZG (NUCLEO-F746ZG) is used as microcontroller in this project.
full repository can be find in DataLogger folder.
All the screenshots of working system is stored in Image folder.

Steps to extract the code in SW4STM32 IDE.

	1.	Download and Unzip this repository.

	2.	open SW4STM32 and navigate to File>Open Projects from File System...
	
	3.	click on Directory and brawser the directory of DataLogger.
		
	4.	The hit enter and the project will be import to your IDE shortly.
	
	
	
Connections :
	
	STM32F746ZG			IIS328DQ			BMP180
		
	I2C1_SCL(PB8)			SCL				SCL
	I2C1_SDA(PB9)			SDI				SDA
	3.3V				VDD				3.3V
	GND				GND				GND
	3.3V				CS				--
	GND				SDO				--

Project Description:

	The program is designed in such way that it will read the Pressure and 
	Temperature data from BMP180 sensor at every 10 seconds.Also it will read Acceleration on 
	X,Y,Z axis from IIS328DQ at every 0.1 seconds.Acceleration reading task is configured in interrupt based logic.
	Where timer 3 is used to generate an interrupt on every 100 milliseconds.
	BMP180 sensor reading task is running in main loop.
	There is no HAL_DELAY function used in main loop. the action of BMP180 sensor reading is performed by using HAL_GetTick()
	method. which returns run time value in microseconds. for more details check main.c file.

Thank you 
for any issue or clarification contact me on mail:
mailto:shubhamtrivedi95@gmail.com
