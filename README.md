# BMP180-and-IIS328DQ-interface-to-STM32

The project is designed using STM32CUBEMX and System Workbench for STM32.
full repository can be find in DataLogger folder.
All the screenshots of working system is stored in Image folder.

Steps to extract the code in SW4STM32 IDE.

	1.	Download and Unzip this repository.

	2.	open SW4STM32 and navigate to File>Open Projects from File System...
	
	3.	click on Directory and brawser the directory of DataLogger.
		
	4.	The hit enter and the project will be import to your IDE shortly.
	
	
	
Connections :
	
	STM32 				IIS328DQ			BMP180
		
	I2C1_SCL(PB8)			SCL				SCL
	I2C1_SDA(PB9)			SDI				SDA
	3.3V				VDD				3.3V
	GND				GND				GND
	3.3V				CS				--
	GND				SDO				--
