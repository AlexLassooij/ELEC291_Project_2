SHELL=cmd
CC = xc32-gcc
OBJCPY = xc32-bin2hex
ARCH = -mprocessor=32MX130F064B
OBJ = Project2.o 
PORTN=$(shell type COMPORT.inc)

Project2.elf: $(OBJ)
	$(CC) $(ARCH) -o Project2.elf Project2.o -mips16 -DXPRJ_default=default -legacy-libc -Wl,-Map=Project2.map
	$(OBJCPY) Project2.elf
	@echo Success!
   
Project2.o: Project2.c Project2.h LCD.h
	$(CC) -mips16 -g -x c -c $(ARCH) -MMD -o Project2.o Project2.c -DXPRJ_default=default -legacy-libc


clean:
	@del *.o *.elf *.hex *.map *.d 2>NUL
	
LoadFlash:
	@Taskkill /IM putty.exe /F 2>NUL | wait 500
	pro32 -p Project2.hex

putty:
	@Taskkill /IM putty.exe /F 2>NUL | wait 500

dummy: Project2.hex Project2.map
	$(CC) --version