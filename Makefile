#
# This is a project Makefile. It is assumed the directory this Makefile resides in is a
# project subdirectory.
#

PROJECT_NAME := Gyro
SUFFIX := $(shell components/ESP32-RevK/buildsuffix)

ifeq ($(wildcard /bin/csh),)
$(error	Please install /bin/csh or equivalent)
endif

all:	main/settings.h
	@echo Make: $(PROJECT_NAME)$(SUFFIX).bin
	@idf.py build
	@cp build/$(PROJECT_NAME).bin $(PROJECT_NAME)$(SUFFIX).bin
	@cp build/bootloader/bootloader.bin $(PROJECT_NAME)$(SUFFIX)-bootloader.bin
	@echo Done: $(PROJECT_NAME)$(SUFFIX).bin

beta:	
	-git pull
	-git submodule update --recursive
	-git commit -a
	@make set
	cp $(PROJECT_NAME)*.bin release/beta
	git commit -a -m Beta
	git push

issue:	
	-git pull
	-git commit -a
	cp -f release/beta/$(PROJECT_NAME)*.bin release
	git commit -a -m Release
	git push

main/settings.h:     components/ESP32-RevK/revk_settings main/settings.def components/*/settings.def
	components/ESP32-RevK/revk_settings $^

components/ESP32-RevK/revk_settings: components/ESP32-RevK/revk_settings.c
	make -C components/ESP32-RevK revk_settings

components/ESP32-RevK/idfmon: components/ESP32-RevK/idfmon.c
	make -C components/ESP32-RevK idfmon

set:    s3

s3:
	components/ESP32-RevK/setbuildsuffix -S3-MINI-N4-R2
	@make

flash:
	idf.py flash

monitor:
	idf.py monitor

clean:
	idf.py clean

menuconfig:
	idf.py menuconfig

pull:
	git pull
	git submodule update --recursive

update:
	-git pull
	-git commit -a
	git submodule update --init --recursive --remote
	-git commit -a -m "Library update"
	-git push
