all: png stl

placeleds: placeleds.c PCBCase/pcb.c PCBCase/pcb.h
	gcc -IPCBCase -O -o $@ $< PCBCase/pcb.c -lpopt -lm -g

leds:	placeleds
	./placeleds Gyro/Gyro.kicad_pcb -x100 -y100 -d17 --diode=2 --count=16 --fill=VLED --tracks --vias=0.8 --power-vias=0.8 --zone-in=1.375 --zone-out=1.375

stl:    Gyro/Gyro.stl

png:	$(patsubst %.kicad_pcb,%.png,$(wildcard */*.kicad_pcb))

PCBCase/case: PCBCase/case.c
	make -C PCBCase

PCBCase/clean: PCBCase/clean.c
	make -C PCBCase

%.stl: %.scad
	echo "Making $@"
	/Applications/OpenSCAD.app/Contents/MacOS/OpenSCAD $< -o $@
	echo "Made $@"

Gyro/Gyro.scad: Gyro/Gyro.kicad_pcb PCBCase/case Makefile
	PCBCase/case -o $@ $< --edge=2 --bottom=2 --top=7 --pcb=2

%.png:	%.kicad_pcb PCBCase/clean PCBCase/render Makefile
	PCBCase/render $<
