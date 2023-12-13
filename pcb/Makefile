all: fab

clean:
	rm -r fab

fab: mtb-rc.kicad_pcb
	kikit fab jlcpcb --no-drc mtb-rc.kicad_pcb fab

.PHONY: all clean
