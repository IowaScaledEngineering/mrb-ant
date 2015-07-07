v 20130925 2
T 42000 54300 9 10 1 0 0 6 1
VIN
T 42000 53900 9 10 1 0 0 6 1
GND
N 58700 50300 58700 50700 4
N 55300 56300 54800 56300 4
N 55000 55900 54800 55900 4
N 53200 55100 53200 56300 4
N 52900 56700 53400 56700 4
N 53400 55900 52600 55900 4
{
T 51900 55900 5 10 1 1 0 0 1
netname=/RESET
}
T 53600 57200 9 10 1 0 0 0 1
ICSP Header
C 63100 52600 1 90 0 resistor-1.sym
{
T 62700 52900 5 10 0 0 90 0 1
device=RESISTOR
T 62800 53000 5 10 1 1 90 0 1
refdes=R5
T 63300 53000 5 10 1 1 90 0 1
value=10k
T 63100 52600 5 10 0 0 90 0 1
footprint=0805
}
C 62800 51500 1 270 0 capacitor-1.sym
{
T 63500 51300 5 10 0 1 270 0 1
device=CAPACITOR
T 63100 51200 5 10 1 1 0 0 1
refdes=C9
T 63700 51300 5 10 0 0 270 0 1
symversion=0.1
T 63100 50700 5 10 1 1 0 0 1
value=1uF
T 62800 51500 5 10 0 0 0 0 1
footprint=0805
T 63100 50500 5 10 1 1 0 0 1
description=16V
}
N 63000 51500 63000 52600 4
N 63000 53500 63000 58800 4
T 67500 40500 9 10 1 0 0 0 1
MRBus Analog Network Throttle
T 67500 40200 9 10 1 0 0 0 1
mrb-ant.sch
T 67500 39900 9 10 1 0 0 0 1
1
T 69000 39900 9 10 1 0 0 0 1
1
T 71500 39900 9 10 1 0 0 0 1
Nathan D. Holmes
C 40700 39600 0 0 0 title-bordered-D.sym
N 59100 57400 59100 58800 4
N 58700 57400 58700 58800 4
N 61300 55700 62100 55700 4
N 61300 51500 63000 51500 4
{
T 62300 51600 5 10 1 1 0 0 1
netname=/RESET
}
C 55300 54800 1 270 0 crystal-1.sym
{
T 55800 54600 5 10 0 0 270 0 1
device=CRYSTAL
T 55600 54600 5 10 1 1 270 0 1
refdes=Y1
T 56000 54600 5 10 0 0 270 0 1
symversion=0.1
T 54650 54400 5 10 1 1 0 0 1
value=20MHz
T 55300 54800 5 10 0 1 270 0 1
footprint=crystal-hc49-smt
}
C 54900 54300 1 180 0 capacitor-1.sym
{
T 54700 53600 5 10 0 1 180 0 1
device=CAPACITOR
T 54600 53800 5 10 1 1 0 0 1
refdes=C7
T 54700 53400 5 10 0 0 180 0 1
symversion=0.1
T 53600 53700 5 10 1 1 0 0 1
value=22pF
T 54900 54300 5 10 0 0 270 0 1
footprint=0805
T 53500 53900 5 10 1 1 0 0 1
description=16V, NP0
}
N 56100 54500 56500 54500 4
N 54900 54800 56500 54800 4
N 55300 55700 55300 56500 4
{
T 55300 57100 5 10 1 1 270 0 1
netname=MOSI
}
N 51500 53600 56500 53600 4
N 53000 55100 56500 55100 4
{
T 52400 55100 5 10 1 1 0 0 1
netname=SCLK
}
N 52900 55400 56500 55400 4
N 52900 55400 52900 56700 4
N 55300 55700 56500 55700 4
N 63000 50500 63000 50600 4
C 53400 55700 1 0 0 avrprog-1.sym
{
T 53400 57300 5 10 0 1 0 0 1
device=AVRPROG
T 54000 57000 5 10 1 1 0 0 1
refdes=J5
T 53600 55500 5 10 0 1 0 0 1
footprint=JUMPER3x2
}
T 42100 45900 9 10 1 0 0 2 3
Notes:
1) All capacitors are ceramic (X7R/X5R) unless otherwise noted.
2) All capacitors and resistors are 0805 unless otherwise noted.
N 45600 58800 46800 58800 4
N 54800 56700 55000 56700 4
N 55000 56700 55000 58800 4
C 54900 54600 1 0 1 capacitor-1.sym
{
T 54700 55300 5 10 0 1 180 2 1
device=CAPACITOR
T 54900 54900 5 10 1 1 0 6 1
refdes=C6
T 54700 55500 5 10 0 0 180 2 1
symversion=0.1
T 54000 54400 5 10 1 1 0 6 1
value=22pF
T 54900 54600 5 10 0 0 270 6 1
footprint=0805
T 54300 54600 5 10 1 1 0 6 1
description=16V, NP0
}
C 50200 56500 1 0 0 gnd-1.sym
N 49100 58600 49100 58800 4
C 50400 57900 1 90 0 resistor-1.sym
{
T 50000 58200 5 10 0 0 90 0 1
device=RESISTOR
T 50100 58100 5 10 1 1 90 0 1
refdes=R1
T 50600 58100 5 10 1 1 90 0 1
value=330
T 50400 57900 5 10 0 0 90 0 1
footprint=0805
}
C 50500 56900 1 90 0 led-3.sym
{
T 50750 56850 5 10 1 1 90 0 1
device=GREEN LED
T 49950 57350 5 10 1 1 90 0 1
refdes=D3
T 50500 56900 5 10 0 0 0 0 1
footprint=1206
}
N 50300 57900 50300 57800 4
N 50300 56800 50300 56900 4
C 48900 58600 1 270 0 capacitor-1.sym
{
T 49600 58400 5 10 0 1 270 0 1
device=CAPACITOR
T 49200 58300 5 10 1 1 0 0 1
refdes=C2
T 49800 58400 5 10 0 0 270 0 1
symversion=0.1
T 49200 57800 5 10 1 1 0 0 1
value=1uF
T 49200 57400 5 10 0 1 0 0 1
footprint=0805
T 49200 57600 5 10 1 1 0 0 1
comment=16V
}
N 45700 55500 49600 55500 4
C 46500 52000 1 0 1 rs485-1.sym
{
T 44850 53800 5 10 0 0 0 6 1
device=MAX489
T 45150 52150 5 10 1 1 0 6 1
refdes=U4
T 45900 51900 5 10 1 1 0 0 1
footprint=SO8
}
N 45700 51300 47400 51300 4
N 46700 51300 46700 52400 4
N 46700 52400 46500 52400 4
C 48000 54500 1 90 0 resistor-1.sym
{
T 47600 54800 5 10 0 0 90 0 1
device=RESISTOR
T 47700 54700 5 10 1 1 90 0 1
refdes=R2
T 48200 54700 5 10 1 1 90 0 1
value=330
T 48000 54500 5 10 0 0 90 0 1
footprint=0805
}
C 48100 53500 1 90 0 led-3.sym
{
T 48350 53450 5 10 1 1 90 0 1
device=AMBER LED
T 47550 53950 5 10 1 1 90 0 1
refdes=D5
T 48100 53500 5 10 0 0 0 0 1
footprint=1206
}
N 47900 54500 47900 54400 4
C 47000 53500 1 90 0 resistor-1.sym
{
T 46600 53800 5 10 0 0 90 0 1
device=RESISTOR
T 46700 53700 5 10 1 1 90 0 1
refdes=R3
T 47200 53700 5 10 1 1 90 0 1
value=10k
T 47000 53500 5 10 0 0 90 0 1
footprint=0805
}
C 47500 51500 1 90 0 resistor-1.sym
{
T 47100 51800 5 10 0 0 90 0 1
device=RESISTOR
T 47200 51700 5 10 1 1 90 0 1
refdes=R4
T 47700 51700 5 10 1 1 90 0 1
value=10k
T 47500 51500 5 10 0 0 90 0 1
footprint=0805
}
N 46900 53500 46900 53300 4
N 45700 51000 45700 52000 4
N 45700 53900 45700 55500 4
N 46900 55500 46900 54400 4
N 47900 53500 47900 52700 4
N 47400 52400 47400 53000 4
N 47400 51300 47400 51500 4
N 47900 55400 47900 55500 4
C 42900 52800 1 0 1 termblk2-1.sym
{
T 41900 53450 5 10 0 0 0 6 1
device=TERMBLK2
T 42500 52500 5 10 1 1 0 6 1
refdes=J3
T 42900 52800 5 10 0 1 0 0 1
footprint=TERMBLK2_200MIL
}
N 42900 53000 44400 53000 4
N 44400 52600 44400 58000 4
N 44400 52600 44900 52600 4
N 42900 53400 44900 53400 4
C 45600 50700 1 0 0 gnd-1.sym
T 42000 53300 9 10 1 0 0 6 1
RS485-A
T 42000 52900 9 10 1 0 0 6 1
RS485-B
C 56500 50700 1 0 0 mega328-tqfp32.sym
{
T 61000 57200 5 10 1 1 0 6 1
refdes=U5
T 56800 57500 5 10 0 0 0 0 1
device=ATMega328-TQFP32
T 56800 57700 5 10 0 0 0 0 1
footprint=TQFP32
}
N 53200 56300 53400 56300 4
C 46800 58200 1 0 0 78l05-1.sym
{
T 48400 59500 5 10 0 0 0 0 1
device=7805
T 48100 59200 5 10 1 1 0 6 1
refdes=U1
T 46800 58200 5 10 1 1 0 0 1
footprint=SOT89
}
C 45900 58600 1 270 0 Cap_H-2.sym
{
T 46200 58400 5 10 1 1 0 0 1
refdes=C1
T 47400 58600 5 10 0 0 270 0 1
device=Capacitor
T 46200 57900 5 10 1 1 0 2 1
value=68uF
T 45900 58600 5 10 0 0 0 0 1
footprint=cap-elec-Panasonic-FK--D6.30-H5.80-mm
T 45500 57400 5 10 1 1 0 0 1
description=25V, Electrolytic
}
N 43600 54400 43600 58800 4
N 44200 53400 44200 58200 4
C 43200 44400 1 0 0 hole-1.sym
{
T 43200 44400 5 10 0 1 0 0 1
device=HOLE
T 43400 45000 5 10 1 1 0 4 1
refdes=H1
T 43200 44400 5 10 0 0 0 0 1
footprint=STANDOFF_HEX_n4
}
C 43700 44400 1 0 0 hole-1.sym
{
T 43700 44400 5 10 0 1 0 0 1
device=HOLE
T 43900 45000 5 10 1 1 0 4 1
refdes=H2
T 43700 44400 5 10 0 0 0 0 1
footprint=STANDOFF_HEX_n4
}
N 43600 56800 43100 56800 4
N 43100 56600 43800 56600 4
N 43800 53900 43800 58600 4
N 43100 58600 43800 58600 4
N 43100 56200 44200 56200 4
N 44200 58200 43100 58200 4
N 43100 58000 44400 58000 4
N 43100 57600 44600 57600 4
N 44600 57600 44600 55600 4
N 44600 55600 43100 55600 4
N 43100 55400 44800 55400 4
N 44800 55400 44800 57400 4
N 44800 57400 43100 57400 4
N 43600 58400 43100 58400 4
N 43600 56400 43100 56400 4
N 43100 57800 43800 57800 4
N 43800 55800 43100 55800 4
C 42200 55200 1 0 0 rj45-dual.sym
{
T 42200 58100 5 10 0 0 0 0 1
device=RJ45
T 42200 55200 5 10 0 0 0 0 1
footprint=modular_8p8c_dual
T 41900 57900 5 10 1 1 0 0 1
refdes=J1
}
C 43700 53600 1 0 0 gnd-1.sym
N 43100 58800 44700 58800 4
N 43100 56000 44400 56000 4
C 42900 53800 1 0 1 termblk2-1.sym
{
T 41900 54450 5 10 0 0 0 6 1
device=TERMBLK2
T 42500 54800 5 10 1 1 0 6 1
refdes=J2
T 42900 53800 5 10 0 1 0 0 1
footprint=TERMBLK2_200MIL
}
N 42900 54000 43800 54000 4
N 42900 54400 43600 54400 4
N 46100 58600 46100 58800 4
C 49000 57100 1 0 0 gnd-1.sym
N 46100 57700 49100 57700 4
N 49100 57700 49100 57400 4
N 47600 58200 47600 57700 4
C 45900 55500 1 270 0 capacitor-1.sym
{
T 46600 55300 5 10 0 1 270 0 1
device=CAPACITOR
T 46200 55200 5 10 1 1 0 0 1
refdes=C3
T 46800 55300 5 10 0 0 270 0 1
symversion=0.1
T 46200 54700 5 10 1 1 0 0 1
value=0.1uF
T 45900 55500 5 10 0 0 0 0 1
footprint=0805
T 46200 54500 5 10 1 1 0 0 1
description=16V
}
C 46000 54300 1 0 0 gnd-1.sym
N 54900 54100 56100 54100 4
N 56100 54100 56100 54500 4
N 54000 54800 53300 54800 4
N 53300 54800 53300 54100 4
N 53300 54100 54000 54100 4
C 59300 58800 1 270 0 capacitor-1.sym
{
T 60000 58600 5 10 0 1 270 0 1
device=CAPACITOR
T 59600 58500 5 10 1 1 0 0 1
refdes=C4
T 60200 58600 5 10 0 0 270 0 1
symversion=0.1
T 59600 58000 5 10 1 1 0 0 1
value=0.1uF
T 59600 57600 5 10 0 1 0 0 1
footprint=0805
T 59600 57800 5 10 1 1 0 0 1
comment=16V
}
C 61900 56600 1 270 0 capacitor-1.sym
{
T 62600 56400 5 10 0 1 270 0 1
device=CAPACITOR
T 62200 56300 5 10 1 1 0 0 1
refdes=C8
T 62800 56400 5 10 0 0 270 0 1
symversion=0.1
T 62200 55800 5 10 1 1 0 0 1
value=0.1uF
T 62200 55400 5 10 0 1 0 0 1
footprint=0805
T 62200 55600 5 10 1 1 0 0 1
comment=16V
}
C 44200 44400 1 0 0 hole-1.sym
{
T 44200 44400 5 10 0 1 0 0 1
device=HOLE
T 44400 45000 5 10 1 1 0 4 1
refdes=H3
T 44200 44400 5 10 0 0 0 0 1
footprint=STANDOFF_HEX_n4
}
C 44700 44400 1 0 0 hole-1.sym
{
T 44700 44400 5 10 0 1 0 0 1
device=HOLE
T 44900 45000 5 10 1 1 0 4 1
refdes=H4
T 44700 44400 5 10 0 0 0 0 1
footprint=STANDOFF_HEX_n4
}
C 64900 53200 1 0 0 lm358-1.sym
{
T 65575 53800 5 10 0 0 0 0 1
device=LM358
T 65600 54550 5 10 0 0 0 0 1
footprint=SO8
T 65100 54100 5 10 1 1 0 0 1
refdes=U7
T 64900 53200 5 10 0 0 0 0 1
slot=1
}
C 68700 53600 1 90 1 2N3904-2.sym
{
T 68560 53413 5 10 1 1 0 2 1
device=MMBT2222
T 68600 53600 5 10 1 1 0 2 1
refdes=Q2
T 67794 52696 5 10 0 0 270 2 1
footprint=SOT23
}
C 68600 54500 1 0 0 resistor-1.sym
{
T 68900 54900 5 10 0 0 0 0 1
device=TE 2-1623788-5
T 68800 54800 5 10 1 1 0 0 1
refdes=R10
T 68800 54300 5 10 1 1 0 0 1
value=0.22
T 68600 54500 5 10 0 0 0 0 1
footprint=SQ-5W
T 68700 54100 5 10 1 1 0 0 1
comment=>= 3W
}
N 68600 54600 67500 54600 4
C 68300 53600 1 90 0 resistor-1.sym
{
T 67900 53900 5 10 0 0 90 0 1
device=RESISTOR
T 68000 53800 5 10 1 1 90 0 1
refdes=R9
T 68500 53800 5 10 1 1 90 0 1
value=22
T 68300 53600 5 10 0 0 90 0 1
footprint=0805
}
N 68200 54500 68200 54600 4
N 67500 53000 67700 53000 4
N 68700 53000 69700 53000 4
N 69700 53000 69700 54600 4
N 69500 54600 71800 54600 4
N 71500 50800 71500 53800 4
C 71900 55500 1 0 0 termblk2-1.sym
{
T 72900 56150 5 10 0 0 0 0 1
device=TERMBLK2
T 72300 55200 5 10 1 1 0 0 1
refdes=J6
T 71900 55500 5 10 0 1 0 6 1
footprint=TERMBLK2_200MIL
}
C 69800 52100 1 90 0 resistor-1.sym
{
T 69400 52400 5 10 0 0 90 0 1
device=RESISTOR
T 69500 52300 5 10 1 1 90 0 1
refdes=R8
T 70000 52300 5 10 1 1 90 0 1
value=27k
T 69800 52100 5 10 0 0 90 0 1
footprint=0805
}
C 69800 50900 1 90 0 resistor-1.sym
{
T 69400 51200 5 10 0 0 90 0 1
device=RESISTOR
T 69500 51100 5 10 1 1 90 0 1
refdes=R7
T 70000 51100 5 10 1 1 90 0 1
value=10k
T 69800 50900 5 10 0 0 90 0 1
footprint=0805
}
N 69700 51800 69700 52100 4
N 69700 50900 69700 50800 4
N 69700 51900 64900 51900 4
N 64900 51900 64900 53400 4
N 65400 54000 65400 54600 4
N 42800 46700 42800 48600 4
T 72800 55600 9 10 1 0 0 0 3
Track
Output
A
T 41000 48500 9 10 1 0 0 0 1
+15-20V
C 44000 47800 1 90 0 resistor-1.sym
{
T 43600 48100 5 10 0 0 90 0 1
device=RESISTOR
T 44000 47800 5 10 0 0 90 0 1
footprint=0805
T 43700 48000 5 10 1 1 90 0 1
refdes=R6
T 44200 48000 5 10 1 1 90 0 1
value=3k
}
C 44100 46900 1 90 0 led-3.sym
{
T 44100 46900 5 10 0 0 0 0 1
footprint=1206
T 43750 46850 5 10 1 1 90 0 1
refdes=D4
}
N 43900 48700 43900 48800 4
N 47000 48800 47800 48800 4
N 48600 48200 48600 46700 4
C 49700 48400 1 270 0 capacitor-1.sym
{
T 50400 48200 5 10 0 1 270 0 1
device=CAPACITOR
T 50200 48100 5 10 1 1 0 0 1
refdes=C11
T 50600 48200 5 10 0 0 270 0 1
symversion=0.1
T 50200 47800 5 10 1 1 0 0 1
value=1uF
T 50000 47200 5 10 0 1 0 0 1
footprint=0805
T 50200 47600 5 10 1 1 0 0 1
comment=16V
}
N 49400 48800 49900 48800 4
N 49900 47500 49900 46700 4
N 42800 46700 49900 46700 4
N 63900 53800 64900 53800 4
N 60100 45400 60300 45400 4
{
T 59500 45400 5 10 1 1 0 0 1
netname=SCLK
}
N 60100 45700 60300 45700 4
{
T 59500 45700 5 10 1 1 0 0 1
netname=/CS1
}
N 60100 45100 60300 45100 4
{
T 59500 45100 5 10 1 1 0 0 1
netname=MOSI
}
C 45500 44400 1 0 0 hole-1.sym
{
T 45500 44400 5 10 0 1 0 0 1
device=TO220 HEATSINK
T 45700 45000 5 10 1 1 0 4 1
refdes=HS1
T 45500 44400 5 10 0 0 0 0 1
footprint=aavid-heatsink-530002B02500G
}
C 67500 53600 1 90 0 2N6045G-1.sym
{
T 67000 54800 5 10 1 1 0 0 1
device=2N6045G
T 66800 55000 5 10 0 0 90 0 1
footprint=TO220-3
T 65800 54770 5 10 1 1 0 0 1
refdes=Q1
}
C 72900 52000 1 0 1 relay-DPDT-1.sym
{
T 72600 54850 5 10 1 1 0 6 1
refdes=K1
T 72900 52000 5 10 0 0 0 6 1
footprint=RELAY-DPDT
}
N 71800 52600 70800 52600 4
N 70800 52600 70800 54600 4
N 71500 53800 71800 53800 4
N 71800 53400 71500 53400 4
N 71800 53000 71200 53000 4
N 71200 53000 71200 56100 4
N 71800 54200 71500 54200 4
C 72900 52600 1 0 0 5V-plus-1.sym
{
T 72900 52600 5 10 0 0 0 0 1
netname=+5V
}
N 72900 52500 73100 52500 4
N 73100 52500 73100 52600 4
C 49500 51700 1 0 0 si8642-1.sym
{
T 49900 54700 5 10 1 1 0 0 1
refdes=U3
T 50200 51500 5 10 1 1 0 0 1
device=Si8642BA
T 49900 54300 5 10 0 0 0 0 1
footprint=SO16
}
N 46500 53300 49600 53300 4
N 51500 53300 56500 53300 4
N 51500 53000 56500 53000 4
N 46500 53000 49600 53000 4
N 49600 53600 48500 53600 4
N 48500 53600 48500 52700 4
N 48500 52700 46500 52700 4
N 49600 54400 49600 58800 4
C 49500 51300 1 0 0 gnd-1.sym
N 49600 51600 49600 52200 4
N 48400 58800 50300 58800 4
C 45600 60300 1 90 0 header4-1.sym
{
T 44950 61300 5 10 0 0 90 0 1
device=HEADER3
T 43900 60700 5 10 1 1 90 0 1
refdes=JP1
T 45600 60300 5 10 0 0 90 6 1
footprint=JUMPER4-SMT
}
C 44900 59700 1 0 0 gnd-1.sym
N 45000 60000 45000 60300 4
C 45300 59700 1 0 0 gnd-2.sym
N 45400 60000 45400 60300 4
N 44600 60300 44600 58800 4
N 44200 60300 43300 60300 4
N 43300 60300 43300 60600 4
N 49600 52700 49200 52700 4
N 49200 52700 49200 54400 4
N 49200 54400 49600 54400 4
N 55000 58800 63000 58800 4
C 51400 51300 1 0 0 gnd-2.sym
N 51500 51600 51500 52200 4
C 51300 56200 1 0 0 5V-plus-1.sym
N 51500 56200 51500 54400 4
N 49600 53900 49200 53900 4
N 51500 54400 52400 54400 4
N 52400 54400 52400 52700 4
N 52400 52700 51500 52700 4
N 56300 56600 56500 56600 4
{
T 55700 56600 5 10 1 1 0 0 1
netname=/CS1
}
N 61500 52700 61300 52700 4
{
T 62100 52700 5 10 1 1 0 6 1
netname=DIR1
}
N 61500 52400 61300 52400 4
{
T 62100 52400 5 10 1 1 0 6 1
netname=DIR2
}
C 60100 58800 1 270 0 capacitor-1.sym
{
T 60800 58600 5 10 0 1 270 0 1
device=CAPACITOR
T 61000 58600 5 10 0 0 270 0 1
symversion=0.1
T 60400 57600 5 10 0 1 0 0 1
footprint=0805
T 60400 58500 5 10 1 1 0 0 1
refdes=C5
T 60400 58000 5 10 1 1 0 0 1
value=0.1uF
T 60400 57800 5 10 1 1 0 0 1
comment=16V
}
C 53400 53800 1 0 1 gnd-2.sym
C 54900 55600 1 0 0 gnd-2.sym
N 58700 50700 59100 50700 4
C 58800 50000 1 0 1 gnd-2.sym
C 63100 50200 1 0 1 gnd-2.sym
C 60400 57300 1 0 1 gnd-2.sym
N 59500 57900 59500 57800 4
N 59500 57800 60300 57800 4
N 60300 57900 60300 57600 4
C 62200 55100 1 0 1 gnd-2.sym
N 62100 55400 62100 55700 4
N 62100 56600 61300 56600 4
N 61300 56600 61300 56300 4
N 71200 56100 71900 56100 4
N 71900 55700 71500 55700 4
N 71500 55700 71500 54200 4
N 43900 46900 43900 46700 4
C 49700 48800 1 0 0 5V-plus-1.sym
{
T 49700 48800 5 10 0 0 0 0 1
netname=+5V
}
C 43100 60600 1 0 0 vcc-1.sym
N 42800 48800 46100 48800 4
C 44500 48800 1 0 0 vcc-1.sym
C 42900 46400 1 0 1 gnd-2.sym
C 60200 43500 1 0 0 mcp49x2-1.sym
{
T 60600 46300 5 10 1 1 0 0 1
refdes=U6
T 60800 43700 5 10 1 1 0 0 1
device=MCP4902
T 60600 46100 5 10 0 0 0 0 1
footprint=SO14
}
C 58100 58800 1 0 0 5V-plus-1.sym
{
T 58100 58800 5 10 0 0 0 0 1
netname=+5V
}
C 59100 48000 1 0 0 5V-plus-1.sym
{
T 59100 48000 5 10 0 0 0 0 1
netname=+5V
}
N 60300 46000 59300 46000 4
N 59300 44500 59300 48000 4
C 60600 47800 1 270 0 capacitor-1.sym
{
T 61300 47600 5 10 0 1 270 0 1
device=CAPACITOR
T 60300 47500 5 10 1 1 0 0 1
refdes=C12
T 61500 47600 5 10 0 0 270 0 1
symversion=0.1
T 61100 47400 5 10 1 1 0 0 1
value=0.1uF
T 60900 46600 5 10 0 1 0 0 1
footprint=0805
T 61100 47200 5 10 1 1 0 0 1
comment=16V
}
N 59300 47800 62600 47800 4
C 60700 46600 1 0 0 gnd-2.sym
N 65400 54600 65800 54600 4
N 65900 53600 67500 53600 4
N 67500 53600 67500 53000 4
C 69800 50500 1 0 1 gnd-2.sym
N 71500 50800 69700 50800 4
C 65300 52900 1 0 0 gnd-2.sym
C 53200 50800 1 0 0 switch-dip5-1.sym
{
T 53700 52975 5 8 0 0 0 0 1
device=SWITCH_DIP5
T 53500 52750 5 10 1 1 0 0 1
refdes=SW1
T 53200 50800 5 10 0 0 0 0 1
footprint=DIPSW10
}
N 56500 52700 54800 52700 4
N 54800 52700 54800 52400 4
N 54800 52400 54500 52400 4
N 56500 52400 55000 52400 4
N 54500 52100 55000 52100 4
N 55000 52100 55000 52400 4
N 54500 51800 55300 51800 4
N 55300 51800 55300 52100 4
N 55300 52100 56500 52100 4
N 54500 51500 55600 51500 4
N 55600 51500 55600 51800 4
N 55600 51800 56500 51800 4
N 54500 51200 56000 51200 4
N 56000 51200 56000 51500 4
N 56000 51500 56500 51500 4
N 53200 51000 53200 52400 4
C 53300 50700 1 0 1 gnd-2.sym
C 65200 54600 1 0 0 vcc-1.sym
C 71300 42600 1 0 0 drdc3105e6-1.sym
{
T 72950 44400 5 10 0 0 0 0 1
device=DRDC3105E6
T 71300 42600 5 10 0 0 0 0 1
footprint=SOT26
T 73150 43550 5 10 1 1 0 0 1
refdes=U10
}
C 72200 42200 1 0 1 gnd-2.sym
N 72100 42500 72100 42600 4
N 72100 42600 72600 42600 4
C 64900 47400 1 0 0 lm358-1.sym
{
T 65575 48000 5 10 0 0 0 0 1
device=LM358
T 65600 48750 5 10 0 0 0 0 1
footprint=SO8
T 64900 47400 5 10 0 0 0 0 1
slot=2
T 65100 48300 5 10 1 1 0 0 1
refdes=U7
}
C 68700 47800 1 90 1 2N3904-2.sym
{
T 67794 46896 5 10 0 0 270 2 1
footprint=SOT23
T 68560 47613 5 10 1 1 0 2 1
device=MMBT2222
T 68600 47800 5 10 1 1 0 2 1
refdes=Q4
}
C 68600 48700 1 0 0 resistor-1.sym
{
T 68900 49100 5 10 0 0 0 0 1
device=TE 2-1623788-5
T 68600 48700 5 10 0 0 0 0 1
footprint=SQ-5W
T 68800 49000 5 10 1 1 0 0 1
refdes=R14
T 68800 48500 5 10 1 1 0 0 1
value=0.22
T 68700 48300 5 10 1 1 0 0 1
comment=>= 3W
}
N 68600 48800 67500 48800 4
C 68300 47800 1 90 0 resistor-1.sym
{
T 67900 48100 5 10 0 0 90 0 1
device=RESISTOR
T 68300 47800 5 10 0 0 90 0 1
footprint=0805
T 68000 48000 5 10 1 1 90 0 1
refdes=R13
T 68500 48000 5 10 1 1 90 0 1
value=22
}
N 68200 48700 68200 48800 4
N 67500 47200 67700 47200 4
N 68700 47200 69700 47200 4
N 69700 47200 69700 48800 4
N 69500 48800 71800 48800 4
N 71500 45000 71500 48000 4
C 71900 49700 1 0 0 termblk2-1.sym
{
T 72900 50350 5 10 0 0 0 0 1
device=TERMBLK2
T 71900 49700 5 10 0 1 0 6 1
footprint=TERMBLK2_200MIL
T 72300 49400 5 10 1 1 0 0 1
refdes=J7
}
C 69800 46300 1 90 0 resistor-1.sym
{
T 69400 46600 5 10 0 0 90 0 1
device=RESISTOR
T 69800 46300 5 10 0 0 90 0 1
footprint=0805
T 69500 46500 5 10 1 1 90 0 1
refdes=R12
T 70000 46500 5 10 1 1 90 0 1
value=27k
}
C 69800 45100 1 90 0 resistor-1.sym
{
T 69400 45400 5 10 0 0 90 0 1
device=RESISTOR
T 69800 45100 5 10 0 0 90 0 1
footprint=0805
T 69500 45300 5 10 1 1 90 0 1
refdes=R11
T 70000 45300 5 10 1 1 90 0 1
value=10k
}
N 69700 46000 69700 46300 4
N 69700 45100 69700 45000 4
N 69700 46100 64900 46100 4
N 64900 46100 64900 47600 4
N 65400 48200 65400 49600 4
N 64500 48000 64900 48000 4
C 67500 47800 1 90 0 2N6045G-1.sym
{
T 66800 49200 5 10 0 0 90 0 1
footprint=TO220-3
T 67000 49000 5 10 1 1 0 0 1
device=2N6045G
T 65800 48970 5 10 1 1 0 0 1
refdes=Q3
}
C 72900 46200 1 0 1 relay-DPDT-1.sym
{
T 72900 46200 5 10 0 0 0 6 1
footprint=RELAY-DPDT
T 72600 49050 5 10 1 1 0 6 1
refdes=K2
}
N 71800 46800 70800 46800 4
N 70800 46800 70800 48800 4
N 71500 48000 71800 48000 4
N 71800 47600 71500 47600 4
N 71800 47200 71200 47200 4
N 71200 47200 71200 50300 4
N 71800 48400 71500 48400 4
C 72900 46800 1 0 0 5V-plus-1.sym
{
T 72900 46800 5 10 0 0 0 0 1
netname=+5V
}
N 72900 46700 73100 46700 4
N 73100 46700 73100 46800 4
N 71200 50300 71900 50300 4
N 71900 49900 71500 49900 4
N 71500 49900 71500 48400 4
N 65400 48800 65800 48800 4
N 65900 47800 67500 47800 4
N 67500 47800 67500 47200 4
C 69800 44700 1 0 1 gnd-2.sym
N 71500 45000 69700 45000 4
C 65300 47100 1 0 0 gnd-2.sym
C 65200 49600 1 0 0 vcc-1.sym
T 72800 49700 9 10 1 0 0 0 3
Track
Output
B
N 72900 46300 72900 45400 4
N 72900 45400 72100 45400 4
N 72100 45400 72100 43900 4
N 72600 43900 72600 45000 4
N 72600 45000 73700 45000 4
N 73700 45000 73700 52100 4
N 73700 52100 72900 52100 4
N 73600 43200 73400 43200 4
{
T 74200 43200 5 10 1 1 0 6 1
netname=DIR1
}
N 71100 43200 71300 43200 4
{
T 70500 43200 5 10 1 1 0 0 1
netname=DIR2
}
C 64200 49500 1 270 0 capacitor-1.sym
{
T 64900 49300 5 10 0 1 270 0 1
device=CAPACITOR
T 64300 49600 5 10 1 1 0 0 1
refdes=C13
T 65100 49300 5 10 0 0 270 0 1
symversion=0.1
T 64700 49100 5 10 1 1 0 0 1
value=1uF
T 64500 48300 5 10 0 1 0 0 1
footprint=0805
T 64700 48900 5 10 1 1 0 0 1
comment=50V
}
N 64400 49500 65400 49500 4
C 64300 48300 1 0 0 gnd-2.sym
N 62100 45500 62600 45500 4
N 62600 44300 62600 47800 4
N 62100 44300 62600 44300 4
N 63900 53800 63900 45800 4
N 63900 45800 62100 45800 4
N 62100 44600 64500 44600 4
N 64500 44600 64500 48000 4
N 59300 44500 60300 44500 4
N 60300 44800 59900 44800 4
N 59900 44800 59900 44200 4
N 59900 44200 60300 44200 4
C 60000 43900 1 0 1 gnd-2.sym
N 49900 48400 49900 48800 4
C 46100 44400 1 0 0 hole-1.sym
{
T 46100 44400 5 10 0 1 0 0 1
device=TO220 HEATSINK
T 46300 45000 5 10 1 1 0 4 1
refdes=HS2
T 46100 44400 5 10 0 0 0 0 1
footprint=aavid-heatsink-530002B02500G
}
C 68000 54700 1 0 1 tc74-1.sym
{
T 67500 55250 5 10 1 1 0 0 1
refdes=U8
T 67700 56750 5 10 1 1 0 6 1
device=TC74A0
T 68000 54700 5 10 0 0 0 0 1
footprint=TO220-5
}
C 68100 57900 1 0 0 5V-plus-1.sym
{
T 68100 57900 5 10 0 0 0 0 1
netname=+5V
}
N 68000 56500 68300 56500 4
N 68300 56500 68300 57900 4
C 68200 55300 1 0 0 gnd-2.sym
N 68000 55600 68300 55600 4
N 68000 56200 70000 56200 4
{
T 70100 56100 5 10 1 1 0 0 1
netname=SCL
}
N 68000 55900 70000 55900 4
{
T 70100 55800 5 10 1 1 0 0 1
netname=SDA
}
C 68000 48900 1 0 1 tc74-1.sym
{
T 67500 49450 5 10 1 1 0 0 1
refdes=U9
T 67700 50950 5 10 1 1 0 6 1
device=TC74A1
T 68000 48900 5 10 0 0 0 0 1
footprint=TO220-5
}
C 68100 50800 1 0 0 5V-plus-1.sym
{
T 68100 50800 5 10 0 0 0 0 1
netname=+5V
}
N 68000 50700 68300 50700 4
N 68300 50700 68300 50800 4
C 68200 49500 1 0 0 gnd-2.sym
N 68000 49800 68300 49800 4
N 68000 50400 68300 50400 4
{
T 68500 50300 5 10 1 1 0 0 1
netname=SCL
}
N 68000 50100 68300 50100 4
{
T 68500 50000 5 10 1 1 0 0 1
netname=SDA
}
L 66800 50300 66500 50300 3 0 0 0 -1 -1
L 66500 50300 66500 49300 3 0 0 0 -1 -1
T 65400 50400 9 10 1 0 0 0 2
Thermally coupled
via heatsink HS2
L 66500 56100 66500 55100 3 0 0 0 -1 -1
L 66800 56100 66500 56100 3 0 0 0 -1 -1
T 65400 56200 9 10 1 0 0 0 2
Thermally coupled
via heatsink HS1
N 61300 51800 61500 51800 4
{
T 61700 51700 5 10 1 1 0 0 1
netname=SCL
}
N 61300 52100 61500 52100 4
{
T 61700 52000 5 10 1 1 0 0 1
netname=SDA
}
N 68300 57900 69800 57900 4
N 68900 56200 68900 56700 4
N 61300 54500 61500 54500 4
{
T 61600 54400 5 10 1 1 0 0 1
netname=VPWR
}
C 47400 47900 1 90 0 resistor-1.sym
{
T 47000 48200 5 10 0 0 90 0 1
device=RESISTOR
T 47100 48100 5 10 1 1 90 0 1
refdes=R19
T 47600 48100 5 10 1 1 90 0 1
value=100k
T 47400 47900 5 10 0 0 90 0 1
footprint=0805
}
C 47400 46700 1 90 0 resistor-1.sym
{
T 47000 47000 5 10 0 0 90 0 1
device=RESISTOR
T 47100 46900 5 10 1 1 90 0 1
refdes=R18
T 47600 46900 5 10 1 1 90 0 1
value=10k
T 47400 46700 5 10 0 0 90 0 1
footprint=0805
}
N 47300 47900 47300 47600 4
N 47300 47700 47500 47700 4
{
T 47600 47600 5 10 1 1 0 0 1
netname=VPWR
}
C 61500 57100 1 0 0 5V-plus-1.sym
{
T 61500 57100 5 10 0 0 0 0 1
netname=+5V
}
N 61300 56000 61700 56000 4
N 61700 56000 61700 57100 4
N 70500 50800 70500 51500 4
N 70500 52400 70500 54600 4
N 70400 45000 70400 45700 4
N 70400 46600 70400 48800 4
C 65500 57400 1 0 0 SMP4-BC-1.sym
{
T 66600 58095 5 10 1 1 0 0 1
refdes=D7
T 65500 58495 5 10 0 1 0 0 1
footprint=SMP4-BC-PLCC4
T 66600 57900 5 10 1 1 0 0 1
device=SMP4-BC
}
C 69000 56300 1 90 0 res-pack2-1.sym
{
T 68700 56695 5 10 1 1 90 0 1
refdes=R15
T 69300 56500 5 10 1 1 90 0 1
footprint=RPACK2-0606
T 68700 57300 5 10 1 1 90 0 1
value=2k
}
N 68900 57600 68900 57900 4
C 69900 56400 1 90 0 res-pack2-1.sym
{
T 69600 56695 5 10 1 1 90 0 1
refdes=R15
T 70200 56600 5 10 1 1 90 0 1
footprint=RPACK2-0606
T 69600 57300 5 10 1 1 90 0 1
value=2k
T 69900 56400 5 10 0 0 0 0 1
slot=2
}
N 69800 57700 69800 57900 4
N 69800 56800 69800 55900 4
C 65800 58700 1 90 0 res-pack2-1.sym
{
T 65500 59095 5 10 1 1 90 0 1
refdes=R16
T 66700 58900 5 10 1 1 90 0 1
footprint=RPACK2-0606
T 65500 59700 5 10 1 1 90 0 1
value=1k
}
C 66400 58700 1 90 0 res-pack2-1.sym
{
T 66100 59095 5 10 1 1 90 0 1
refdes=R16
T 66700 58900 5 10 0 1 90 0 1
footprint=RPACK2-0606
T 66100 59700 5 10 1 1 90 0 1
value=1k
T 66400 58700 5 10 0 0 0 0 1
slot=2
}
N 65700 58700 65700 59100 4
N 66300 59100 66300 58700 4
C 65900 56800 1 0 0 gnd-2.sym
N 65700 57400 65700 57100 4
N 65700 57100 66300 57100 4
N 66300 57400 66300 57100 4
C 65800 42600 1 0 0 SMP4-BC-1.sym
{
T 65800 43695 5 10 0 1 0 0 1
footprint=SMP4-BC-PLCC4
T 66900 43295 5 10 1 1 0 0 1
refdes=D9
T 66900 43100 5 10 1 1 0 0 1
device=SMP4-BC
}
C 66100 43700 1 90 0 res-pack2-1.sym
{
T 65800 44095 5 10 1 1 90 0 1
refdes=R17
T 67000 43900 5 10 1 1 90 0 1
footprint=RPACK2-0606
T 65800 44700 5 10 1 1 90 0 1
value=1k
}
C 66700 43700 1 90 0 res-pack2-1.sym
{
T 67000 43900 5 10 0 1 90 0 1
footprint=RPACK2-0606
T 66700 43700 5 10 0 0 0 0 1
slot=2
T 66400 44095 5 10 1 1 90 0 1
refdes=R17
T 66400 44700 5 10 1 1 90 0 1
value=1k
}
N 66000 43900 66000 44100 4
N 66600 44100 66600 43900 4
C 66200 42000 1 0 0 gnd-2.sym
N 66000 42600 66000 42300 4
N 66000 42300 66600 42300 4
N 66600 42600 66600 42300 4
N 65700 60400 65700 60000 4
{
T 65800 60600 5 10 1 1 90 0 1
netname=AL1
}
N 66300 60400 66300 60000 4
{
T 66400 60600 5 10 1 1 90 0 1
netname=AL2
}
N 66600 45400 66600 45000 4
{
T 66700 45600 5 10 1 1 90 0 1
netname=BL2
}
N 66000 45400 66000 45000 4
{
T 66100 45600 5 10 1 1 90 0 1
netname=BL1
}
N 51500 53900 51700 53900 4
{
T 51800 53800 5 10 1 1 0 0 1
netname=BUSON
}
C 41800 48300 1 0 0 pwrjack3-1.sym
{
T 41900 48800 5 10 0 0 0 0 1
device=PWRJACK
T 41800 49000 5 10 1 1 0 0 1
refdes=J4
T 41800 48300 5 10 0 0 0 0 1
footprint=CUI_PJ-202AH
}
C 45300 48000 1 90 1 Cap_H-2.sym
{
T 45000 47800 5 10 1 1 0 6 1
refdes=C10
T 43800 48000 5 10 0 0 270 2 1
device=Capacitor
T 45300 46500 5 10 1 1 0 8 1
value=100uF
T 46100 46100 5 10 1 1 0 6 1
description=50V, Electrolytic
T 45300 48000 5 10 0 0 0 6 1
footprint=cap-elec-Panasonic-FK--D8.00-H10.20-mm
}
N 45100 47100 45100 46700 4
N 45100 48000 45100 48800 4
C 46100 48000 1 90 1 Cap_H-2.sym
{
T 45800 47800 5 10 1 1 0 6 1
refdes=C10A
T 44600 48000 5 10 0 0 270 2 1
device=Capacitor
T 46100 46500 5 10 1 1 0 8 1
value=100uF
T 46100 46100 5 10 0 1 0 6 1
description=50V, Electrolytic
T 46100 48000 5 10 0 0 0 6 1
footprint=cap-elec-Panasonic-FK--D8.00-H10.20-mm
}
N 45900 48000 45900 48800 4
N 45900 47100 45900 46700 4
C 50600 56000 1 270 0 capacitor-1.sym
{
T 51300 55800 5 10 0 1 270 0 1
device=CAPACITOR
T 50900 55700 5 10 1 1 0 0 1
refdes=C14
T 51500 55800 5 10 0 0 270 0 1
symversion=0.1
T 50900 55200 5 10 1 1 0 0 1
value=0.1uF
T 50900 54800 5 10 0 1 0 0 1
footprint=0805
T 50900 55000 5 10 1 1 0 0 1
comment=16V
}
N 50800 56000 51500 56000 4
C 50900 54800 1 0 1 gnd-2.sym
C 46100 48600 1 0 0 diode-1.sym
{
T 46500 49200 5 10 0 0 0 0 1
device=DIODE
T 46400 49300 5 10 1 1 0 0 1
refdes=D2
T 46100 48600 5 10 0 0 0 0 1
footprint=SOD123
T 45900 49100 5 10 1 1 0 0 1
model=MBR0540LT1G
}
C 44700 58600 1 0 0 diode-1.sym
{
T 45100 59200 5 10 0 0 0 0 1
device=DIODE
T 45000 59300 5 10 1 1 0 0 1
refdes=D1
T 44700 58600 5 10 0 0 0 0 1
footprint=SOD123
T 45000 59100 5 10 1 1 0 0 1
model=MBR0540LT1G
}
C 55900 47300 1 0 1 pca9536-1.sym
{
T 55500 50100 5 10 1 1 0 6 1
refdes=U11
T 55000 48400 5 10 1 1 0 6 1
device=PCA9536
T 55500 49900 5 10 0 0 0 6 1
footprint=SO8
}
C 53500 50100 1 0 1 5V-plus-1.sym
{
T 53500 50100 5 10 0 0 0 6 1
netname=+5V
}
N 51700 49800 53500 49800 4
N 53300 49800 53300 50100 4
N 53500 49500 53300 49500 4
{
T 53100 49400 5 10 1 1 0 6 1
netname=SDA
}
N 53500 49200 53300 49200 4
{
T 53100 49100 5 10 1 1 0 6 1
netname=SCL
}
C 53400 48300 1 0 1 gnd-2.sym
N 51700 48900 53500 48900 4
N 53300 48900 53300 48600 4
N 56000 49800 55800 49800 4
{
T 56100 49700 5 10 1 1 0 0 1
netname=AL1
}
N 56000 49500 55800 49500 4
{
T 56100 49400 5 10 1 1 0 0 1
netname=AL2
}
N 56000 49200 55800 49200 4
{
T 56100 49100 5 10 1 1 0 0 1
netname=BL1
}
N 56000 48900 55800 48900 4
{
T 56100 48800 5 10 1 1 0 0 1
netname=BL2
}
N 61300 53300 61500 53300 4
{
T 61600 53200 5 10 1 1 0 0 1
netname=BUSON
}
C 52500 44300 1 0 0 relay-DPDT-1.sym
{
T 52500 44300 5 10 0 0 0 0 1
footprint=RELAY-DPDT
T 52800 47150 5 10 1 1 0 0 1
refdes=K3
}
C 55700 46300 1 0 0 termblk2-1.sym
{
T 56700 46950 5 10 0 0 0 0 1
device=TERMBLK2
T 55700 46300 5 10 0 1 0 6 1
footprint=TERMBLK2_200MIL
T 56600 46600 5 10 1 1 0 0 1
refdes=J8
}
C 55700 45500 1 0 0 termblk2-1.sym
{
T 56700 46150 5 10 0 0 0 0 1
device=TERMBLK2
T 55700 45500 5 10 0 1 0 6 1
footprint=TERMBLK2_200MIL
T 56600 45900 5 10 1 1 0 0 1
refdes=J9
}
C 55700 44700 1 0 0 termblk2-1.sym
{
T 56700 45350 5 10 0 0 0 0 1
device=TERMBLK2
T 55700 44700 5 10 0 1 0 6 1
footprint=TERMBLK2_200MIL
T 56600 45100 5 10 1 1 0 0 1
refdes=J10
}
C 52500 41100 1 0 0 relay-DPDT-1.sym
{
T 52500 41100 5 10 0 0 0 0 1
footprint=RELAY-DPDT
T 52800 43950 5 10 1 1 0 0 1
refdes=K4
}
N 53600 46900 55700 46900 4
N 53600 46500 55700 46500 4
N 53600 46100 55700 46100 4
N 53600 45700 53800 45700 4
N 53800 45700 53800 46900 4
N 54000 46500 54000 45300 4
N 54000 45300 53600 45300 4
N 54200 46100 54200 44900 4
N 54200 44900 53600 44900 4
N 53600 43700 54600 43700 4
N 54600 42500 54600 45700 4
N 54600 45700 55700 45700 4
N 53600 43300 54900 43300 4
N 54900 42100 54900 45300 4
N 54900 45300 55700 45300 4
N 53600 42900 55200 42900 4
N 55200 41700 55200 44900 4
N 55200 44900 55700 44900 4
N 53600 42500 54600 42500 4
N 53600 42100 54900 42100 4
N 53600 41700 55200 41700 4
C 48700 42800 1 0 0 drdc3105e6-1.sym
{
T 50350 44600 5 10 0 0 0 0 1
device=DRDC3105E6
T 48700 42800 5 10 0 0 0 0 1
footprint=SOT26
T 50550 43750 5 10 1 1 0 0 1
refdes=U12
}
N 52500 41200 51800 41200 4
N 51800 41200 51800 44100 4
N 49500 44100 49500 44400 4
N 49500 44400 52500 44400 4
N 50000 44100 51800 44100 4
C 52000 44800 1 0 0 5V-plus-1.sym
{
T 52000 44800 5 10 0 0 0 0 1
netname=+5V
}
N 52200 44800 52500 44800 4
C 49900 42000 1 0 1 gnd-2.sym
N 49500 42800 50000 42800 4
N 49800 42300 49800 42800 4
C 52000 41600 1 0 0 5V-plus-1.sym
{
T 52000 41600 5 10 0 0 0 0 1
netname=+5V
}
N 52200 41600 52500 41600 4
N 48700 43400 48400 43400 4
{
T 47900 43500 5 10 1 1 0 0 1
netname=ACC1
}
N 50800 43400 51100 43400 4
{
T 51600 43300 5 10 1 1 180 0 1
netname=ACC2
}
N 56500 56000 56300 56000 4
{
T 55700 56000 5 10 1 1 0 0 1
netname=ACC2
}
N 56500 56300 56300 56300 4
{
T 55700 56300 5 10 1 1 0 0 1
netname=ACC1
}
C 47800 48200 1 0 0 7805-1.sym
{
T 49400 49500 5 10 0 0 0 0 1
device=R-78E-5.0-0.5
T 49200 49200 5 10 1 1 0 6 1
refdes=U2
T 47800 48200 5 10 0 0 0 0 1
footprint=RECOM-TO220
}
C 51500 49800 1 270 0 capacitor-1.sym
{
T 52200 49600 5 10 0 1 270 0 1
device=CAPACITOR
T 52000 49500 5 10 1 1 0 0 1
refdes=C15
T 52400 49600 5 10 0 0 270 0 1
symversion=0.1
T 52000 49200 5 10 1 1 0 0 1
value=1uF
T 51800 48600 5 10 0 1 0 0 1
footprint=0805
T 52000 49000 5 10 1 1 0 0 1
comment=16V
}
C 63700 55700 1 0 0 mosfet-with-diode-1.sym
{
T 64600 56200 5 10 0 0 180 8 1
device=NPN_TRANSISTOR
T 63900 56600 5 10 1 1 0 8 1
refdes=Q5
T 64800 55000 5 10 1 1 270 8 1
value=IRFML8244TRPBF
T 63700 55700 5 10 0 0 270 8 1
footprint=SOT23_MOS
}
C 64400 55200 1 0 1 gnd-2.sym
N 64300 55700 64300 55500 4
N 61300 53000 62400 53000 4
N 62400 53000 62400 54400 4
N 62400 54400 63400 54400 4
N 63700 56200 63400 56200 4
N 63400 56200 63400 54400 4
C 64200 57100 1 0 1 header2-1.sym
{
T 63200 57750 5 10 0 0 180 2 1
device=HEADER2
T 64600 57500 5 10 1 1 180 2 1
refdes=J11
T 64200 57100 5 10 0 1 270 0 1
footprint=JUMPER2
}
C 64100 58400 1 0 0 5V-plus-1.sym
{
T 64100 58400 5 10 0 0 0 0 1
netname=+5V
}
N 64200 57300 64300 57300 4
N 64300 57300 64300 56700 4
N 64300 58400 64300 57700 4
N 64300 57700 64200 57700 4
T 63200 58000 9 10 1 0 0 0 2
Optional
Fan
C 43800 49800 1 90 0 res-pack2-1.sym
{
T 44000 50195 5 10 1 1 90 0 1
refdes=R20
T 44200 49900 5 10 1 1 90 0 1
footprint=RPACK2-0606
T 44000 50700 5 10 1 1 90 0 1
value=2k
T 43800 49800 5 10 0 0 0 0 1
slot=2
}
C 43400 49800 1 90 0 res-pack2-1.sym
{
T 43100 50195 5 10 1 1 90 0 1
refdes=R20
T 43600 49900 5 10 0 1 90 0 1
footprint=RPACK2-0606
T 43100 50800 5 10 1 1 90 0 1
value=2k
}
N 44600 52600 44600 51200 4
N 44600 51200 44100 51200 4
N 42900 51200 42200 51200 4
N 42200 51200 42200 52200 4
N 43500 53400 43500 52200 4
N 43500 52200 42200 52200 4
C 43600 49900 1 0 0 gnd-1.sym
N 43700 51100 43700 51200 4
N 43300 51100 43300 51200 4
N 48900 55500 48900 49600 4
N 48900 49600 43300 49600 4
N 43300 49600 43300 50200 4
C 42700 51200 1 270 1 header4-1.sym
{
T 43350 52200 5 10 0 0 90 2 1
device=HEADER3
T 44100 52100 5 10 1 1 180 2 1
refdes=JP2
T 42700 51200 5 10 0 0 90 8 1
footprint=JUMPER4-SMT
}
C 70700 45700 1 90 0 schottky-diode-1.sym
{
T 70900 45900 5 10 1 1 90 0 1
device=SK24
T 70100 46000 5 10 1 1 90 0 1
refdes=D8
T 70700 45700 5 10 0 0 90 0 1
footprint=DO-214AA_SMB
}
C 70800 51500 1 90 0 schottky-diode-1.sym
{
T 71000 51700 5 10 1 1 90 0 1
device=SK24
T 70200 51800 5 10 1 1 90 0 1
refdes=D6
T 70800 51500 5 10 0 0 90 0 1
footprint=DO-214AA_SMB
}
