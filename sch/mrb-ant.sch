v 20130925 2
T 42000 54300 9 10 1 0 0 6 1
VIN
T 42000 53900 9 10 1 0 0 6 1
GND
N 57700 50300 57700 50700 4
C 57600 50000 1 0 0 gnd-1.sym
N 54300 56300 53800 56300 4
C 53900 55600 1 0 0 gnd-1.sym
N 54000 55900 53800 55900 4
N 52200 55100 52200 56300 4
N 51900 56700 52400 56700 4
N 52400 55900 51600 55900 4
{
T 51100 56000 5 10 1 1 0 0 1
netname=/RESET
}
T 52600 57200 9 10 1 0 0 0 1
ICSP Header
N 50300 52700 50300 53600 4
C 63200 52600 1 90 0 resistor-1.sym
{
T 62800 52900 5 10 0 0 90 0 1
device=RESISTOR
T 63300 53100 5 10 1 1 270 0 1
refdes=R5
T 62800 53200 5 10 1 1 270 0 1
value=10k
T 63200 52600 5 10 0 0 90 0 1
footprint=0805
}
C 62900 51500 1 270 0 capacitor-1.sym
{
T 63600 51300 5 10 0 1 270 0 1
device=CAPACITOR
T 63200 51200 5 10 1 1 0 0 1
refdes=C5
T 63800 51300 5 10 0 0 270 0 1
symversion=0.1
T 63200 50700 5 10 1 1 0 0 1
value=1uF
T 62900 51500 5 10 0 0 0 0 1
footprint=0805
T 63200 50500 5 10 1 1 0 0 1
description=16V
}
C 63000 50200 1 0 0 gnd-1.sym
N 63100 51500 63100 52600 4
N 63100 53500 63100 58800 4
T 67000 40900 9 10 1 0 0 0 1
I2C Awesome Stick
T 66800 40600 9 10 1 0 0 0 1
mrb-ias.sch
T 67000 40300 9 10 1 0 0 0 1
1
T 68500 40300 9 10 1 0 0 0 1
1
T 70800 40300 9 10 1 0 0 0 1
Michael Petersen
C 40000 40000 0 0 0 title-bordered-D.sym
N 58100 57400 58100 58800 4
N 57700 57400 57700 58800 4
C 60400 55200 1 0 0 gnd-1.sym
N 60500 55500 60500 55700 4
N 60500 55700 60300 55700 4
N 48400 58800 63100 58800 4
N 60300 56000 60600 56000 4
N 60600 56000 60600 58800 4
N 60300 51500 63100 51500 4
{
T 62400 51600 5 10 1 1 0 0 1
netname=/RESET
}
C 54300 54800 1 270 0 crystal-1.sym
{
T 54800 54600 5 10 0 0 270 0 1
device=CRYSTAL
T 54600 54600 5 10 1 1 270 0 1
refdes=Y1
T 55000 54600 5 10 0 0 270 0 1
symversion=0.1
T 53650 54400 5 10 1 1 0 0 1
value=20MHz
T 54300 54800 5 10 0 1 270 0 1
footprint=crystal-hc49
}
C 53900 54300 1 180 0 capacitor-1.sym
{
T 53700 53600 5 10 0 1 180 0 1
device=CAPACITOR
T 53000 53800 5 10 1 1 0 0 1
refdes=C12
T 53700 53400 5 10 0 0 180 0 1
symversion=0.1
T 53800 53900 5 10 1 1 0 0 1
value=22pF
T 53900 54300 5 10 0 0 270 0 1
footprint=0805
T 53400 53700 5 10 1 1 0 0 1
description=16V, NP0
}
N 55100 54500 55500 54500 4
N 53900 54800 55500 54800 4
N 58100 50300 58100 50700 4
C 58000 50000 1 0 0 gnd-1.sym
N 54300 55700 54300 56500 4
{
T 54300 57100 5 10 1 1 270 0 1
netname=MOSI
}
N 50300 53600 55500 53600 4
N 46500 53300 55500 53300 4
N 52000 55100 55500 55100 4
{
T 51400 55100 5 10 1 1 0 0 1
netname=SCLK
}
N 51900 55400 55500 55400 4
N 51900 55400 51900 56700 4
N 54300 55700 55500 55700 4
N 46500 53000 55500 53000 4
N 46400 60300 46800 60300 4
N 63100 50500 63100 50600 4
C 52400 55700 1 0 0 avrprog-1.sym
{
T 52400 57300 5 10 0 1 0 0 1
device=AVRPROG
T 53000 57000 5 10 1 1 0 0 1
refdes=J4
T 52600 55500 5 10 0 1 0 0 1
footprint=JUMPER3x2
}
T 68400 43500 9 10 1 0 0 2 3
Notes:
1) All capacitors are ceramic (X7R/X5R) unless otherwise noted.
2) All capacitors and resistors are 0805 unless otherwise noted.
N 45600 58800 46800 58800 4
N 53800 56700 54000 56700 4
N 54000 56700 54000 58800 4
C 53900 54600 1 0 1 capacitor-1.sym
{
T 53700 55300 5 10 0 1 180 2 1
device=CAPACITOR
T 54000 54900 5 10 1 1 0 6 1
refdes=C13
T 53700 55500 5 10 0 0 180 2 1
symversion=0.1
T 53000 54400 5 10 1 1 0 6 1
value=22pF
T 53900 54600 5 10 0 0 270 6 1
footprint=0805
T 53300 54600 5 10 1 1 0 6 1
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
refdes=C4
T 49800 58400 5 10 0 0 270 0 1
symversion=0.1
T 49200 57800 5 10 1 1 0 0 1
value=1uF
T 49200 57400 5 10 0 1 0 0 1
footprint=0805
T 49200 57600 5 10 1 1 0 0 1
comment=16V
}
C 48600 59000 1 0 0 5V-plus-1.sym
{
T 48600 59000 5 10 0 0 0 0 1
netname=+5V
}
N 48800 59000 48800 58800 4
N 45700 55500 47900 55500 4
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
refdes=R8
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
refdes=R7
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
N 46500 52700 50300 52700 4
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
C 55500 50700 1 0 0 mega328-tqfp32.sym
{
T 60000 57200 5 10 1 1 0 6 1
refdes=U3
T 55800 57500 5 10 0 0 0 0 1
device=ATMega328-TQFP32
T 55800 57700 5 10 0 0 0 0 1
footprint=TQFP32
}
N 52200 56300 52400 56300 4
C 47700 60400 1 180 0 resistor-1.sym
{
T 47400 60000 5 10 0 0 180 0 1
device=RESISTOR
T 47500 60100 5 10 1 1 180 0 1
refdes=R3
T 47500 60600 5 10 1 1 180 0 1
value=20k
T 47700 60400 5 10 0 0 180 0 1
footprint=0805
}
C 62500 54500 1 270 0 resistor-1.sym
{
T 62900 54200 5 10 0 0 270 0 1
device=RESISTOR
T 62800 54300 5 10 1 1 270 0 1
refdes=R4
T 62300 54300 5 10 1 1 270 0 1
value=10k
T 62500 54500 5 10 0 0 270 0 1
footprint=0805
}
C 62500 53300 1 0 0 gnd-1.sym
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
T 45500 58000 5 10 1 1 0 2 1
value=68uF
T 45900 58600 5 10 0 0 0 0 1
footprint=cap-elec-Panasonic-FK--D6.30-H5.80-mm
T 44700 57700 5 10 1 1 0 0 1
description=25V, Electrolytic
}
C 44700 58600 1 0 0 schottky-1.sym
{
T 45022 59272 5 10 0 0 0 0 1
device=DIODE
T 45000 59100 5 10 1 1 0 0 1
refdes=D1
T 45041 59432 5 10 0 1 0 0 1
footprint=SOD123
T 44500 58300 5 10 1 1 0 0 1
model=MBR0540LT1G
}
N 43600 54400 43600 58800 4
N 44200 53400 44200 58200 4
C 69500 42000 1 0 0 hole-1.sym
{
T 69500 42000 5 10 0 1 0 0 1
device=HOLE
T 69700 42600 5 10 1 1 0 4 1
refdes=H1
T 69500 42000 5 10 0 0 0 0 1
footprint=STANDOFF_HEX_n4
}
C 70000 42000 1 0 0 hole-1.sym
{
T 70000 42000 5 10 0 1 0 0 1
device=HOLE
T 70200 42600 5 10 1 1 0 4 1
refdes=H2
T 70000 42000 5 10 0 0 0 0 1
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
C 46100 55500 1 0 0 5V-plus-1.sym
{
T 46100 55500 5 10 0 0 0 0 1
netname=+5V
}
C 45900 55500 1 270 0 capacitor-1.sym
{
T 46600 55300 5 10 0 1 270 0 1
device=CAPACITOR
T 46200 55200 5 10 1 1 0 0 1
refdes=C3
T 46800 55300 5 10 0 0 270 0 1
symversion=0.1
T 46200 54700 5 10 1 1 0 0 1
value=1uF
T 45900 55500 5 10 0 0 0 0 1
footprint=0805
T 46200 54500 5 10 1 1 0 0 1
description=16V
}
C 46000 54300 1 0 0 gnd-1.sym
N 46400 58800 46400 60300 4
N 53900 54100 55100 54100 4
N 55100 54100 55100 54500 4
N 53000 54800 52300 54800 4
N 52300 54800 52300 54100 4
N 52300 54100 53000 54100 4
C 52200 53800 1 0 0 gnd-1.sym
N 60300 54500 62600 54500 4
N 62600 54500 62600 60300 4
N 62600 60300 47700 60300 4
C 58300 58800 1 270 0 capacitor-1.sym
{
T 59000 58600 5 10 0 1 270 0 1
device=CAPACITOR
T 58600 58500 5 10 1 1 0 0 1
refdes=C7
T 59200 58600 5 10 0 0 270 0 1
symversion=0.1
T 58600 58000 5 10 1 1 0 0 1
value=0.1uF
T 58600 57600 5 10 0 1 0 0 1
footprint=0805
T 58600 57800 5 10 1 1 0 0 1
comment=16V
}
C 58400 57600 1 0 0 gnd-1.sym
C 60900 56300 1 270 0 capacitor-1.sym
{
T 61600 56100 5 10 0 1 270 0 1
device=CAPACITOR
T 61200 56000 5 10 1 1 0 0 1
refdes=C8
T 61800 56100 5 10 0 0 270 0 1
symversion=0.1
T 61200 55500 5 10 1 1 0 0 1
value=0.1uF
T 61200 55100 5 10 0 1 0 0 1
footprint=0805
T 61200 55300 5 10 1 1 0 0 1
comment=16V
}
C 61000 55100 1 0 0 gnd-1.sym
N 60300 56300 61100 56300 4
C 70500 42000 1 0 0 hole-1.sym
{
T 70500 42000 5 10 0 1 0 0 1
device=HOLE
T 70700 42600 5 10 1 1 0 4 1
refdes=H3
T 70500 42000 5 10 0 0 0 0 1
footprint=STANDOFF_HEX_n4
}
C 71000 42000 1 0 0 hole-1.sym
{
T 71000 42000 5 10 0 1 0 0 1
device=HOLE
T 71200 42600 5 10 1 1 0 4 1
refdes=H4
T 71000 42000 5 10 0 0 0 0 1
footprint=STANDOFF_HEX_n4
}
C 61400 45200 1 90 0 darlington_NPN-1.sym
{
T 60300 46600 5 10 0 0 90 0 1
device=darlington, NPN
T 60700 46600 5 10 0 0 90 0 1
footprint=TO220
T 59800 46370 5 10 1 1 0 0 1
refdes=Q1
T 59600 46700 5 10 1 1 0 0 1
comment=LARGE heatsink needed!
T 61000 46400 5 10 1 1 0 0 1
device=2N6045G
T 61400 45200 5 10 0 0 0 0 1
footprint=TO220
}
C 58800 43400 1 0 0 lm358-1.sym
{
T 59475 44000 5 10 0 0 0 0 1
device=LM358
T 59500 44750 5 10 0 0 0 0 1
footprint=SO8
T 59000 44300 5 10 1 1 0 0 1
refdes=U8
T 58800 43400 5 10 0 0 0 0 1
slot=2
}
N 60500 45200 60500 43800 4
N 60500 43800 59800 43800 4
C 62600 45000 1 90 1 2N3904-2.sym
{
T 62460 44813 5 10 1 1 0 2 1
device=MMBT2222
T 62500 45000 5 10 1 1 0 2 1
refdes=Q2
T 61694 44096 5 10 0 0 270 2 1
footprint=SOT23
}
C 62500 46100 1 0 0 resistor-1.sym
{
T 62800 46500 5 10 0 0 0 0 1
device=RESISTOR
T 62700 46400 5 10 1 1 0 0 1
refdes=R12
T 62700 45900 5 10 1 1 0 0 1
value=0.22
T 62500 46100 5 10 0 0 0 0 1
footprint=0805
T 62600 45700 5 10 1 1 0 0 1
comment=>= 3W
}
N 62500 46200 61400 46200 4
C 62200 45200 1 90 0 resistor-1.sym
{
T 61800 45500 5 10 0 0 90 0 1
device=RESISTOR
T 61900 45400 5 10 1 1 90 0 1
refdes=R11
T 62400 45400 5 10 1 1 90 0 1
value=22
T 62200 45200 5 10 0 0 90 0 1
footprint=0805
}
N 62100 45000 62100 45200 4
N 62100 46100 62100 46200 4
N 60500 44400 61600 44400 4
N 62600 44400 63600 44400 4
N 63600 44200 63600 46200 4
C 65500 45200 1 0 1 relay-DPDT-2.sym
{
T 65200 46350 5 10 1 1 0 6 1
refdes=K1
T 65500 45200 5 10 0 0 0 0 1
footprint=RELAY-DPDT
}
N 63400 46200 64500 46200 4
N 64500 45600 63600 45600 4
N 64500 46000 64200 46000 4
N 64200 42000 64200 46000 4
N 64200 45800 64500 45800 4
C 66700 46300 1 180 1 termblk2-1.sym
{
T 67700 45650 5 10 0 0 180 6 1
device=TERMBLK2
T 67100 46600 5 10 1 1 180 6 1
refdes=J6
T 66700 46300 5 10 0 1 180 0 1
footprint=TERMBLK2_200MIL
}
N 65500 46100 66700 46100 4
N 66700 45700 65500 45700 4
N 43100 42000 64200 42000 4
N 59300 42000 59300 43400 4
C 63700 43300 1 90 0 resistor-1.sym
{
T 63300 43600 5 10 0 0 90 0 1
device=RESISTOR
T 63400 43500 5 10 1 1 90 0 1
refdes=R10
T 63900 43500 5 10 1 1 90 0 1
value=27k
T 63700 43300 5 10 0 0 90 0 1
footprint=0805
}
C 63700 42100 1 90 0 resistor-1.sym
{
T 63300 42400 5 10 0 0 90 0 1
device=RESISTOR
T 63400 42300 5 10 1 1 90 0 1
refdes=R9
T 63900 42300 5 10 1 1 90 0 1
value=10k
T 63700 42100 5 10 0 0 90 0 1
footprint=0805
}
N 63600 43000 63600 43300 4
N 63600 42100 63600 42000 4
N 63600 43100 58800 43100 4
N 58800 43100 58800 43600 4
N 59300 44200 59300 46200 4
N 43100 46200 59700 46200 4
C 43100 45600 1 0 1 termblk2-1.sym
{
T 42100 46250 5 10 0 0 0 6 1
device=TERMBLK2
T 42700 45300 5 10 1 1 0 6 1
refdes=J5
T 43100 45600 5 10 0 1 0 0 1
footprint=TERMBLK2_200MIL
}
N 43100 42000 43100 45800 4
C 64700 42900 1 0 0 drdc3105e6-1.sym
{
T 66350 44700 5 10 0 0 0 0 1
device=DRDC3105E6
T 66550 43850 5 10 1 1 0 0 1
refdes=U7
T 64700 42900 5 10 0 0 0 0 1
footprint=SOT26
}
C 66100 45200 1 0 0 5V-plus-1.sym
{
T 66100 45200 5 10 0 0 0 0 1
netname=+5V
}
N 65500 45500 66000 45500 4
N 66000 45500 66000 45200 4
N 66000 45200 66300 45200 4
N 65500 45300 65500 44200 4
C 65400 42600 1 0 0 gnd-1.sym
N 64700 43500 64700 42200 4
N 55400 57800 56200 57800 4
{
T 56300 57700 5 10 1 1 0 0 1
netname=DIR_RLY
}
N 55400 57800 55400 56600 4
N 55400 56600 55500 56600 4
N 64700 42200 65500 42200 4
{
T 65600 42100 5 10 1 1 0 0 1
netname=DIR_RLY
}
T 67500 45700 9 10 1 0 0 0 2
Track
Output
T 41600 46100 9 10 1 0 0 0 1
+15-20V
T 41500 45700 9 10 1 0 0 0 1
TRK GND
C 44300 44900 1 90 0 resistor-1.sym
{
T 43900 45200 5 10 0 0 90 0 1
device=RESISTOR
T 44300 44900 5 10 0 0 90 0 1
footprint=0805
T 44000 45100 5 10 1 1 90 0 1
refdes=R1
T 44500 45100 5 10 1 1 90 0 1
value=3k
}
C 44400 43400 1 90 0 led-3.sym
{
T 44400 43400 5 10 0 0 0 0 1
footprint=1206
T 43850 44250 5 10 1 1 90 0 1
refdes=D4
}
N 44200 44900 44200 44300 4
N 44200 45800 44200 46200 4
N 44200 43400 43100 43400 4
C 60600 40700 1 0 0 lm358-1.sym
{
T 61275 41300 5 10 0 0 0 0 1
device=LM358
T 61300 42050 5 10 0 0 0 0 1
footprint=DIP8
T 60800 41600 5 10 1 1 0 0 1
refdes=U8
T 60600 40700 5 10 0 0 0 0 1
slot=1
}
C 48600 44700 1 0 0 78l05-1.sym
{
T 50200 46000 5 10 0 0 0 0 1
device=7805
T 49900 45700 5 10 1 1 0 6 1
refdes=U2
T 48600 44700 5 10 1 1 0 0 1
footprint=SOT89
}
C 48000 45300 1 270 0 Cap_H-2.sym
{
T 48300 45100 5 10 1 1 0 0 1
refdes=C3
T 49500 45300 5 10 0 0 270 0 1
device=Capacitor
T 47500 44900 5 10 1 1 0 2 1
value=68uF
T 48000 45300 5 10 0 0 0 0 1
footprint=cap-elec-Panasonic-FK--D6.30-H5.80-mm
T 46800 44500 5 10 1 1 0 0 1
description=25V, Electrolytic
}
C 46900 45100 1 0 0 schottky-1.sym
{
T 47222 45772 5 10 0 0 0 0 1
device=DIODE
T 47200 45800 5 10 1 1 0 0 1
refdes=D2
T 47241 45932 5 10 0 1 0 0 1
footprint=SOD123
T 47000 45600 5 10 1 1 0 0 1
model=MBR0540LT1G
}
N 47800 45300 48600 45300 4
N 46900 45300 46900 46200 4
N 49400 44700 49400 42000 4
N 48200 44400 48200 42000 4
C 50500 45300 1 270 0 capacitor-1.sym
{
T 51200 45100 5 10 0 1 270 0 1
device=CAPACITOR
T 50400 45000 5 10 1 1 0 0 1
refdes=C6
T 51400 45100 5 10 0 0 270 0 1
symversion=0.1
T 50300 44500 5 10 1 1 0 0 1
value=1uF
T 50800 44100 5 10 0 1 0 0 1
footprint=0805
T 50300 44300 5 10 1 1 0 0 1
comment=16V
}
N 50200 45300 50700 45300 4
N 50700 44400 50700 44200 4
N 50700 44200 49400 44200 4
C 55900 42500 1 0 0 mcp49x1-1.sym
{
T 56300 45300 5 10 1 1 0 0 1
refdes=U5
T 56700 45300 5 10 1 1 0 0 1
device=MCP4911
T 56300 45100 5 10 0 0 0 0 1
footprint=SO8
}
N 57800 44000 58800 44000 4
C 52900 42500 1 0 0 si8640-1.sym
{
T 53300 45500 5 10 1 1 0 0 1
refdes=U6
T 53900 45500 5 10 1 1 0 0 1
device=Si8640BB-B
T 53300 45100 5 10 0 0 0 0 1
footprint=SO16
}
N 54900 43000 54900 42000 4
C 51400 45400 1 0 0 5V-plus-1.sym
{
T 51400 45400 5 10 0 0 0 0 1
netname=+5V
}
N 55200 42900 55200 45900 4
N 50700 45900 58400 45900 4
N 50700 45900 50700 45300 4
N 56000 45900 56000 45000 4
N 57800 45900 57800 44500 4
N 54900 44700 56000 44700 4
N 54900 44400 56000 44400 4
N 54900 44100 56000 44100 4
N 56000 41900 56000 43800 4
C 52900 42300 1 0 0 gnd-1.sym
N 53000 43000 53000 42600 4
C 58200 45900 1 270 0 capacitor-1.sym
{
T 58900 45700 5 10 0 1 270 0 1
device=CAPACITOR
T 58500 45600 5 10 1 1 0 0 1
refdes=C10
T 59100 45700 5 10 0 0 270 0 1
symversion=0.1
T 58500 45100 5 10 1 1 0 0 1
value=0.1uF
T 58500 44700 5 10 0 1 0 0 1
footprint=0805
T 58500 44800 5 10 1 1 0 0 1
comment=16V
}
N 58400 45000 58400 42000 4
C 55000 42900 1 270 0 capacitor-1.sym
{
T 55700 42700 5 10 0 1 270 0 1
device=CAPACITOR
T 55300 42100 5 10 1 1 0 0 1
refdes=C11
T 55900 42700 5 10 0 0 270 0 1
symversion=0.1
T 55000 41600 5 10 1 1 0 0 1
value=0.1uF
T 55300 41700 5 10 0 1 0 0 1
footprint=0805
T 55000 41300 5 10 1 1 0 0 1
comment=16V
}
N 54900 45200 55200 45200 4
C 51400 45400 1 270 0 capacitor-1.sym
{
T 52100 45200 5 10 0 1 270 0 1
device=CAPACITOR
T 51300 45100 5 10 1 1 0 0 1
refdes=C9
T 52300 45200 5 10 0 0 270 0 1
symversion=0.1
T 51700 45100 5 10 1 1 0 0 1
value=0.1uF
T 51700 44200 5 10 0 1 0 0 1
footprint=0805
T 51200 44400 5 10 1 1 0 0 1
comment=16V
}
C 51500 44200 1 0 0 gnd-1.sym
N 51600 45400 53000 45400 4
N 53000 45400 53000 45200 4
N 52800 44400 53000 44400 4
{
T 52200 44400 5 10 1 1 0 0 1
netname=SCLK
}
N 52800 44700 53000 44700 4
{
T 52200 44700 5 10 1 1 0 0 1
netname=/CS1
}
N 52800 44100 53000 44100 4
{
T 52200 44100 5 10 1 1 0 0 1
netname=MOSI
}
C 52400 43500 1 0 0 5V-plus-1.sym
{
T 52400 43500 5 10 0 0 0 0 1
netname=+5V
}
N 52600 43500 53000 43500 4
N 53000 43500 53000 43800 4
N 55300 52700 55500 52700 4
{
T 54700 52700 5 10 1 1 0 0 1
netname=/CS1
}
C 44700 46200 1 0 0 vcc-1.sym
C 60900 41600 1 0 0 vcc-1.sym
N 61100 41600 61100 41500 4
N 60600 40900 60600 40400 4
N 60600 40400 61600 40400 4
N 61600 41100 61600 40400 4
N 61100 40700 61100 40600 4
N 61100 40600 60100 40600 4
N 60100 40600 60100 42000 4
N 60100 41300 60600 41300 4
N 54900 43500 55200 43500 4
C 45800 44400 1 270 0 Cap_H-2.sym
{
T 46100 44200 5 10 1 1 0 0 1
refdes=C2
T 47300 44400 5 10 0 0 270 0 1
device=Capacitor
T 45200 44000 5 10 1 1 0 2 1
value=560uF
T 45800 44400 5 10 0 0 0 0 1
footprint=cap-elec-Panasonic-FK--D6.30-H5.80-mm
T 44600 43600 5 10 1 1 0 0 1
description=35V, Electrolytic
}
N 46000 44400 46000 46200 4
N 46000 43500 46000 42000 4
