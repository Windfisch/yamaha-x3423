EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L bluepill:BlueBlackPill A2
U 1 1 60C7FA8A
P 3950 3275
F 0 "A2" H 3725 2325 60  0000 C CNN
F 1 "BluePill" H 3725 2200 60  0000 C CNN
F 2 "footprints:BLUEPILL-blackpill" H 3850 4025 60  0001 C CNN
F 3 "" H 3850 4025 60  0001 C CNN
	1    3950 3275
	1    0    0    -1  
$EndComp
$Comp
L power:VCC #PWR037
U 1 1 60C80D15
P 3950 2225
F 0 "#PWR037" H 3950 2075 50  0001 C CNN
F 1 "VCC" H 3965 2398 50  0000 C CNN
F 2 "" H 3950 2225 50  0001 C CNN
F 3 "" H 3950 2225 50  0001 C CNN
	1    3950 2225
	1    0    0    -1  
$EndComp
Wire Wire Line
	925  1925 2225 1925
Wire Wire Line
	925  2025 2150 2025
Wire Wire Line
	925  2125 2075 2125
Wire Wire Line
	925  2225 2000 2225
Wire Wire Line
	925  2325 1925 2325
$Comp
L Connector_Generic:Conn_01x34 J11
U 1 1 60C7C3D1
P 725 3625
F 0 "J11" H 643 1700 50  0000 C CNN
F 1 "Conn_01x34" H 643 1791 50  0000 C CNN
F 2 "footprints:Molex_52044_3445" H 725 3625 50  0001 C CNN
F 3 "~" H 725 3625 50  0001 C CNN
	1    725  3625
	-1   0    0    1   
$EndComp
Wire Wire Line
	925  2625 1175 2625
Wire Wire Line
	925  2425 1025 2425
Wire Wire Line
	1025 2425 1025 2525
Wire Wire Line
	1025 3425 925  3425
Wire Wire Line
	925  3325 1025 3325
Connection ~ 1025 3325
Wire Wire Line
	1025 3325 1025 3425
Wire Wire Line
	925  3225 1025 3225
Connection ~ 1025 3225
Wire Wire Line
	1025 3225 1025 3325
Wire Wire Line
	925  3125 1025 3125
Connection ~ 1025 3125
Wire Wire Line
	1025 3125 1025 3225
Wire Wire Line
	925  2525 1025 2525
Connection ~ 1025 2525
Wire Wire Line
	1025 2525 1025 3125
Wire Wire Line
	925  2725 925  2775
Wire Wire Line
	925  2925 925  2975
Connection ~ 925  2975
Wire Wire Line
	925  2975 925  3025
Wire Wire Line
	925  2775 1175 2775
Connection ~ 925  2775
Wire Wire Line
	925  2775 925  2825
Wire Wire Line
	925  2975 1175 2975
$Comp
L power:GNDA #PWR032
U 1 1 60CA27CF
P 1175 3175
F 0 "#PWR032" H 1175 2925 50  0001 C CNN
F 1 "GNDA" H 1350 3175 50  0000 C CNN
F 2 "" H 1175 3175 50  0001 C CNN
F 3 "" H 1175 3175 50  0001 C CNN
	1    1175 3175
	1    0    0    -1  
$EndComp
Wire Wire Line
	1175 3175 1175 3125
Wire Wire Line
	1175 3125 1025 3125
$Comp
L power:+15V #PWR030
U 1 1 60CA59A9
P 1175 2775
F 0 "#PWR030" H 1175 2625 50  0001 C CNN
F 1 "+15V" H 1325 2825 50  0000 C CNN
F 2 "" H 1175 2775 50  0001 C CNN
F 3 "" H 1175 2775 50  0001 C CNN
	1    1175 2775
	1    0    0    -1  
$EndComp
$Comp
L power:-15V #PWR031
U 1 1 60CA63E6
P 1175 2975
F 0 "#PWR031" H 1175 3075 50  0001 C CNN
F 1 "-15V" H 1025 2975 50  0000 C CNN
F 2 "" H 1175 2975 50  0001 C CNN
F 3 "" H 1175 2975 50  0001 C CNN
	1    1175 2975
	-1   0    0    1   
$EndComp
Entry Wire Line
	1175 3725 1275 3825
Entry Wire Line
	1175 3825 1275 3925
Entry Wire Line
	1175 4025 1275 4125
Entry Wire Line
	1175 4125 1275 4225
Entry Wire Line
	1175 4325 1275 4425
Entry Wire Line
	1175 4425 1275 4525
Entry Wire Line
	1175 4525 1275 4625
Entry Wire Line
	1175 4625 1275 4725
Entry Wire Line
	1175 4825 1275 4925
Entry Wire Line
	1175 4925 1275 5025
Entry Wire Line
	1175 5025 1275 5125
Entry Wire Line
	1175 5125 1275 5225
Wire Wire Line
	925  3725 1175 3725
Wire Wire Line
	1175 3825 925  3825
Wire Wire Line
	1175 4025 925  4025
Wire Wire Line
	925  4125 1175 4125
Wire Wire Line
	925  3625 1025 3625
Wire Wire Line
	1025 3625 1025 3925
Wire Wire Line
	1025 3925 925  3925
Wire Wire Line
	1025 3925 1025 4225
Wire Wire Line
	1025 4225 925  4225
Connection ~ 1025 3925
Wire Wire Line
	1025 4225 1025 4725
Wire Wire Line
	1025 4725 925  4725
Connection ~ 1025 4225
Wire Wire Line
	1025 4725 1025 5225
Wire Wire Line
	1025 5225 925  5225
Connection ~ 1025 4725
$Comp
L power:GND #PWR028
U 1 1 60CCA555
P 1025 5225
F 0 "#PWR028" H 1025 4975 50  0001 C CNN
F 1 "GND" H 1030 5052 50  0000 C CNN
F 2 "" H 1025 5225 50  0001 C CNN
F 3 "" H 1025 5225 50  0001 C CNN
	1    1025 5225
	1    0    0    -1  
$EndComp
Connection ~ 1025 5225
Entry Wire Line
	1175 3525 1275 3625
Wire Wire Line
	1175 3525 925  3525
Wire Wire Line
	1175 4325 925  4325
Wire Wire Line
	925  4425 1175 4425
Wire Wire Line
	1175 4525 925  4525
Wire Wire Line
	925  4625 1175 4625
Wire Wire Line
	1175 4825 925  4825
Wire Wire Line
	925  4925 1175 4925
Wire Wire Line
	1175 5025 925  5025
Wire Wire Line
	925  5125 1175 5125
Text Label 1075 3525 0    50   ~ 0
reset
Text Label 1075 3725 0    50   ~ 0
fd0
Text Label 1075 3825 0    50   ~ 0
fd1
Text Label 1075 4025 0    50   ~ 0
fd2
Text Label 1075 4125 0    50   ~ 0
fd3
Text Label 1075 4325 0    50   ~ 0
d0
Text Label 1075 4425 0    50   ~ 0
d1
Text Label 1075 4525 0    50   ~ 0
d2
Text Label 1075 4625 0    50   ~ 0
d3
Text Label 1075 4825 0    50   ~ 0
d4
Text Label 1075 4925 0    50   ~ 0
d5
Text Label 1075 5025 0    50   ~ 0
d6
Text Label 1075 5125 0    50   ~ 0
d7
Entry Wire Line
	4950 3475 5050 3575
Entry Wire Line
	4950 3575 5050 3675
Entry Wire Line
	4950 4075 5050 4175
Entry Wire Line
	2950 3875 2850 3975
Entry Wire Line
	2950 3775 2850 3875
Entry Wire Line
	4950 2475 5050 2575
Entry Wire Line
	4950 2575 5050 2675
Entry Wire Line
	4950 2675 5050 2775
Entry Wire Line
	4950 2775 5050 2875
Entry Wire Line
	4950 3675 5050 3775
Wire Wire Line
	4950 3675 4700 3675
Wire Wire Line
	4950 2975 4700 2975
Wire Wire Line
	4700 2775 4950 2775
Wire Wire Line
	4950 2675 4700 2675
Wire Wire Line
	4700 2575 4950 2575
Wire Wire Line
	4950 2475 4700 2475
Wire Wire Line
	3200 3775 2950 3775
Wire Wire Line
	2950 3875 3200 3875
Wire Wire Line
	4700 4075 4950 4075
Text Label 4850 3475 0    50   ~ 0
fd2
Text Label 4850 2975 0    50   ~ 0
d0
Text Label 4850 2775 0    50   ~ 0
d1
Text Label 4850 2675 0    50   ~ 0
d2
Text Label 4850 2575 0    50   ~ 0
d3
Text Label 4850 2475 0    50   ~ 0
d4
Text Label 3050 3775 2    50   ~ 0
d5
Text Label 3050 3875 2    50   ~ 0
d6
Text Label 4850 4075 0    50   ~ 0
d7
Text GLabel 4700 3975 2    50   Input ~ 0
sck
Text GLabel 4700 3775 2    50   Input ~ 0
sda
NoConn ~ 4700 3375
NoConn ~ 4700 3275
$Comp
L Connector_Generic:Conn_01x03 J13
U 1 1 60DA5BBE
P 5325 1975
F 0 "J13" V 5289 1787 50  0000 R CNN
F 1 "Conn_01x03" V 5198 1787 50  0000 R CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Horizontal" H 5325 1975 50  0001 C CNN
F 3 "~" H 5325 1975 50  0001 C CNN
	1    5325 1975
	0    -1   -1   0   
$EndComp
Text Label 4850 3075 0    50   ~ 0
reset
Text Label 4850 3175 0    50   ~ 0
fd3
Wire Wire Line
	4700 3075 4950 3075
Wire Wire Line
	4950 3175 4700 3175
Entry Wire Line
	4950 2975 5050 3075
Entry Wire Line
	4950 3175 5050 3275
Entry Wire Line
	4950 3075 5050 3175
Wire Wire Line
	4950 3475 4775 3475
Wire Wire Line
	4700 3575 4775 3575
Wire Wire Line
	4775 3475 4775 3400
Connection ~ 4775 3475
Wire Wire Line
	4775 3475 4700 3475
Wire Wire Line
	5225 3400 5225 2175
Wire Wire Line
	4775 3575 4775 3500
Connection ~ 4775 3575
Wire Wire Line
	4775 3575 4950 3575
Wire Wire Line
	5325 3500 5325 2175
Text Label 4850 3675 0    50   ~ 0
fd0
Text Label 4850 3575 0    50   ~ 0
fd1
$Comp
L 74xx:74HC595 U1
U 1 1 60E8BAE3
P 3575 6150
F 0 "U1" H 3575 6931 50  0000 C CNN
F 1 "74HC595" H 3575 6840 50  0000 C CNN
F 2 "Package_DIP:DIP-16_W7.62mm" H 3575 6150 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/sn74hc595.pdf" H 3575 6150 50  0001 C CNN
	1    3575 6150
	1    0    0    -1  
$EndComp
Text GLabel 3975 5750 2    50   Input ~ 0
cs0
Text GLabel 3975 5850 2    50   Input ~ 0
cs1
Text GLabel 3975 5950 2    50   Input ~ 0
cs2
Text GLabel 3975 6050 2    50   Input ~ 0
cs3
Text GLabel 3975 6150 2    50   Input ~ 0
cs4
Text GLabel 3975 6250 2    50   Input ~ 0
cs5
Text GLabel 3975 6350 2    50   Input ~ 0
cs6
Text GLabel 3975 6450 2    50   Input ~ 0
cs7
Text GLabel 3200 2575 0    50   Input ~ 0
cs8
Wire Bus Line
	1275 5225 2850 5225
Wire Wire Line
	2550 5750 3175 5750
Wire Wire Line
	2450 5950 3175 5950
Wire Wire Line
	2350 6250 3175 6250
Wire Wire Line
	3575 5550 3275 5550
Wire Wire Line
	3000 5550 3000 6050
Wire Wire Line
	3000 6050 3175 6050
Wire Wire Line
	3175 6350 3175 6850
Wire Wire Line
	3175 6850 3575 6850
$Comp
L power:GND #PWR036
U 1 1 60EE6609
P 3575 6850
F 0 "#PWR036" H 3575 6600 50  0001 C CNN
F 1 "GND" H 3580 6677 50  0000 C CNN
F 2 "" H 3575 6850 50  0001 C CNN
F 3 "" H 3575 6850 50  0001 C CNN
	1    3575 6850
	1    0    0    -1  
$EndComp
Connection ~ 3575 6850
$Comp
L power:VCC #PWR035
U 1 1 60EE7195
P 3275 5550
F 0 "#PWR035" H 3275 5400 50  0001 C CNN
F 1 "VCC" H 3290 5723 50  0000 C CNN
F 2 "" H 3275 5550 50  0001 C CNN
F 3 "" H 3275 5550 50  0001 C CNN
	1    3275 5550
	1    0    0    -1  
$EndComp
Connection ~ 3275 5550
Wire Wire Line
	3275 5550 3000 5550
NoConn ~ 3975 6650
Connection ~ 2850 5225
Wire Bus Line
	2850 5225 5050 5225
NoConn ~ 3200 2475
Text GLabel 7600 2175 0    50   Input ~ 0
cs_dac
Text GLabel 3200 2675 0    50   Input ~ 0
cs_dac
Text GLabel 7600 2275 0    50   Input ~ 0
sck
Text GLabel 7600 2375 0    50   Input ~ 0
sda
$Comp
L Analog_DAC:MCP4811 U3
U 1 1 60F04605
P 8100 2175
F 0 "U3" H 8525 2375 50  0000 L CNN
F 1 "MCP4811" H 7975 2175 50  0000 L CNN
F 2 "Package_DIP:DIP-8_W7.62mm" H 9000 2075 50  0001 C CNN
F 3 "http://ww1.microchip.com/downloads/en/DeviceDoc/22244B.pdf" H 9000 2075 50  0001 C CNN
	1    8100 2175
	1    0    0    -1  
$EndComp
Wire Wire Line
	7600 2075 7200 2075
Wire Wire Line
	7200 2075 7200 2575
Wire Wire Line
	7200 2575 8100 2575
$Comp
L power:GND #PWR047
U 1 1 60F1F258
P 8100 2575
F 0 "#PWR047" H 8100 2325 50  0001 C CNN
F 1 "GND" H 8105 2402 50  0000 C CNN
F 2 "" H 8100 2575 50  0001 C CNN
F 3 "" H 8100 2575 50  0001 C CNN
	1    8100 2575
	1    0    0    -1  
$EndComp
Connection ~ 8100 2575
$Comp
L power:VCC #PWR046
U 1 1 60F1FB18
P 8100 1775
F 0 "#PWR046" H 8100 1625 50  0001 C CNN
F 1 "VCC" H 8115 1948 50  0000 C CNN
F 2 "" H 8100 1775 50  0001 C CNN
F 3 "" H 8100 1775 50  0001 C CNN
	1    8100 1775
	1    0    0    -1  
$EndComp
$Comp
L Jumper:Jumper_2_Bridged JP4
U 1 1 60F205EA
P 7400 1975
F 0 "JP4" H 7400 2078 50  0000 C CNN
F 1 "Jumper_2_Bridged" H 7400 2079 50  0001 C CNN
F 2 "footprints:SolderDipJumper2alt" H 7400 1975 50  0001 C CNN
F 3 "~" H 7400 1975 50  0001 C CNN
	1    7400 1975
	1    0    0    -1  
$EndComp
Wire Wire Line
	8100 1775 7200 1775
Wire Wire Line
	7200 1775 7200 1975
Connection ~ 8100 1775
$Comp
L Switch:SW_Push SW1
U 1 1 60F4B166
P 6025 3375
F 0 "SW1" H 6025 3660 50  0000 C CNN
F 1 "SW_Push" H 6025 3569 50  0000 C CNN
F 2 "Button_Switch_THT:SW_Tactile_Straight_KSL0Axx1LFTR" H 6025 3575 50  0001 C CNN
F 3 "~" H 6025 3575 50  0001 C CNN
	1    6025 3375
	-1   0    0    -1  
$EndComp
$Comp
L power:GND #PWR040
U 1 1 60F5511A
P 6225 3375
F 0 "#PWR040" H 6225 3125 50  0001 C CNN
F 1 "GND" H 6230 3202 50  0000 C CNN
F 2 "" H 6225 3375 50  0001 C CNN
F 3 "" H 6225 3375 50  0001 C CNN
	1    6225 3375
	-1   0    0    -1  
$EndComp
Wire Wire Line
	5825 2875 5825 3375
$Comp
L Amplifier_Operational:LM324 U4
U 1 1 60F5EE6A
P 9325 1725
F 0 "U4" H 9400 1925 50  0000 C CNN
F 1 "LM324" H 9425 1575 50  0000 C CNN
F 2 "Package_DIP:DIP-14_W7.62mm" H 9275 1825 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/lm2902-n.pdf" H 9375 1925 50  0001 C CNN
	1    9325 1725
	1    0    0    -1  
$EndComp
$Comp
L Amplifier_Operational:LM324 U4
U 5 1 60F617A5
P 9325 1725
F 0 "U4" H 9283 1725 50  0001 L CNN
F 1 "LM324" H 9283 1680 50  0001 L CNN
F 2 "Package_DIP:DIP-14_W7.62mm" H 9275 1825 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/lm2902-n.pdf" H 9375 1925 50  0001 C CNN
	5    9325 1725
	1    0    0    -1  
$EndComp
$Comp
L power:+15V #PWR049
U 1 1 60F64F72
P 9225 1425
F 0 "#PWR049" H 9225 1275 50  0001 C CNN
F 1 "+15V" H 9240 1598 50  0000 C CNN
F 2 "" H 9225 1425 50  0001 C CNN
F 3 "" H 9225 1425 50  0001 C CNN
	1    9225 1425
	1    0    0    -1  
$EndComp
$Comp
L power:-15V #PWR050
U 1 1 60F657C1
P 9225 2025
F 0 "#PWR050" H 9225 2125 50  0001 C CNN
F 1 "-15V" H 9240 2198 50  0000 C CNN
F 2 "" H 9225 2025 50  0001 C CNN
F 3 "" H 9225 2025 50  0001 C CNN
	1    9225 2025
	-1   0    0    1   
$EndComp
$Comp
L Device:R R11
U 1 1 60F8B57F
P 9625 2525
F 0 "R11" H 9555 2479 50  0000 R CNN
F 1 "2.2k" H 9555 2570 50  0000 R CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 9555 2525 50  0001 C CNN
F 3 "~" H 9625 2525 50  0001 C CNN
	1    9625 2525
	-1   0    0    1   
$EndComp
$Comp
L Device:R R9
U 1 1 60F9275C
P 9625 1875
F 0 "R9" H 9555 1829 50  0000 R CNN
F 1 "1k" H 9555 1920 50  0000 R CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 9555 1875 50  0001 C CNN
F 3 "~" H 9625 1875 50  0001 C CNN
	1    9625 1875
	-1   0    0    1   
$EndComp
Text GLabel 1175 2625 2    50   Input ~ 0
dac_out
Text GLabel 9625 1725 2    50   Input ~ 0
dac_out
$Comp
L power:GND #PWR039
U 1 1 60FA452F
P 5425 2175
F 0 "#PWR039" H 5425 1925 50  0001 C CNN
F 1 "GND" H 5430 2002 50  0000 C CNN
F 2 "" H 5425 2175 50  0001 C CNN
F 3 "" H 5425 2175 50  0001 C CNN
	1    5425 2175
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR038
U 1 1 60FAB249
P 4100 4325
F 0 "#PWR038" H 4100 4075 50  0001 C CNN
F 1 "GND" H 4105 4152 50  0000 C CNN
F 2 "" H 4100 4325 50  0001 C CNN
F 3 "" H 4100 4325 50  0001 C CNN
	1    4100 4325
	1    0    0    -1  
$EndComp
NoConn ~ 4100 2225
NoConn ~ 3800 2225
$Comp
L power:GNDA #PWR024
U 1 1 60FE51F0
P 750 6600
F 0 "#PWR024" H 750 6350 50  0001 C CNN
F 1 "GNDA" H 700 6425 50  0000 C CNN
F 2 "" H 750 6600 50  0001 C CNN
F 3 "" H 750 6600 50  0001 C CNN
	1    750  6600
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR027
U 1 1 60FE563C
P 950 6600
F 0 "#PWR027" H 950 6350 50  0001 C CNN
F 1 "GND" H 1000 6425 50  0000 C CNN
F 2 "" H 950 6600 50  0001 C CNN
F 3 "" H 950 6600 50  0001 C CNN
	1    950  6600
	1    0    0    -1  
$EndComp
Text Notes 625  5100 1    50   ~ 0
Note: the connector is reversed
Wire Wire Line
	2225 3275 3200 3275
Wire Wire Line
	2150 3375 3200 3375
Wire Wire Line
	2075 3475 3200 3475
Wire Wire Line
	2000 3575 3200 3575
Wire Wire Line
	1925 3675 3200 3675
Wire Wire Line
	4775 3400 5225 3400
Wire Wire Line
	4775 3500 5325 3500
Text GLabel 3200 3175 0    50   Input ~ 0
PA4
Text GLabel 3200 3075 0    50   Input ~ 0
PA3
Text GLabel 3200 2975 0    50   Input ~ 0
PA2
Text GLabel 6275 6100 0    50   Input ~ 0
PA4
Text GLabel 6275 5600 0    50   Input ~ 0
PA3
Text GLabel 6275 5100 0    50   Input ~ 0
PA2
$Comp
L Jumper:Jumper_3_Bridged12 JP2
U 1 1 6109D3B0
P 6275 5350
F 0 "JP2" V 6275 5475 50  0000 L CNN
F 1 "Jumper_3_Bridged12" V 6230 5417 50  0001 L CNN
F 2 "footprints:SolderDipJumper3alt" H 6275 5350 50  0001 C CNN
F 3 "~" H 6275 5350 50  0001 C CNN
	1    6275 5350
	0    -1   1    0   
$EndComp
$Comp
L Jumper:Jumper_3_Bridged12 JP3
U 1 1 610A5AB0
P 6275 5850
F 0 "JP3" V 6275 5975 50  0000 L CNN
F 1 "Jumper_3_Bridged12" V 6230 5917 50  0001 L CNN
F 2 "footprints:SolderDipJumper3alt" H 6275 5850 50  0001 C CNN
F 3 "~" H 6275 5850 50  0001 C CNN
	1    6275 5850
	0    -1   1    0   
$EndComp
Wire Wire Line
	2225 1925 2225 3275
Wire Wire Line
	2075 2125 2075 3475
Wire Wire Line
	2150 2025 2150 3375
Wire Wire Line
	2000 2225 2000 3575
Wire Wire Line
	1925 2325 1925 3675
Wire Wire Line
	2550 2875 3200 2875
Wire Wire Line
	2550 2875 2550 5750
Wire Wire Line
	2450 2775 3200 2775
$Comp
L Jumper:Jumper_3_Bridged12 JP1
U 1 1 610F869E
P 2450 4075
F 0 "JP1" V 2575 3850 50  0000 L CNN
F 1 "Jumper_3_Bridged12" V 2405 4142 50  0001 L CNN
F 2 "footprints:SolderDipJumper3alt" H 2450 4075 50  0001 C CNN
F 3 "~" H 2450 4075 50  0001 C CNN
	1    2450 4075
	1    0    0    -1  
$EndComp
Wire Wire Line
	3200 4075 2700 4075
Wire Wire Line
	2200 4075 2200 3925
Wire Wire Line
	2200 3925 2450 3925
Wire Wire Line
	2450 3925 2450 2775
Wire Wire Line
	2450 4225 2450 5950
Wire Wire Line
	2350 6250 2350 5275
Wire Wire Line
	2350 5275 5125 5275
Wire Wire Line
	5125 5275 5125 3875
Wire Wire Line
	5125 3875 4700 3875
$Comp
L Isolator:6N137 U2
U 1 1 6112AB58
P 7650 5850
F 0 "U2" H 7650 6317 50  0000 C CNN
F 1 "6N137" H 7650 6226 50  0000 C CNN
F 2 "Package_DIP:DIP-8_W7.62mm" H 7650 5350 50  0001 C CNN
F 3 "https://docs.broadcom.com/docs/AV02-0940EN" H 6800 6400 50  0001 C CNN
	1    7650 5850
	-1   0    0    -1  
$EndComp
$Comp
L power:VCC #PWR043
U 1 1 6113022A
P 7350 5650
F 0 "#PWR043" H 7350 5500 50  0001 C CNN
F 1 "VCC" H 7365 5823 50  0000 C CNN
F 2 "" H 7350 5650 50  0001 C CNN
F 3 "" H 7350 5650 50  0001 C CNN
	1    7350 5650
	1    0    0    -1  
$EndComp
NoConn ~ 7350 5750
$Comp
L power:GND #PWR044
U 1 1 6113899F
P 7350 6050
F 0 "#PWR044" H 7350 5800 50  0001 C CNN
F 1 "GND" H 7355 5877 50  0000 C CNN
F 2 "" H 7350 6050 50  0001 C CNN
F 3 "" H 7350 6050 50  0001 C CNN
	1    7350 6050
	1    0    0    -1  
$EndComp
Wire Wire Line
	7350 5850 7000 5850
$Comp
L Device:R R4
U 1 1 6114C764
P 6875 5700
F 0 "R4" H 6806 5654 50  0000 R CNN
F 1 "R" H 6806 5745 50  0000 R CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 6805 5700 50  0001 C CNN
F 3 "~" H 6875 5700 50  0001 C CNN
	1    6875 5700
	1    0    0    1   
$EndComp
Connection ~ 6875 5850
Wire Wire Line
	6875 5850 6425 5850
$Comp
L power:VCC #PWR042
U 1 1 6114CE27
P 6925 5250
F 0 "#PWR042" H 6925 5100 50  0001 C CNN
F 1 "VCC" H 6925 5400 50  0000 C CNN
F 2 "" H 6925 5250 50  0001 C CNN
F 3 "" H 6925 5250 50  0001 C CNN
	1    6925 5250
	1    0    0    -1  
$EndComp
$Comp
L Connector:DIN-5_180degree J15
U 1 1 6115894E
P 8550 5950
F 0 "J15" H 8550 5675 50  0000 C CNN
F 1 "DIN-5_180degree" H 8550 5584 50  0000 C CNN
F 2 "footprints:DIN5" H 8550 5950 50  0001 C CNN
F 3 "http://www.mouser.com/ds/2/18/40_c091_abd_e-75918.pdf" H 8550 5950 50  0001 C CNN
	1    8550 5950
	1    0    0    -1  
$EndComp
$Comp
L Device:R R7
U 1 1 611613A9
P 8100 5850
F 0 "R7" V 7893 5850 50  0000 C CNN
F 1 "R" V 7984 5850 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 8030 5850 50  0001 C CNN
F 3 "~" H 8100 5850 50  0001 C CNN
	1    8100 5850
	0    -1   1    0   
$EndComp
Wire Wire Line
	7950 6050 8975 6050
Wire Wire Line
	8975 6050 8975 5850
Wire Wire Line
	8975 5850 8850 5850
NoConn ~ 8850 5950
NoConn ~ 8250 5950
NoConn ~ 8550 5650
$Comp
L Connector:DIN-5_180degree J14
U 1 1 61183178
P 8550 4775
F 0 "J14" H 8550 4500 50  0000 C CNN
F 1 "DIN-5_180degree" H 8550 4409 50  0000 C CNN
F 2 "footprints:DIN5" H 8550 4775 50  0001 C CNN
F 3 "http://www.mouser.com/ds/2/18/40_c091_abd_e-75918.pdf" H 8550 4775 50  0001 C CNN
	1    8550 4775
	1    0    0    -1  
$EndComp
$Comp
L Device:R R6
U 1 1 6119CB3D
P 8100 4675
F 0 "R6" V 7893 4675 50  0000 C CNN
F 1 "R" V 7984 4675 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 8030 4675 50  0001 C CNN
F 3 "~" H 8100 4675 50  0001 C CNN
	1    8100 4675
	0    -1   1    0   
$EndComp
$Comp
L Device:R R8
U 1 1 6119D224
P 9000 4675
F 0 "R8" V 8793 4675 50  0000 C CNN
F 1 "R" V 8884 4675 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 8930 4675 50  0001 C CNN
F 3 "~" H 9000 4675 50  0001 C CNN
	1    9000 4675
	0    -1   1    0   
$EndComp
Wire Wire Line
	9150 4900 9150 4675
$Comp
L power:VCC #PWR045
U 1 1 611A7E2D
P 7950 4675
F 0 "#PWR045" H 7950 4525 50  0001 C CNN
F 1 "VCC" H 7965 4848 50  0000 C CNN
F 2 "" H 7950 4675 50  0001 C CNN
F 3 "" H 7950 4675 50  0001 C CNN
	1    7950 4675
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR048
U 1 1 611A8B3D
P 8750 4375
F 0 "#PWR048" H 8750 4125 50  0001 C CNN
F 1 "GND" H 8755 4202 50  0000 C CNN
F 2 "" H 8750 4375 50  0001 C CNN
F 3 "" H 8750 4375 50  0001 C CNN
	1    8750 4375
	1    0    0    -1  
$EndComp
Wire Wire Line
	8750 4375 8550 4375
Wire Wire Line
	8550 4375 8550 4475
NoConn ~ 8250 4775
NoConn ~ 8850 4775
Wire Wire Line
	9625 2375 9625 2325
$Comp
L power:GND #PWR051
U 1 1 612AA6BC
P 9625 2675
F 0 "#PWR051" H 9625 2425 50  0001 C CNN
F 1 "GND" H 9630 2502 50  0000 C CNN
F 2 "" H 9625 2675 50  0001 C CNN
F 3 "" H 9625 2675 50  0001 C CNN
	1    9625 2675
	1    0    0    -1  
$EndComp
Wire Wire Line
	9625 2325 9025 2325
Wire Wire Line
	9025 2325 9025 1825
Wire Wire Line
	8700 2175 8850 2175
Wire Wire Line
	8850 2175 8850 1625
Wire Wire Line
	8850 1625 9025 1625
$Comp
L Device:R R10
U 1 1 612DD87F
P 9625 2175
F 0 "R10" H 9555 2129 50  0000 R CNN
F 1 "220" H 9555 2220 50  0000 R CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 9555 2175 50  0001 C CNN
F 3 "~" H 9625 2175 50  0001 C CNN
	1    9625 2175
	-1   0    0    1   
$EndComp
Connection ~ 9625 2325
$Comp
L Device:LED D2
U 1 1 61315AAD
P 7000 5400
F 0 "D2" V 7025 5225 50  0000 L CNN
F 1 "LED" V 6950 5175 50  0000 L CNN
F 2 "LED_THT:LED_D3.0mm_Horizontal_O1.27mm_Z2.0mm_IRBlack" H 7000 5400 50  0001 C CNN
F 3 "~" H 7000 5400 50  0001 C CNN
	1    7000 5400
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R5
U 1 1 61316C87
P 7000 5700
F 0 "R5" H 6931 5654 50  0000 R CNN
F 1 "R" H 6931 5745 50  0000 R CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 6930 5700 50  0001 C CNN
F 3 "~" H 7000 5700 50  0001 C CNN
	1    7000 5700
	-1   0    0    -1  
$EndComp
Connection ~ 7000 5850
Wire Wire Line
	7000 5850 6875 5850
Wire Wire Line
	7000 5250 6925 5250
Wire Wire Line
	6875 5250 6875 5550
Connection ~ 6925 5250
Wire Wire Line
	6925 5250 6875 5250
Wire Wire Line
	6425 5350 6600 5350
Wire Wire Line
	6600 5350 6600 4900
Wire Wire Line
	6600 4900 9150 4900
$Comp
L Device:LED D1
U 1 1 613D7673
P 6600 4450
F 0 "D1" V 6625 4275 50  0000 L CNN
F 1 "LED" V 6550 4225 50  0000 L CNN
F 2 "LED_THT:LED_D3.0mm_Horizontal_O1.27mm_Z2.0mm_IRBlack" H 6600 4450 50  0001 C CNN
F 3 "~" H 6600 4450 50  0001 C CNN
	1    6600 4450
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R3
U 1 1 613D767D
P 6600 4750
F 0 "R3" H 6531 4704 50  0000 R CNN
F 1 "R" H 6531 4795 50  0000 R CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 6530 4750 50  0001 C CNN
F 3 "~" H 6600 4750 50  0001 C CNN
	1    6600 4750
	-1   0    0    -1  
$EndComp
Connection ~ 6600 4900
$Comp
L power:VCC #PWR041
U 1 1 613F4E95
P 6600 4300
F 0 "#PWR041" H 6600 4150 50  0001 C CNN
F 1 "VCC" H 6600 4450 50  0000 C CNN
F 2 "" H 6600 4300 50  0001 C CNN
F 3 "" H 6600 4300 50  0001 C CNN
	1    6600 4300
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x12 J12
U 1 1 6141ADB1
P 1325 6100
F 0 "J12" V 1450 6046 50  0000 C CNN
F 1 "Conn_01x12" V 1541 6046 50  0000 C CNN
F 2 "Connector_JST:JST_PH_B12B-PH-K_1x12_P2.00mm_Vertical" H 1325 6100 50  0001 C CNN
F 3 "~" H 1325 6100 50  0001 C CNN
	1    1325 6100
	0    -1   1    0   
$EndComp
$Comp
L power:+5V #PWR026
U 1 1 61435252
P 875 5825
F 0 "#PWR026" H 875 5675 50  0001 C CNN
F 1 "+5V" H 890 5998 50  0000 C CNN
F 2 "" H 875 5825 50  0001 C CNN
F 3 "" H 875 5825 50  0001 C CNN
	1    875  5825
	1    0    0    -1  
$EndComp
Wire Wire Line
	825  5900 875  5900
Connection ~ 875  5900
Wire Wire Line
	875  5900 925  5900
Wire Wire Line
	875  5825 875  5900
$Comp
L power:GND #PWR029
U 1 1 61450161
P 1125 5750
F 0 "#PWR029" H 1125 5500 50  0001 C CNN
F 1 "GND" H 1125 5850 50  0000 C CNN
F 2 "" H 1125 5750 50  0001 C CNN
F 3 "" H 1125 5750 50  0001 C CNN
	1    1125 5750
	1    0    0    -1  
$EndComp
Wire Wire Line
	1125 5900 1025 5900
Wire Wire Line
	1025 5900 1025 5750
Wire Wire Line
	1025 5750 1125 5750
Connection ~ 1025 5900
Wire Wire Line
	1225 5900 1325 5900
Connection ~ 1325 5900
Wire Wire Line
	1325 5900 1375 5900
Connection ~ 1425 5900
Wire Wire Line
	1425 5900 1525 5900
Wire Wire Line
	1625 5900 1725 5900
Connection ~ 1725 5900
Wire Wire Line
	1725 5900 1825 5900
Connection ~ 1825 5900
Wire Wire Line
	1825 5900 1925 5900
$Comp
L power:GNDPWR #PWR025
U 1 1 61477B25
P 850 6700
F 0 "#PWR025" H 850 6500 50  0001 C CNN
F 1 "GNDPWR" H 854 6546 50  0000 C CNN
F 2 "" H 850 6650 50  0001 C CNN
F 3 "" H 850 6650 50  0001 C CNN
	1    850  6700
	1    0    0    -1  
$EndComp
$Comp
L Device:Net-Tie_3_Tee NT1
U 1 1 6147C231
P 850 6600
F 0 "NT1" H 850 6781 50  0000 C CNN
F 1 "Net-Tie_3_Tee" H 850 6690 50  0000 C CNN
F 2 "NetTie:NetTie-3_SMD_Pad0.5mm" H 850 6600 50  0001 C CNN
F 3 "~" H 850 6600 50  0001 C CNN
	1    850  6600
	1    0    0    -1  
$EndComp
$Comp
L power:GNDPWR #PWR034
U 1 1 614A4486
P 1875 5650
F 0 "#PWR034" H 1875 5450 50  0001 C CNN
F 1 "GNDPWR" H 1879 5496 50  0000 C CNN
F 2 "" H 1875 5600 50  0001 C CNN
F 3 "" H 1875 5600 50  0001 C CNN
	1    1875 5650
	1    0    0    -1  
$EndComp
Wire Wire Line
	1875 5650 1725 5650
Wire Wire Line
	1725 5650 1725 5900
$Comp
L power:+VDC #PWR033
U 1 1 614AEB84
P 1375 5900
F 0 "#PWR033" H 1375 5800 50  0001 C CNN
F 1 "+VDC" H 1375 6175 50  0000 C CNN
F 2 "" H 1375 5900 50  0001 C CNN
F 3 "" H 1375 5900 50  0001 C CNN
	1    1375 5900
	1    0    0    -1  
$EndComp
Connection ~ 1375 5900
Wire Wire Line
	1375 5900 1425 5900
Text GLabel 7375 975  0    50   Input ~ 0
sck
Text GLabel 7375 1075 0    50   Input ~ 0
sda
Text GLabel 7375 1175 0    50   Input ~ 0
res
Text GLabel 7375 1275 0    50   Input ~ 0
dc
Text GLabel 7375 1375 0    50   Input ~ 0
cs8
$Comp
L Connector_Generic:Conn_01x07 J9
U 1 1 60C71C6A
P 7575 1075
F 0 "J9" H 7575 1500 50  0000 C CNN
F 1 "Conn_01x07" H 7655 1026 50  0001 L CNN
F 2 "Connector_JST:JST_XH_B7B-XH-A_1x07_P2.50mm_Vertical" H 7575 1075 50  0001 C CNN
F 3 "~" H 7575 1075 50  0001 C CNN
	1    7575 1075
	1    0    0    -1  
$EndComp
$Comp
L power:VCC #PWR017
U 1 1 60C71C60
P 7025 875
F 0 "#PWR017" H 7025 725 50  0001 C CNN
F 1 "VCC" H 7040 1048 50  0000 C CNN
F 2 "" H 7025 875 50  0001 C CNN
F 3 "" H 7025 875 50  0001 C CNN
	1    7025 875 
	1    0    0    -1  
$EndComp
Wire Wire Line
	7025 875  7375 875 
$Comp
L power:GND #PWR018
U 1 1 60C71C55
P 7200 700
F 0 "#PWR018" H 7200 450 50  0001 C CNN
F 1 "GND" H 7205 527 50  0001 C CNN
F 2 "" H 7200 700 50  0001 C CNN
F 3 "" H 7200 700 50  0001 C CNN
	1    7200 700 
	1    0    0    -1  
$EndComp
Wire Wire Line
	7375 775  7350 775 
Wire Wire Line
	7350 775  7350 675 
Wire Wire Line
	7350 675  7200 675 
Wire Wire Line
	7200 675  7200 700 
Text GLabel 6575 975  0    50   Input ~ 0
sck
Text GLabel 6575 1075 0    50   Input ~ 0
sda
Text GLabel 6575 1175 0    50   Input ~ 0
res
Text GLabel 6575 1275 0    50   Input ~ 0
dc
Text GLabel 6575 1375 0    50   Input ~ 0
cs7
$Comp
L Connector_Generic:Conn_01x07 J8
U 1 1 60C6923E
P 6775 1075
F 0 "J8" H 6775 1500 50  0000 C CNN
F 1 "Conn_01x07" H 6855 1026 50  0001 L CNN
F 2 "Connector_JST:JST_XH_B7B-XH-A_1x07_P2.50mm_Vertical" H 6775 1075 50  0001 C CNN
F 3 "~" H 6775 1075 50  0001 C CNN
	1    6775 1075
	1    0    0    -1  
$EndComp
$Comp
L power:VCC #PWR015
U 1 1 60C69234
P 6225 875
F 0 "#PWR015" H 6225 725 50  0001 C CNN
F 1 "VCC" H 6240 1048 50  0000 C CNN
F 2 "" H 6225 875 50  0001 C CNN
F 3 "" H 6225 875 50  0001 C CNN
	1    6225 875 
	1    0    0    -1  
$EndComp
Wire Wire Line
	6225 875  6575 875 
$Comp
L power:GND #PWR016
U 1 1 60C69229
P 6400 700
F 0 "#PWR016" H 6400 450 50  0001 C CNN
F 1 "GND" H 6405 527 50  0001 C CNN
F 2 "" H 6400 700 50  0001 C CNN
F 3 "" H 6400 700 50  0001 C CNN
	1    6400 700 
	1    0    0    -1  
$EndComp
Wire Wire Line
	6575 775  6550 775 
Wire Wire Line
	6550 775  6550 675 
Wire Wire Line
	6550 675  6400 675 
Wire Wire Line
	6400 675  6400 700 
Text GLabel 5775 975  0    50   Input ~ 0
sck
Text GLabel 5775 1075 0    50   Input ~ 0
sda
Text GLabel 5775 1175 0    50   Input ~ 0
res
Text GLabel 5775 1275 0    50   Input ~ 0
dc
Text GLabel 5775 1375 0    50   Input ~ 0
cs6
$Comp
L Connector_Generic:Conn_01x07 J7
U 1 1 60C69216
P 5975 1075
F 0 "J7" H 5975 1500 50  0000 C CNN
F 1 "Conn_01x07" H 6055 1026 50  0001 L CNN
F 2 "Connector_JST:JST_XH_B7B-XH-A_1x07_P2.50mm_Vertical" H 5975 1075 50  0001 C CNN
F 3 "~" H 5975 1075 50  0001 C CNN
	1    5975 1075
	1    0    0    -1  
$EndComp
$Comp
L power:VCC #PWR013
U 1 1 60C6920C
P 5425 875
F 0 "#PWR013" H 5425 725 50  0001 C CNN
F 1 "VCC" H 5440 1048 50  0000 C CNN
F 2 "" H 5425 875 50  0001 C CNN
F 3 "" H 5425 875 50  0001 C CNN
	1    5425 875 
	1    0    0    -1  
$EndComp
Wire Wire Line
	5425 875  5775 875 
$Comp
L power:GND #PWR014
U 1 1 60C69201
P 5600 700
F 0 "#PWR014" H 5600 450 50  0001 C CNN
F 1 "GND" H 5605 527 50  0001 C CNN
F 2 "" H 5600 700 50  0001 C CNN
F 3 "" H 5600 700 50  0001 C CNN
	1    5600 700 
	1    0    0    -1  
$EndComp
Wire Wire Line
	5775 775  5750 775 
Wire Wire Line
	5750 775  5750 675 
Wire Wire Line
	5750 675  5600 675 
Wire Wire Line
	5600 675  5600 700 
Text GLabel 4975 975  0    50   Input ~ 0
sck
Text GLabel 4975 1075 0    50   Input ~ 0
sda
Text GLabel 4975 1175 0    50   Input ~ 0
res
Text GLabel 4975 1275 0    50   Input ~ 0
dc
Text GLabel 4975 1375 0    50   Input ~ 0
cs5
$Comp
L Connector_Generic:Conn_01x07 J6
U 1 1 60C691EE
P 5175 1075
F 0 "J6" H 5175 1500 50  0000 C CNN
F 1 "Conn_01x07" H 5255 1026 50  0001 L CNN
F 2 "Connector_JST:JST_XH_B7B-XH-A_1x07_P2.50mm_Vertical" H 5175 1075 50  0001 C CNN
F 3 "~" H 5175 1075 50  0001 C CNN
	1    5175 1075
	1    0    0    -1  
$EndComp
$Comp
L power:VCC #PWR011
U 1 1 60C691E4
P 4625 875
F 0 "#PWR011" H 4625 725 50  0001 C CNN
F 1 "VCC" H 4640 1048 50  0000 C CNN
F 2 "" H 4625 875 50  0001 C CNN
F 3 "" H 4625 875 50  0001 C CNN
	1    4625 875 
	1    0    0    -1  
$EndComp
Wire Wire Line
	4625 875  4975 875 
$Comp
L power:GND #PWR012
U 1 1 60C691D9
P 4800 700
F 0 "#PWR012" H 4800 450 50  0001 C CNN
F 1 "GND" H 4805 527 50  0001 C CNN
F 2 "" H 4800 700 50  0001 C CNN
F 3 "" H 4800 700 50  0001 C CNN
	1    4800 700 
	1    0    0    -1  
$EndComp
Wire Wire Line
	4975 775  4950 775 
Wire Wire Line
	4950 775  4950 675 
Wire Wire Line
	4950 675  4800 675 
Wire Wire Line
	4800 675  4800 700 
Text GLabel 4175 975  0    50   Input ~ 0
sck
Text GLabel 4175 1075 0    50   Input ~ 0
sda
Text GLabel 4175 1175 0    50   Input ~ 0
res
Text GLabel 4175 1275 0    50   Input ~ 0
dc
Text GLabel 4175 1375 0    50   Input ~ 0
cs4
$Comp
L Connector_Generic:Conn_01x07 J5
U 1 1 60C691C6
P 4375 1075
F 0 "J5" H 4375 1500 50  0000 C CNN
F 1 "Conn_01x07" H 4455 1026 50  0001 L CNN
F 2 "Connector_JST:JST_XH_B7B-XH-A_1x07_P2.50mm_Vertical" H 4375 1075 50  0001 C CNN
F 3 "~" H 4375 1075 50  0001 C CNN
	1    4375 1075
	1    0    0    -1  
$EndComp
$Comp
L power:VCC #PWR09
U 1 1 60C691BC
P 3825 875
F 0 "#PWR09" H 3825 725 50  0001 C CNN
F 1 "VCC" H 3840 1048 50  0000 C CNN
F 2 "" H 3825 875 50  0001 C CNN
F 3 "" H 3825 875 50  0001 C CNN
	1    3825 875 
	1    0    0    -1  
$EndComp
Wire Wire Line
	3825 875  4175 875 
$Comp
L power:GND #PWR010
U 1 1 60C691B1
P 4000 700
F 0 "#PWR010" H 4000 450 50  0001 C CNN
F 1 "GND" H 4005 527 50  0001 C CNN
F 2 "" H 4000 700 50  0001 C CNN
F 3 "" H 4000 700 50  0001 C CNN
	1    4000 700 
	1    0    0    -1  
$EndComp
Wire Wire Line
	4175 775  4150 775 
Wire Wire Line
	4150 775  4150 675 
Wire Wire Line
	4150 675  4000 675 
Wire Wire Line
	4000 675  4000 700 
Text GLabel 3375 975  0    50   Input ~ 0
sck
Text GLabel 3375 1075 0    50   Input ~ 0
sda
Text GLabel 3375 1175 0    50   Input ~ 0
res
Text GLabel 3375 1275 0    50   Input ~ 0
dc
Text GLabel 3375 1375 0    50   Input ~ 0
cs3
$Comp
L Connector_Generic:Conn_01x07 J4
U 1 1 60C5FC40
P 3575 1075
F 0 "J4" H 3575 1500 50  0000 C CNN
F 1 "Conn_01x07" H 3655 1026 50  0001 L CNN
F 2 "Connector_JST:JST_XH_B7B-XH-A_1x07_P2.50mm_Vertical" H 3575 1075 50  0001 C CNN
F 3 "~" H 3575 1075 50  0001 C CNN
	1    3575 1075
	1    0    0    -1  
$EndComp
$Comp
L power:VCC #PWR07
U 1 1 60C5FC36
P 3025 875
F 0 "#PWR07" H 3025 725 50  0001 C CNN
F 1 "VCC" H 3040 1048 50  0000 C CNN
F 2 "" H 3025 875 50  0001 C CNN
F 3 "" H 3025 875 50  0001 C CNN
	1    3025 875 
	1    0    0    -1  
$EndComp
Wire Wire Line
	3025 875  3375 875 
$Comp
L power:GND #PWR08
U 1 1 60C5FC2B
P 3200 700
F 0 "#PWR08" H 3200 450 50  0001 C CNN
F 1 "GND" H 3205 527 50  0001 C CNN
F 2 "" H 3200 700 50  0001 C CNN
F 3 "" H 3200 700 50  0001 C CNN
	1    3200 700 
	1    0    0    -1  
$EndComp
Wire Wire Line
	3375 775  3350 775 
Wire Wire Line
	3350 775  3350 675 
Wire Wire Line
	3350 675  3200 675 
Wire Wire Line
	3200 675  3200 700 
Text GLabel 2575 975  0    50   Input ~ 0
sck
Text GLabel 2575 1075 0    50   Input ~ 0
sda
Text GLabel 2575 1175 0    50   Input ~ 0
res
Text GLabel 2575 1275 0    50   Input ~ 0
dc
Text GLabel 2575 1375 0    50   Input ~ 0
cs2
$Comp
L Connector_Generic:Conn_01x07 J3
U 1 1 60C5FC18
P 2775 1075
F 0 "J3" H 2775 1500 50  0000 C CNN
F 1 "Conn_01x07" H 2855 1026 50  0001 L CNN
F 2 "Connector_JST:JST_XH_B7B-XH-A_1x07_P2.50mm_Vertical" H 2775 1075 50  0001 C CNN
F 3 "~" H 2775 1075 50  0001 C CNN
	1    2775 1075
	1    0    0    -1  
$EndComp
$Comp
L power:VCC #PWR05
U 1 1 60C5FC0E
P 2225 875
F 0 "#PWR05" H 2225 725 50  0001 C CNN
F 1 "VCC" H 2240 1048 50  0000 C CNN
F 2 "" H 2225 875 50  0001 C CNN
F 3 "" H 2225 875 50  0001 C CNN
	1    2225 875 
	1    0    0    -1  
$EndComp
Wire Wire Line
	2225 875  2575 875 
$Comp
L power:GND #PWR06
U 1 1 60C5FC03
P 2400 700
F 0 "#PWR06" H 2400 450 50  0001 C CNN
F 1 "GND" H 2405 527 50  0001 C CNN
F 2 "" H 2400 700 50  0001 C CNN
F 3 "" H 2400 700 50  0001 C CNN
	1    2400 700 
	1    0    0    -1  
$EndComp
Wire Wire Line
	2575 775  2550 775 
Wire Wire Line
	2550 775  2550 675 
Wire Wire Line
	2550 675  2400 675 
Wire Wire Line
	2400 675  2400 700 
Text GLabel 1775 975  0    50   Input ~ 0
sck
Text GLabel 1775 1075 0    50   Input ~ 0
sda
Text GLabel 1775 1175 0    50   Input ~ 0
res
Text GLabel 1775 1275 0    50   Input ~ 0
dc
Text GLabel 1775 1375 0    50   Input ~ 0
cs1
$Comp
L Connector_Generic:Conn_01x07 J2
U 1 1 60C5C749
P 1975 1075
F 0 "J2" H 1975 1500 50  0000 C CNN
F 1 "Conn_01x07" H 2055 1026 50  0001 L CNN
F 2 "Connector_JST:JST_XH_B7B-XH-A_1x07_P2.50mm_Vertical" H 1975 1075 50  0001 C CNN
F 3 "~" H 1975 1075 50  0001 C CNN
	1    1975 1075
	1    0    0    -1  
$EndComp
$Comp
L power:VCC #PWR03
U 1 1 60C5C73F
P 1425 875
F 0 "#PWR03" H 1425 725 50  0001 C CNN
F 1 "VCC" H 1440 1048 50  0000 C CNN
F 2 "" H 1425 875 50  0001 C CNN
F 3 "" H 1425 875 50  0001 C CNN
	1    1425 875 
	1    0    0    -1  
$EndComp
Wire Wire Line
	1425 875  1775 875 
$Comp
L power:GND #PWR04
U 1 1 60C5C734
P 1600 700
F 0 "#PWR04" H 1600 450 50  0001 C CNN
F 1 "GND" H 1605 527 50  0001 C CNN
F 2 "" H 1600 700 50  0001 C CNN
F 3 "" H 1600 700 50  0001 C CNN
	1    1600 700 
	1    0    0    -1  
$EndComp
Wire Wire Line
	1775 775  1750 775 
Wire Wire Line
	1750 775  1750 675 
Wire Wire Line
	1750 675  1600 675 
Wire Wire Line
	1600 675  1600 700 
Text GLabel 975  975  0    50   Input ~ 0
sck
Text GLabel 975  1075 0    50   Input ~ 0
sda
Text GLabel 975  1175 0    50   Input ~ 0
res
Text GLabel 975  1275 0    50   Input ~ 0
dc
Text GLabel 975  1375 0    50   Input ~ 0
cs0
$Comp
L Connector_Generic:Conn_01x07 J1
U 1 1 60C5691A
P 1175 1075
F 0 "J1" H 1175 1500 50  0000 C CNN
F 1 "Conn_01x07" H 1255 1026 50  0001 L CNN
F 2 "Connector_JST:JST_XH_B7B-XH-A_1x07_P2.50mm_Vertical" H 1175 1075 50  0001 C CNN
F 3 "~" H 1175 1075 50  0001 C CNN
	1    1175 1075
	1    0    0    -1  
$EndComp
$Comp
L power:VCC #PWR01
U 1 1 60C579A6
P 625 875
F 0 "#PWR01" H 625 725 50  0001 C CNN
F 1 "VCC" H 640 1048 50  0000 C CNN
F 2 "" H 625 875 50  0001 C CNN
F 3 "" H 625 875 50  0001 C CNN
	1    625  875 
	1    0    0    -1  
$EndComp
Wire Wire Line
	625  875  975  875 
$Comp
L power:GND #PWR02
U 1 1 60C57198
P 800 700
F 0 "#PWR02" H 800 450 50  0001 C CNN
F 1 "GND" H 900 575 50  0000 C CNN
F 2 "" H 800 700 50  0001 C CNN
F 3 "" H 800 700 50  0001 C CNN
	1    800  700 
	1    0    0    -1  
$EndComp
Wire Wire Line
	975  775  950  775 
Wire Wire Line
	950  775  950  675 
Wire Wire Line
	950  675  800  675 
Wire Wire Line
	800  675  800  700 
$Comp
L power:GNDA #PWR023
U 1 1 6151E876
P 2975 7325
F 0 "#PWR023" H 2975 7075 50  0001 C CNN
F 1 "GNDA" H 3100 7175 50  0000 C CNN
F 2 "" H 2975 7325 50  0001 C CNN
F 3 "" H 2975 7325 50  0001 C CNN
	1    2975 7325
	1    0    0    -1  
$EndComp
Connection ~ 1600 7425
Connection ~ 2800 7625
Connection ~ 2800 7025
Wire Wire Line
	2650 7025 2800 7025
Wire Wire Line
	2650 7225 2650 7025
Wire Wire Line
	2650 7625 2650 7425
Wire Wire Line
	2800 7625 2650 7625
Connection ~ 2800 7325
$Comp
L Device:R R2
U 1 1 614F29EF
P 2800 7475
F 0 "R2" H 2850 7425 50  0000 L CNN
F 1 "1k" V 2800 7425 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 2730 7475 50  0001 C CNN
F 3 "~" H 2800 7475 50  0001 C CNN
	1    2800 7475
	1    0    0    -1  
$EndComp
Wire Wire Line
	2650 7325 2800 7325
$Comp
L Device:R R1
U 1 1 614E7BC0
P 2800 7175
F 0 "R1" H 2850 7125 50  0000 L CNN
F 1 "1k" V 2800 7125 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 2730 7175 50  0001 C CNN
F 3 "~" H 2800 7175 50  0001 C CNN
	1    2800 7175
	1    0    0    -1  
$EndComp
Wire Wire Line
	2975 7325 2800 7325
$Comp
L power:+15V #PWR021
U 1 1 614CE498
P 2800 7025
F 0 "#PWR021" H 2800 6875 50  0001 C CNN
F 1 "+15V" H 2950 7075 50  0000 C CNN
F 2 "" H 2800 7025 50  0001 C CNN
F 3 "" H 2800 7025 50  0001 C CNN
	1    2800 7025
	1    0    0    -1  
$EndComp
$Comp
L power:-15V #PWR022
U 1 1 614CB6DD
P 2800 7625
F 0 "#PWR022" H 2800 7725 50  0001 C CNN
F 1 "-15V" H 2650 7625 50  0000 C CNN
F 2 "" H 2800 7625 50  0001 C CNN
F 3 "" H 2800 7625 50  0001 C CNN
	1    2800 7625
	-1   0    0    1   
$EndComp
Connection ~ 1600 7225
Wire Wire Line
	1600 7275 1600 7225
Wire Wire Line
	1600 7425 1600 7375
Wire Wire Line
	1375 7425 1600 7425
$Comp
L bluepill:DD39AJPA A1
U 1 1 614B1B14
P 2150 7325
F 0 "A1" H 2125 7650 50  0000 C CNN
F 1 "DD39AJPA" H 2125 7559 50  0000 C CNN
F 2 "footprints:DD39AJPA" H 2150 7325 50  0001 C CNN
F 3 "" H 2150 7325 50  0001 C CNN
	1    2150 7325
	1    0    0    -1  
$EndComp
$Comp
L power:GNDPWR #PWR020
U 1 1 614769FE
P 1600 7425
F 0 "#PWR020" H 1600 7225 50  0001 C CNN
F 1 "GNDPWR" H 1604 7271 50  0000 C CNN
F 2 "" H 1600 7375 50  0001 C CNN
F 3 "" H 1600 7375 50  0001 C CNN
	1    1600 7425
	1    0    0    -1  
$EndComp
Wire Wire Line
	1600 7225 1600 7125
Wire Wire Line
	1375 7225 1600 7225
$Comp
L power:+VDC #PWR019
U 1 1 61208A70
P 1600 7125
F 0 "#PWR019" H 1600 7025 50  0001 C CNN
F 1 "+VDC" H 1600 7400 50  0000 C CNN
F 2 "" H 1600 7125 50  0001 C CNN
F 3 "" H 1600 7125 50  0001 C CNN
	1    1600 7125
	1    0    0    -1  
$EndComp
$Comp
L Connector:Barrel_Jack J10
U 1 1 611F3957
P 1075 7325
F 0 "J10" H 1132 7650 50  0000 C CNN
F 1 "Barrel_Jack" H 1132 7559 50  0000 C CNN
F 2 "Connector_BarrelJack:BarrelJack_CUI_PJ-063AH_Horizontal" H 1125 7285 50  0001 C CNN
F 3 "~" H 1125 7285 50  0001 C CNN
	1    1075 7325
	1    0    0    -1  
$EndComp
Wire Wire Line
	4700 2875 5825 2875
Wire Bus Line
	2850 3875 2850 5225
Wire Bus Line
	1275 3625 1275 5225
Wire Bus Line
	5050 2575 5050 5225
$EndSCHEMATC
