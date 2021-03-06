EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A3 16535 11693
encoding utf-8
Sheet 1 1
Title "PMSM dirver "
Date "2021-03-30"
Rev "1"
Comp "M.Ashrf - SEM "
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L drv8323s:DRV8323H U2
U 1 1 60590921
P 13300 4975
F 0 "U2" H 13300 6462 60  0000 C CNN
F 1 "DRV8323H" H 13300 6356 60  0000 C CNN
F 2 "Package_DFN_QFN:QFN-40-1EP_6x6mm_P0.5mm_EP4.6x4.6mm_ThermalVias" H 13300 4975 60  0001 C CNN
F 3 "" H 13300 4975 60  0001 C CNN
	1    13300 4975
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small R4
U 1 1 60576DD4
P 14650 5425
F 0 "R4" H 14709 5471 50  0000 L CNN
F 1 "10K" H 14709 5380 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" H 14650 5425 50  0001 C CNN
F 3 "~" H 14650 5425 50  0001 C CNN
	1    14650 5425
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR027
U 1 1 60576DD5
P 14650 5325
F 0 "#PWR027" H 14650 5175 50  0001 C CNN
F 1 "+3.3V" H 14665 5498 50  0000 C CNN
F 2 "" H 14650 5325 50  0001 C CNN
F 3 "" H 14650 5325 50  0001 C CNN
	1    14650 5325
	1    0    0    -1  
$EndComp
Wire Wire Line
	14100 5575 14650 5575
Wire Wire Line
	14650 5575 14650 5525
$Comp
L Device:C_Small C18
U 1 1 60576DD6
P 10800 4025
F 0 "C18" H 10892 4071 50  0000 L CNN
F 1 "47u" H 10892 3980 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 10800 4025 50  0001 C CNN
F 3 "~" H 10800 4025 50  0001 C CNN
	1    10800 4025
	1    0    0    -1  
$EndComp
Text GLabel 12500 3975 0    50   Input ~ 0
CPH
Text GLabel 10800 3875 1    50   Input ~ 0
CPH
Text GLabel 12500 4075 0    50   Input ~ 0
CPL
Text GLabel 10800 4175 3    50   Input ~ 0
CPL
Wire Wire Line
	10800 4175 10800 4125
Wire Wire Line
	10800 3925 10800 3875
$Comp
L power:+48V #PWR016
U 1 1 60576DD7
P 12450 3250
F 0 "#PWR016" H 12450 3100 50  0001 C CNN
F 1 "+48V" H 12465 3423 50  0000 C CNN
F 2 "" H 12450 3250 50  0001 C CNN
F 3 "" H 12450 3250 50  0001 C CNN
	1    12450 3250
	1    0    0    -1  
$EndComp
Wire Wire Line
	12500 3775 12450 3775
$Comp
L Device:C_Small C17
U 1 1 605BF1B3
P 12075 3675
F 0 "C17" H 12167 3721 50  0000 L CNN
F 1 "47u" H 12167 3630 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 12075 3675 50  0001 C CNN
F 3 "~" H 12075 3675 50  0001 C CNN
	1    12075 3675
	1    0    0    -1  
$EndComp
Wire Wire Line
	12450 3250 12450 3400
Wire Wire Line
	12500 3875 12075 3875
Wire Wire Line
	12075 3875 12075 3775
Wire Wire Line
	12075 3575 12075 3400
Wire Wire Line
	12075 3400 12450 3400
Connection ~ 12450 3400
Wire Wire Line
	12450 3400 12450 3775
$Comp
L Device:C_Small C16
U 1 1 605CC7FF
P 11750 3675
F 0 "C16" H 11750 3750 50  0000 L CNN
F 1 "100n" H 11750 3600 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 11750 3675 50  0001 C CNN
F 3 "~" H 11750 3675 50  0001 C CNN
	1    11750 3675
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C15
U 1 1 605CCC32
P 11450 3675
F 0 "C15" H 11450 3750 50  0000 L CNN
F 1 "47u" H 11450 3600 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 11450 3675 50  0001 C CNN
F 3 "~" H 11450 3675 50  0001 C CNN
	1    11450 3675
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR031
U 1 1 605CCF16
P 13300 6725
F 0 "#PWR031" H 13300 6475 50  0001 C CNN
F 1 "GND" H 13305 6552 50  0000 C CNN
F 2 "" H 13300 6725 50  0001 C CNN
F 3 "" H 13300 6725 50  0001 C CNN
	1    13300 6725
	1    0    0    -1  
$EndComp
Wire Wire Line
	13300 6725 13300 6675
$Comp
L power:GND #PWR021
U 1 1 605D1DD6
P 11450 3875
F 0 "#PWR021" H 11450 3625 50  0001 C CNN
F 1 "GND" H 11455 3702 50  0000 C CNN
F 2 "" H 11450 3875 50  0001 C CNN
F 3 "" H 11450 3875 50  0001 C CNN
	1    11450 3875
	1    0    0    -1  
$EndComp
Wire Wire Line
	11450 3875 11450 3825
Wire Wire Line
	11750 3775 11750 3825
Wire Wire Line
	11750 3825 11450 3825
Connection ~ 11450 3825
Wire Wire Line
	11450 3825 11450 3775
Wire Wire Line
	11750 3575 11750 3400
Wire Wire Line
	11750 3400 12075 3400
Connection ~ 12075 3400
Wire Wire Line
	11450 3575 11450 3400
Wire Wire Line
	11450 3400 11750 3400
Connection ~ 11750 3400
NoConn ~ 12500 4375
NoConn ~ 12500 4475
Text GLabel 14100 4075 2    50   Input ~ 0
GHA
Text GLabel 14100 4175 2    50   Input ~ 0
SHA
Text GLabel 14100 4275 2    50   Input ~ 0
GLA
Text GLabel 14100 5975 2    50   Input ~ 0
GND
Text GLabel 14100 6175 2    50   Input ~ 0
GND
Text GLabel 14100 6375 2    50   Input ~ 0
GND
Text GLabel 14100 6275 2    50   Input ~ 0
SPA
$Comp
L power:+3.3V #PWR028
U 1 1 6064A4B6
P 11975 5925
F 0 "#PWR028" H 11975 5775 50  0001 C CNN
F 1 "+3.3V" H 11800 5925 50  0000 C CNN
F 2 "" H 11975 5925 50  0001 C CNN
F 3 "" H 11975 5925 50  0001 C CNN
	1    11975 5925
	1    0    0    -1  
$EndComp
Wire Wire Line
	11975 5925 11975 5975
Wire Wire Line
	11975 5975 12500 5975
Text GLabel 14100 4575 2    50   Input ~ 0
GHB
Text GLabel 14100 4675 2    50   Input ~ 0
SHB
Text GLabel 14100 4775 2    50   Input ~ 0
GLB
Text GLabel 14100 6075 2    50   Input ~ 0
SPB
Text GLabel 14100 5875 2    50   Input ~ 0
SPC
Text GLabel 14100 5075 2    50   Input ~ 0
GHC
Text GLabel 14100 5175 2    50   Input ~ 0
SHC
Text GLabel 14100 5275 2    50   Input ~ 0
GLC
Wire Wire Line
	14100 3775 14275 3775
Wire Wire Line
	14275 3775 14275 3400
Wire Wire Line
	14275 3400 12450 3400
$Comp
L Device:R_Small R6
U 1 1 6055C0F1
P 2325 8875
F 0 "R6" H 2384 8921 50  0000 L CNN
F 1 "5" H 2384 8830 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric" H 2325 8875 50  0001 C CNN
F 3 "~" H 2325 8875 50  0001 C CNN
	1    2325 8875
	0    -1   -1   0   
$EndComp
$Comp
L Device:R_Small R9
U 1 1 60576DCF
P 2325 9500
F 0 "R9" H 2384 9546 50  0000 L CNN
F 1 "5" H 2384 9455 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric" H 2325 9500 50  0001 C CNN
F 3 "~" H 2325 9500 50  0001 C CNN
	1    2325 9500
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR036
U 1 1 60576DD0
P 2725 10175
F 0 "#PWR036" H 2725 9925 50  0001 C CNN
F 1 "GND" H 2730 10002 50  0000 C CNN
F 2 "" H 2725 10175 50  0001 C CNN
F 3 "" H 2725 10175 50  0001 C CNN
	1    2725 10175
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small R12
U 1 1 60576DD1
P 2725 9950
F 0 "R12" H 2784 9996 50  0000 L CNN
F 1 "5m" H 2784 9905 50  0000 L CNN
F 2 "Resistor_SMD:R_2512_6332Metric" H 2725 9950 50  0001 C CNN
F 3 "~" H 2725 9950 50  0001 C CNN
	1    2725 9950
	1    0    0    -1  
$EndComp
$Comp
L power:+48V #PWR033
U 1 1 60576DD2
P 2725 8525
F 0 "#PWR033" H 2725 8375 50  0001 C CNN
F 1 "+48V" H 2740 8698 50  0000 C CNN
F 2 "" H 2725 8525 50  0001 C CNN
F 3 "" H 2725 8525 50  0001 C CNN
	1    2725 8525
	1    0    0    -1  
$EndComp
Wire Wire Line
	2725 10050 2725 10125
Text GLabel 2500 9800 0    50   Input ~ 0
SPA
Wire Wire Line
	2500 9800 2725 9800
Wire Wire Line
	2725 9800 2725 9850
Text GLabel 2500 10125 0    50   Input ~ 0
GND
Wire Wire Line
	2500 10125 2725 10125
Connection ~ 2725 10125
Wire Wire Line
	2725 10125 2725 10175
Text GLabel 2175 8875 0    50   Input ~ 0
GHA
Wire Wire Line
	2175 8875 2225 8875
Text GLabel 2875 9200 2    50   Input ~ 0
SHA
Text GLabel 2175 9500 0    50   Input ~ 0
GLA
Wire Wire Line
	2175 9500 2225 9500
Wire Wire Line
	2725 8675 2725 8575
Wire Wire Line
	2725 9800 2725 9700
Connection ~ 2725 9800
Wire Wire Line
	2875 9200 2725 9200
Wire Wire Line
	2725 9075 2725 9100
Connection ~ 2725 9200
Wire Wire Line
	2725 9200 2725 9300
$Comp
L Device:R_Small R7
U 1 1 6060E9F5
P 4350 8875
F 0 "R7" H 4409 8921 50  0000 L CNN
F 1 "5" H 4409 8830 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric" H 4350 8875 50  0001 C CNN
F 3 "~" H 4350 8875 50  0001 C CNN
	1    4350 8875
	0    -1   -1   0   
$EndComp
$Comp
L Device:R_Small R10
U 1 1 6060E9FC
P 4350 9500
F 0 "R10" H 4409 9546 50  0000 L CNN
F 1 "5" H 4409 9455 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric" H 4350 9500 50  0001 C CNN
F 3 "~" H 4350 9500 50  0001 C CNN
	1    4350 9500
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR037
U 1 1 6060EA03
P 4750 10175
F 0 "#PWR037" H 4750 9925 50  0001 C CNN
F 1 "GND" H 4755 10002 50  0000 C CNN
F 2 "" H 4750 10175 50  0001 C CNN
F 3 "" H 4750 10175 50  0001 C CNN
	1    4750 10175
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small R13
U 1 1 6060EA09
P 4750 9950
F 0 "R13" H 4809 9996 50  0000 L CNN
F 1 "5m" H 4809 9905 50  0000 L CNN
F 2 "Resistor_SMD:R_2512_6332Metric" H 4750 9950 50  0001 C CNN
F 3 "~" H 4750 9950 50  0001 C CNN
	1    4750 9950
	1    0    0    -1  
$EndComp
$Comp
L power:+48V #PWR034
U 1 1 6060EA0F
P 4750 8525
F 0 "#PWR034" H 4750 8375 50  0001 C CNN
F 1 "+48V" H 4765 8698 50  0000 C CNN
F 2 "" H 4750 8525 50  0001 C CNN
F 3 "" H 4750 8525 50  0001 C CNN
	1    4750 8525
	1    0    0    -1  
$EndComp
Wire Wire Line
	4750 10050 4750 10125
Text GLabel 4525 9800 0    50   Input ~ 0
SPB
Wire Wire Line
	4525 9800 4750 9800
Wire Wire Line
	4750 9800 4750 9850
Text GLabel 4525 10125 0    50   Input ~ 0
GND
Wire Wire Line
	4525 10125 4750 10125
Connection ~ 4750 10125
Wire Wire Line
	4750 10125 4750 10175
Text GLabel 4200 8875 0    50   Input ~ 0
GHB
Wire Wire Line
	4200 8875 4250 8875
Text GLabel 4900 9200 2    50   Input ~ 0
SHB
Text GLabel 4200 9500 0    50   Input ~ 0
GLB
Wire Wire Line
	4200 9500 4250 9500
Wire Wire Line
	4750 8675 4750 8575
Wire Wire Line
	4750 9800 4750 9700
Connection ~ 4750 9800
Wire Wire Line
	4900 9200 4750 9200
Wire Wire Line
	4750 9075 4750 9125
Connection ~ 4750 9200
Wire Wire Line
	4750 9200 4750 9300
$Comp
L Device:R_Small R8
U 1 1 60613934
P 6075 8875
F 0 "R8" H 6134 8921 50  0000 L CNN
F 1 "5" H 6134 8830 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric" H 6075 8875 50  0001 C CNN
F 3 "~" H 6075 8875 50  0001 C CNN
	1    6075 8875
	0    -1   -1   0   
$EndComp
$Comp
L Device:R_Small R11
U 1 1 6061393B
P 6075 9500
F 0 "R11" H 6134 9546 50  0000 L CNN
F 1 "5" H 6134 9455 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric" H 6075 9500 50  0001 C CNN
F 3 "~" H 6075 9500 50  0001 C CNN
	1    6075 9500
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR038
U 1 1 60613942
P 6475 10175
F 0 "#PWR038" H 6475 9925 50  0001 C CNN
F 1 "GND" H 6480 10002 50  0000 C CNN
F 2 "" H 6475 10175 50  0001 C CNN
F 3 "" H 6475 10175 50  0001 C CNN
	1    6475 10175
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small R14
U 1 1 60613948
P 6475 9950
F 0 "R14" H 6534 9996 50  0000 L CNN
F 1 "5m" H 6534 9905 50  0000 L CNN
F 2 "Resistor_SMD:R_2512_6332Metric" H 6475 9950 50  0001 C CNN
F 3 "~" H 6475 9950 50  0001 C CNN
	1    6475 9950
	1    0    0    -1  
$EndComp
$Comp
L power:+48V #PWR035
U 1 1 6061394E
P 6475 8525
F 0 "#PWR035" H 6475 8375 50  0001 C CNN
F 1 "+48V" H 6490 8698 50  0000 C CNN
F 2 "" H 6475 8525 50  0001 C CNN
F 3 "" H 6475 8525 50  0001 C CNN
	1    6475 8525
	1    0    0    -1  
$EndComp
Wire Wire Line
	6475 10050 6475 10125
Text GLabel 6250 9800 0    50   Input ~ 0
SPC
Wire Wire Line
	6250 9800 6475 9800
Wire Wire Line
	6475 9800 6475 9850
Text GLabel 6250 10125 0    50   Input ~ 0
GND
Wire Wire Line
	6250 10125 6475 10125
Connection ~ 6475 10125
Wire Wire Line
	6475 10125 6475 10175
Text GLabel 5925 8875 0    50   Input ~ 0
GHC
Wire Wire Line
	5925 8875 5975 8875
Text GLabel 6625 9200 2    50   Input ~ 0
SHC
Text GLabel 5925 9500 0    50   Input ~ 0
GLC
Wire Wire Line
	5925 9500 5975 9500
Wire Wire Line
	6475 8675 6475 8550
Wire Wire Line
	6475 9800 6475 9700
Connection ~ 6475 9800
Wire Wire Line
	6625 9200 6475 9200
Connection ~ 6475 9200
Wire Wire Line
	6475 9200 6475 9300
$Comp
L Connector:Conn_01x01_Male J9
U 1 1 60701527
P 6850 8550
F 0 "J9" H 6822 8482 50  0000 R CNN
F 1 "pd" H 6822 8573 50  0000 R CNN
F 2 "Connector_Wire:SolderWirePad_1x01_SMD_1x2mm" H 6850 8550 50  0001 C CNN
F 3 "~" H 6850 8550 50  0001 C CNN
	1    6850 8550
	-1   0    0    1   
$EndComp
Wire Wire Line
	6650 8550 6475 8550
Connection ~ 6475 8550
Wire Wire Line
	6475 8550 6475 8525
$Comp
L Connector:Conn_01x01_Male J11
U 1 1 60705ADC
P 5200 8575
F 0 "J11" H 5172 8507 50  0000 R CNN
F 1 "pd" H 5172 8598 50  0000 R CNN
F 2 "Connector_Wire:SolderWirePad_1x01_SMD_1x2mm" H 5200 8575 50  0001 C CNN
F 3 "~" H 5200 8575 50  0001 C CNN
	1    5200 8575
	-1   0    0    1   
$EndComp
$Comp
L Connector:Conn_01x01_Male J10
U 1 1 60706042
P 3550 8575
F 0 "J10" H 3522 8507 50  0000 R CNN
F 1 "pd" H 3522 8598 50  0000 R CNN
F 2 "Connector_Wire:SolderWirePad_1x01_SMD_1x2mm" H 3550 8575 50  0001 C CNN
F 3 "~" H 3550 8575 50  0001 C CNN
	1    3550 8575
	-1   0    0    1   
$EndComp
Wire Wire Line
	3350 8575 2725 8575
Connection ~ 2725 8575
Wire Wire Line
	2725 8575 2725 8525
Wire Wire Line
	5000 8575 4750 8575
Connection ~ 4750 8575
Wire Wire Line
	4750 8575 4750 8525
$Comp
L Connector:Conn_01x01_Male J14
U 1 1 6070C147
P 7225 9125
F 0 "J14" H 7197 9057 50  0000 R CNN
F 1 "pd" H 7197 9148 50  0000 R CNN
F 2 "Connector_Wire:SolderWirePad_1x01_SMD_1x2mm" H 7225 9125 50  0001 C CNN
F 3 "~" H 7225 9125 50  0001 C CNN
	1    7225 9125
	-1   0    0    1   
$EndComp
Wire Wire Line
	7025 9125 6475 9125
Connection ~ 6475 9125
Wire Wire Line
	6475 9125 6475 9200
$Comp
L Connector:Conn_01x01_Male J13
U 1 1 6070F640
P 5600 9125
F 0 "J13" H 5572 9057 50  0000 R CNN
F 1 "pd" H 5572 9148 50  0000 R CNN
F 2 "Connector_Wire:SolderWirePad_1x01_SMD_1x2mm" H 5600 9125 50  0001 C CNN
F 3 "~" H 5600 9125 50  0001 C CNN
	1    5600 9125
	-1   0    0    1   
$EndComp
Wire Wire Line
	5400 9125 4750 9125
Connection ~ 4750 9125
Wire Wire Line
	4750 9125 4750 9200
$Comp
L Connector:Conn_01x01_Male J12
U 1 1 60712DEF
P 3475 9100
F 0 "J12" H 3447 9032 50  0000 R CNN
F 1 "pd" H 3447 9123 50  0000 R CNN
F 2 "Connector_Wire:SolderWirePad_1x01_SMD_1x2mm" H 3475 9100 50  0001 C CNN
F 3 "~" H 3475 9100 50  0001 C CNN
	1    3475 9100
	-1   0    0    1   
$EndComp
Wire Wire Line
	3275 9100 2725 9100
Connection ~ 2725 9100
Wire Wire Line
	2725 9100 2725 9200
$Comp
L Regulator_Linear:AMS1117-3.3 U1
U 1 1 5E7A1557
P 7505 1540
F 0 "U1" H 7505 1782 50  0000 C CNN
F 1 "AMS1117-3.3" H 7505 1691 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-223-3_TabPin2" H 7505 1740 50  0001 C CNN
F 3 "http://www.advanced-monolithic.com/pdf/ds1117.pdf" H 7605 1290 50  0001 C CNN
	1    7505 1540
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C1
U 1 1 5E7AD344
P 7005 1740
F 0 "C1" H 7097 1786 50  0000 L CNN
F 1 "10u" H 7097 1695 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 7005 1740 50  0001 C CNN
F 3 "~" H 7005 1740 50  0001 C CNN
	1    7005 1740
	1    0    0    -1  
$EndComp
Wire Wire Line
	7005 1640 7005 1540
Connection ~ 7005 1540
Wire Wire Line
	7005 1540 7205 1540
$Comp
L Device:C_Small C2
U 1 1 5E7AE84E
P 7955 1740
F 0 "C2" H 8047 1786 50  0000 L CNN
F 1 "10u" H 8047 1695 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 7955 1740 50  0001 C CNN
F 3 "~" H 7955 1740 50  0001 C CNN
	1    7955 1740
	1    0    0    -1  
$EndComp
Wire Wire Line
	7955 1640 7955 1540
Wire Wire Line
	7955 1540 7805 1540
$Comp
L power:GND #PWR08
U 1 1 5E7AF07C
P 7505 1940
F 0 "#PWR08" H 7505 1690 50  0001 C CNN
F 1 "GND" H 7510 1767 50  0000 C CNN
F 2 "" H 7505 1940 50  0001 C CNN
F 3 "" H 7505 1940 50  0001 C CNN
	1    7505 1940
	1    0    0    -1  
$EndComp
Wire Wire Line
	7505 1940 7505 1890
Wire Wire Line
	7005 1840 7005 1890
Wire Wire Line
	7005 1890 7505 1890
Connection ~ 7505 1890
Wire Wire Line
	7505 1890 7505 1840
Wire Wire Line
	7955 1840 7955 1890
Wire Wire Line
	7955 1890 7505 1890
$Comp
L power:+3V3 #PWR04
U 1 1 5E7B11BC
P 7955 1540
F 0 "#PWR04" H 7955 1390 50  0001 C CNN
F 1 "+3V3" H 7970 1713 50  0000 C CNN
F 2 "" H 7955 1540 50  0001 C CNN
F 3 "" H 7955 1540 50  0001 C CNN
	1    7955 1540
	1    0    0    -1  
$EndComp
Connection ~ 7955 1540
Wire Notes Line
	5605 2190 5605 1190
Text Notes 5605 1140 0    50   ~ 0
Input Voltage Regulator
$Comp
L power:+3V3 #PWR019
U 1 1 5E7BB088
P 4920 3525
F 0 "#PWR019" H 4920 3375 50  0001 C CNN
F 1 "+3V3" H 4935 3698 50  0000 C CNN
F 2 "" H 4920 3525 50  0001 C CNN
F 3 "" H 4920 3525 50  0001 C CNN
	1    4920 3525
	1    0    0    -1  
$EndComp
Wire Wire Line
	4920 3625 4920 3575
Wire Wire Line
	5320 3625 5320 3575
Wire Wire Line
	5320 3575 5220 3575
Connection ~ 4920 3575
Wire Wire Line
	4920 3575 4920 3525
Wire Wire Line
	5020 3625 5020 3575
Connection ~ 5020 3575
Wire Wire Line
	5020 3575 4920 3575
Wire Wire Line
	5120 3625 5120 3575
Connection ~ 5120 3575
Wire Wire Line
	5120 3575 5020 3575
Wire Wire Line
	5220 3625 5220 3575
Connection ~ 5220 3575
Wire Wire Line
	5220 3575 5120 3575
$Comp
L power:GND #PWR032
U 1 1 5E7D03F4
P 5020 7325
F 0 "#PWR032" H 5020 7075 50  0001 C CNN
F 1 "GND" H 5025 7152 50  0000 C CNN
F 2 "" H 5020 7325 50  0001 C CNN
F 3 "" H 5020 7325 50  0001 C CNN
	1    5020 7325
	1    0    0    -1  
$EndComp
Wire Wire Line
	5020 7325 5020 7275
Wire Wire Line
	5120 7275 5020 7275
Connection ~ 5020 7275
Wire Wire Line
	5220 7275 5120 7275
Connection ~ 5120 7275
$Comp
L Device:C_Small C5
U 1 1 5E7DA205
P 3445 2925
F 0 "C5" H 3537 2971 50  0000 L CNN
F 1 "100nf" H 3537 2880 50  0000 L CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 3445 2925 50  0001 C CNN
F 3 "~" H 3445 2925 50  0001 C CNN
	1    3445 2925
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C6
U 1 1 5E7DD4E0
P 3845 2925
F 0 "C6" H 3937 2971 50  0000 L CNN
F 1 "100nf" H 3937 2880 50  0000 L CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 3845 2925 50  0001 C CNN
F 3 "~" H 3845 2925 50  0001 C CNN
	1    3845 2925
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C7
U 1 1 5E7DE7F5
P 4245 2925
F 0 "C7" H 4337 2971 50  0000 L CNN
F 1 "100nf" H 4337 2880 50  0000 L CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 4245 2925 50  0001 C CNN
F 3 "~" H 4245 2925 50  0001 C CNN
	1    4245 2925
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C8
U 1 1 5E7DE7FB
P 4645 2925
F 0 "C8" H 4737 2971 50  0000 L CNN
F 1 "100nf" H 4737 2880 50  0000 L CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 4645 2925 50  0001 C CNN
F 3 "~" H 4645 2925 50  0001 C CNN
	1    4645 2925
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C9
U 1 1 5E7DF466
P 5045 2925
F 0 "C9" H 5137 2971 50  0000 L CNN
F 1 "100nf" H 5137 2880 50  0000 L CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 5045 2925 50  0001 C CNN
F 3 "~" H 5045 2925 50  0001 C CNN
	1    5045 2925
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C10
U 1 1 5E7DF46C
P 5445 2925
F 0 "C10" H 5537 2971 50  0000 L CNN
F 1 "100nf" H 5537 2880 50  0000 L CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 5445 2925 50  0001 C CNN
F 3 "~" H 5445 2925 50  0001 C CNN
	1    5445 2925
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C11
U 1 1 5E7E0657
P 5845 2925
F 0 "C11" H 5937 2971 50  0000 L CNN
F 1 "100nf" H 5937 2880 50  0000 L CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 5845 2925 50  0001 C CNN
F 3 "~" H 5845 2925 50  0001 C CNN
	1    5845 2925
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C12
U 1 1 5E7E065D
P 6245 2925
F 0 "C12" H 6337 2971 50  0000 L CNN
F 1 "100nf" H 6337 2880 50  0000 L CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 6245 2925 50  0001 C CNN
F 3 "~" H 6245 2925 50  0001 C CNN
	1    6245 2925
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C4
U 1 1 5E7E130C
P 3045 2925
F 0 "C4" H 3137 2971 50  0000 L CNN
F 1 "100nf" H 3137 2880 50  0000 L CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 3045 2925 50  0001 C CNN
F 3 "~" H 3045 2925 50  0001 C CNN
	1    3045 2925
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C3
U 1 1 5E7E2231
P 2645 2925
F 0 "C3" H 2737 2971 50  0000 L CNN
F 1 "10u" H 2737 2880 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 2645 2925 50  0001 C CNN
F 3 "~" H 2645 2925 50  0001 C CNN
	1    2645 2925
	1    0    0    -1  
$EndComp
Wire Wire Line
	6245 2825 6245 2775
Wire Wire Line
	6245 2775 5845 2775
Wire Wire Line
	2645 2775 2645 2825
Wire Wire Line
	2645 3025 2645 3075
Wire Wire Line
	6245 3075 6245 3025
Wire Wire Line
	5845 2825 5845 2775
Connection ~ 5845 2775
Wire Wire Line
	5845 2775 5445 2775
Wire Wire Line
	5445 2825 5445 2775
Connection ~ 5445 2775
Wire Wire Line
	5445 2775 5045 2775
Wire Wire Line
	5045 2825 5045 2775
Connection ~ 5045 2775
Wire Wire Line
	5045 2775 4645 2775
Wire Wire Line
	4645 2825 4645 2775
Connection ~ 4645 2775
Wire Wire Line
	4645 2775 4245 2775
Wire Wire Line
	4245 2825 4245 2775
Connection ~ 4245 2775
Wire Wire Line
	4245 2775 3845 2775
Wire Wire Line
	3845 2825 3845 2775
Connection ~ 3845 2775
Wire Wire Line
	3845 2775 3445 2775
Wire Wire Line
	3445 2825 3445 2775
Connection ~ 3445 2775
Wire Wire Line
	3445 2775 3045 2775
Wire Wire Line
	3045 2825 3045 2775
Wire Wire Line
	3045 3025 3045 3075
Wire Wire Line
	3045 3075 3445 3075
Wire Wire Line
	3445 3025 3445 3075
Connection ~ 3445 3075
Wire Wire Line
	3445 3075 3845 3075
Wire Wire Line
	3845 3025 3845 3075
Connection ~ 3845 3075
Wire Wire Line
	3845 3075 4245 3075
Wire Wire Line
	4245 3025 4245 3075
Connection ~ 4245 3075
Wire Wire Line
	4645 3025 4645 3075
Wire Wire Line
	4245 3075 4645 3075
Connection ~ 4645 3075
Wire Wire Line
	4645 3075 5045 3075
Wire Wire Line
	5045 3025 5045 3075
Connection ~ 5045 3075
Wire Wire Line
	5045 3075 5445 3075
Wire Wire Line
	5445 3025 5445 3075
Connection ~ 5445 3075
Wire Wire Line
	5445 3075 5845 3075
Wire Wire Line
	5845 3025 5845 3075
Connection ~ 5845 3075
Wire Wire Line
	5845 3075 6245 3075
$Comp
L power:+3V3 #PWR012
U 1 1 5E7F924D
P 2645 2775
F 0 "#PWR012" H 2645 2625 50  0001 C CNN
F 1 "+3V3" H 2660 2948 50  0000 C CNN
F 2 "" H 2645 2775 50  0001 C CNN
F 3 "" H 2645 2775 50  0001 C CNN
	1    2645 2775
	1    0    0    -1  
$EndComp
Connection ~ 2645 2775
$Comp
L power:GND #PWR015
U 1 1 5E7FA209
P 2645 3075
F 0 "#PWR015" H 2645 2825 50  0001 C CNN
F 1 "GND" H 2650 2902 50  0000 C CNN
F 2 "" H 2645 3075 50  0001 C CNN
F 3 "" H 2645 3075 50  0001 C CNN
	1    2645 3075
	1    0    0    -1  
$EndComp
Connection ~ 2645 3075
Wire Wire Line
	2645 2775 3045 2775
Connection ~ 3045 2775
Wire Wire Line
	2645 3075 3045 3075
Connection ~ 3045 3075
$Comp
L Device:C_Small C19
U 1 1 5E81A6C4
P 3820 4475
F 0 "C19" H 3912 4521 50  0000 L CNN
F 1 "2.2u" H 3912 4430 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 3820 4475 50  0001 C CNN
F 3 "~" H 3820 4475 50  0001 C CNN
	1    3820 4475
	1    0    0    -1  
$EndComp
Wire Wire Line
	3820 4375 3820 4225
Wire Wire Line
	3820 4225 4420 4225
$Comp
L power:GND #PWR023
U 1 1 5E81DAC2
P 3820 4575
F 0 "#PWR023" H 3820 4325 50  0001 C CNN
F 1 "GND" H 3825 4402 50  0000 C CNN
F 2 "" H 3820 4575 50  0001 C CNN
F 3 "" H 3820 4575 50  0001 C CNN
	1    3820 4575
	1    0    0    -1  
$EndComp
Text GLabel 2870 3825 0    50   Input ~ 0
NRST
Text GLabel 4420 4025 0    50   Input ~ 0
BOOT0
$Comp
L Device:Crystal_GND24_Small HSE1
U 1 1 5E827EDE
P 7220 6925
F 0 "HSE1" H 6870 7125 50  0000 L CNN
F 1 "16MHz" H 6870 7025 50  0000 L CNN
F 2 "Crystal:Crystal_SMD_3225-4Pin_3.2x2.5mm" H 7220 6925 50  0001 C CNN
F 3 "~" H 7220 6925 50  0001 C CNN
	1    7220 6925
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR030
U 1 1 5E82B851
P 7220 7225
F 0 "#PWR030" H 7220 6975 50  0001 C CNN
F 1 "GND" H 7225 7052 50  0000 C CNN
F 2 "" H 7220 7225 50  0001 C CNN
F 3 "" H 7220 7225 50  0001 C CNN
	1    7220 7225
	1    0    0    -1  
$EndComp
Text GLabel 6870 6925 0    50   Input ~ 0
HSE_IN
Wire Wire Line
	6870 6925 6920 6925
Text GLabel 7770 6925 2    50   Input ~ 0
HSE_OUT
$Comp
L Device:R_Small R5
U 1 1 5E82E580
P 7670 6925
F 0 "R5" V 7474 6925 50  0000 C CNN
F 1 "220" V 7565 6925 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" H 7670 6925 50  0001 C CNN
F 3 "~" H 7670 6925 50  0001 C CNN
	1    7670 6925
	0    1    1    0   
$EndComp
Wire Wire Line
	7570 6925 7520 6925
$Comp
L Device:C_Small C21
U 1 1 5E830A9D
P 7520 7075
F 0 "C21" H 7612 7121 50  0000 L CNN
F 1 "12p" H 7612 7030 50  0000 L CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 7520 7075 50  0001 C CNN
F 3 "~" H 7520 7075 50  0001 C CNN
	1    7520 7075
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C20
U 1 1 5E831218
P 6920 7075
F 0 "C20" H 6720 7125 50  0000 L CNN
F 1 "12p" H 6670 7025 50  0000 L CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 6920 7075 50  0001 C CNN
F 3 "~" H 6920 7075 50  0001 C CNN
	1    6920 7075
	1    0    0    -1  
$EndComp
Wire Wire Line
	6920 6975 6920 6925
Connection ~ 6920 6925
Wire Wire Line
	7520 6975 7520 6925
Connection ~ 7520 6925
Wire Wire Line
	6920 7225 6920 7175
Wire Wire Line
	7520 7225 7520 7175
Connection ~ 7220 7225
Wire Wire Line
	7220 7025 7220 7225
Wire Wire Line
	7220 7225 7370 7225
Wire Wire Line
	7320 6925 7520 6925
Wire Wire Line
	7220 6825 7220 6775
Wire Wire Line
	7220 6775 7370 6775
Wire Wire Line
	7370 6775 7370 7225
Connection ~ 7370 7225
Wire Wire Line
	7370 7225 7520 7225
Wire Wire Line
	6920 6925 7120 6925
Wire Wire Line
	6920 7225 7220 7225
$Comp
L Device:R_Small R2
U 1 1 5E875012
P 6920 5000
F 0 "R2" H 6979 5046 50  0000 L CNN
F 1 "10k" H 6979 4955 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric" H 6920 5000 50  0001 C CNN
F 3 "~" H 6920 5000 50  0001 C CNN
	1    6920 5000
	1    0    0    -1  
$EndComp
Text GLabel 6920 4900 1    50   Input ~ 0
BOOT0
$Comp
L Device:R_Small R3
U 1 1 5E876192
P 7220 5000
F 0 "R3" H 7279 5046 50  0000 L CNN
F 1 "10k" H 7279 4955 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric" H 7220 5000 50  0001 C CNN
F 3 "~" H 7220 5000 50  0001 C CNN
	1    7220 5000
	1    0    0    -1  
$EndComp
Text GLabel 7220 4900 1    50   Input ~ 0
BOOT1
$Comp
L power:GND #PWR024
U 1 1 5E876F8F
P 6920 5100
F 0 "#PWR024" H 6920 4850 50  0001 C CNN
F 1 "GND" H 6925 4927 50  0000 C CNN
F 2 "" H 6920 5100 50  0001 C CNN
F 3 "" H 6920 5100 50  0001 C CNN
	1    6920 5100
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR025
U 1 1 5E877604
P 7220 5100
F 0 "#PWR025" H 7220 4850 50  0001 C CNN
F 1 "GND" H 7225 4927 50  0000 C CNN
F 2 "" H 7220 5100 50  0001 C CNN
F 3 "" H 7220 5100 50  0001 C CNN
	1    7220 5100
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_02x05_Odd_Even J5
U 1 1 5E891CD6
P 14015 1250
F 0 "J5" H 14065 1667 50  0000 C CNN
F 1 "Conn_02x05_Odd_Even" H 14065 1576 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_2x05_P2.54mm_Vertical" H 14015 1250 50  0001 C CNN
F 3 "~" H 14015 1250 50  0001 C CNN
	1    14015 1250
	1    0    0    -1  
$EndComp
$Comp
L power:+3V3 #PWR06
U 1 1 5E893201
P 13815 1050
F 0 "#PWR06" H 13815 900 50  0001 C CNN
F 1 "+3V3" V 13830 1178 50  0000 L CNN
F 2 "" H 13815 1050 50  0001 C CNN
F 3 "" H 13815 1050 50  0001 C CNN
	1    13815 1050
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR010
U 1 1 5E8948BD
P 13715 1550
F 0 "#PWR010" H 13715 1300 50  0001 C CNN
F 1 "GND" H 13720 1377 50  0000 C CNN
F 2 "" H 13715 1550 50  0001 C CNN
F 3 "" H 13715 1550 50  0001 C CNN
	1    13715 1550
	1    0    0    -1  
$EndComp
Wire Wire Line
	13715 1550 13715 1450
Wire Wire Line
	13715 1450 13815 1450
Wire Wire Line
	13815 1250 13715 1250
Wire Wire Line
	13715 1250 13715 1450
Connection ~ 13715 1450
Wire Wire Line
	13815 1150 13715 1150
Wire Wire Line
	13715 1150 13715 1250
Connection ~ 13715 1250
NoConn ~ 13815 1350
NoConn ~ 14315 1350
Text GLabel 14315 1450 2    50   Input ~ 0
NRST
Text GLabel 14315 1050 2    50   Input ~ 0
SWDIO
Text GLabel 14315 1150 2    50   Input ~ 0
SWCLK
Text GLabel 14315 1250 2    50   Input ~ 0
SWO
NoConn ~ 5820 5325
NoConn ~ 5820 5525
NoConn ~ 5820 5625
NoConn ~ 5820 5925
NoConn ~ 5820 6025
NoConn ~ 5820 6625
NoConn ~ 4420 5425
NoConn ~ 4420 6925
NoConn ~ 4420 6825
NoConn ~ 4420 6725
NoConn ~ 4420 6625
NoConn ~ 4420 6525
NoConn ~ 4420 6425
NoConn ~ 4420 6325
$Comp
L Device:LED_Small D2
U 1 1 5E94B3D2
P 8255 1540
F 0 "D2" H 8255 1335 50  0000 C CNN
F 1 "Red" H 8255 1426 50  0000 C CNN
F 2 "LED_SMD:LED_0603_1608Metric" V 8255 1540 50  0001 C CNN
F 3 "~" V 8255 1540 50  0001 C CNN
	1    8255 1540
	-1   0    0    1   
$EndComp
Wire Wire Line
	8155 1540 7955 1540
$Comp
L Device:R_Small R1
U 1 1 5E9509E8
P 8455 1740
F 0 "R1" H 8514 1786 50  0000 L CNN
F 1 "1k" H 8514 1695 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" H 8455 1740 50  0001 C CNN
F 3 "~" H 8455 1740 50  0001 C CNN
	1    8455 1740
	1    0    0    -1  
$EndComp
Wire Wire Line
	8455 1640 8455 1540
Wire Wire Line
	8455 1540 8355 1540
Wire Wire Line
	8455 1840 8455 1890
Wire Wire Line
	8455 1890 7955 1890
Connection ~ 7955 1890
Wire Notes Line
	8705 2190 8705 1190
Wire Notes Line
	5605 1190 8705 1190
Wire Notes Line
	5605 2190 8705 2190
Wire Notes Line
	2495 7525 2495 2525
Wire Notes Line
	2495 2525 8245 2525
Wire Notes Line
	8245 2525 8245 7525
Wire Notes Line
	2495 7525 8245 7525
Text Notes 2495 2475 0    50   ~ 0
Microcontroller
Text Notes 8810 605  0    50   ~ 0
Connectors
$Comp
L MCU_ST_STM32F4:STM32F446RETx U3
U 1 1 6031274F
P 5120 5325
F 0 "U3" H 5120 3436 50  0000 C CNN
F 1 "STM32F446RETx" H 5650 3565 50  0000 C CNN
F 2 "Package_QFP:LQFP-64_10x10mm_P0.5mm" H 4520 3625 50  0001 R CNN
F 3 "http://www.st.com/st-web-ui/static/active/en/resource/technical/document/datasheet/DM00141306.pdf" H 5120 5325 50  0001 C CNN
	1    5120 5325
	1    0    0    -1  
$EndComp
Text GLabel 4420 4925 0    50   Input ~ 0
HSE_IN
Text GLabel 4420 5025 0    50   Input ~ 0
HSE_OUT
NoConn ~ 4420 5225
Wire Wire Line
	5220 7275 5320 7275
Wire Wire Line
	5320 7275 5320 7125
Connection ~ 5220 7275
Wire Wire Line
	5220 7125 5220 7275
Wire Wire Line
	5120 7125 5120 7275
Wire Wire Line
	5020 7125 5020 7275
Wire Wire Line
	5020 7275 4920 7275
Wire Wire Line
	4920 7275 4920 7125
Text GLabel 5820 5825 2    50   Input ~ 0
SWO
Text GLabel 5820 5725 2    50   Input ~ 0
BOOT1
Text GLabel 5820 4625 2    50   Input ~ 0
TIM1_CH1
Text GLabel 5820 4725 2    50   Input ~ 0
TIM1_CH2
Text GLabel 5820 4825 2    50   Input ~ 0
TIM1_CH3
Text GLabel 5820 6725 2    50   Input ~ 0
TIM1_CH1N
Text GLabel 5820 6825 2    50   Input ~ 0
TIM1_CH2N
Text GLabel 5820 6925 2    50   Input ~ 0
TIM1_CH3N
Text GLabel 5820 4025 2    50   Input ~ 0
SOC
Text GLabel 5820 3925 2    50   Input ~ 0
SOB
Text GLabel 5820 3825 2    50   Input ~ 0
SOA
Text GLabel 4420 5525 0    50   Input ~ 0
SPI2_MOSI
Text GLabel 4420 5625 0    50   Input ~ 0
SPI2_MISO
Text GLabel 5820 6125 2    50   Input ~ 0
USART1_TX
Text GLabel 5820 6225 2    50   Input ~ 0
USART1_RX
Text GLabel 4420 5725 0    50   Input ~ 0
SC
Text GLabel 5820 5125 2    50   Input ~ 0
SWDIO
Text GLabel 5820 5225 2    50   Input ~ 0
SWCLK
Wire Wire Line
	6405 1540 7005 1540
Wire Notes Line
	15600 2525 15600 7550
Wire Notes Line
	15600 7550 9775 7550
Wire Notes Line
	9775 7550 9775 2525
Wire Notes Line
	9775 2525 15600 2525
Text Notes 9775 2475 0    50   ~ 0
Mosfet Driver\n
$Comp
L Regulator_Switching:LM2576HVS-12 U4
U 1 1 607F332B
P 3255 1490
F 0 "U4" H 3255 1857 50  0000 C CNN
F 1 "LM2576HVS-12" H 3255 1766 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:TO-263-5_TabPin3" H 3255 1240 50  0001 L CIN
F 3 "http://www.ti.com/lit/ds/symlink/lm2576.pdf" H 3255 1490 50  0001 C CNN
	1    3255 1490
	1    0    0    -1  
$EndComp
$Comp
L Device:CP_Small C13
U 1 1 607F50F2
P 2180 1565
F 0 "C13" H 2268 1611 50  0000 L CNN
F 1 "100u 100v" H 2268 1520 50  0000 L CNN
F 2 "Capacitor_SMD:CP_Elec_10x10" H 2180 1565 50  0001 C CNN
F 3 "~" H 2180 1565 50  0001 C CNN
	1    2180 1565
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0101
U 1 1 607FFB18
P 2180 1840
F 0 "#PWR0101" H 2180 1590 50  0001 C CNN
F 1 "GND" H 2185 1667 50  0000 C CNN
F 2 "" H 2180 1840 50  0001 C CNN
F 3 "" H 2180 1840 50  0001 C CNN
	1    2180 1840
	1    0    0    -1  
$EndComp
Wire Wire Line
	2755 1390 2180 1390
Wire Wire Line
	2180 1390 2180 1465
Wire Wire Line
	2755 1590 2705 1590
$Comp
L power:+48V #PWR0102
U 1 1 6081EFFF
P 2180 1265
F 0 "#PWR0102" H 2180 1115 50  0001 C CNN
F 1 "+48V" H 2195 1438 50  0000 C CNN
F 2 "" H 2180 1265 50  0001 C CNN
F 3 "" H 2180 1265 50  0001 C CNN
	1    2180 1265
	1    0    0    -1  
$EndComp
Wire Wire Line
	2180 1390 2180 1265
Connection ~ 2180 1390
$Comp
L Device:D_Small D1
U 1 1 6083727B
P 3905 1765
F 0 "D1" V 3859 1835 50  0000 L CNN
F 1 "FR2M" V 3950 1835 50  0000 L CNN
F 2 "Diode_SMD:D_SMB" V 3905 1765 50  0001 C CNN
F 3 "~" V 3905 1765 50  0001 C CNN
	1    3905 1765
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR0103
U 1 1 60838876
P 3905 1940
F 0 "#PWR0103" H 3905 1690 50  0001 C CNN
F 1 "GND" H 3910 1767 50  0000 C CNN
F 2 "" H 3905 1940 50  0001 C CNN
F 3 "" H 3905 1940 50  0001 C CNN
	1    3905 1940
	1    0    0    -1  
$EndComp
Wire Wire Line
	2180 1665 2180 1690
$Comp
L power:GND #PWR0104
U 1 1 60842845
P 3255 1915
F 0 "#PWR0104" H 3255 1665 50  0001 C CNN
F 1 "GND" H 3260 1742 50  0000 C CNN
F 2 "" H 3255 1915 50  0001 C CNN
F 3 "" H 3255 1915 50  0001 C CNN
	1    3255 1915
	1    0    0    -1  
$EndComp
Wire Wire Line
	3255 1915 3255 1790
Wire Wire Line
	2705 1590 2705 1690
Wire Wire Line
	2705 1690 2180 1690
Connection ~ 2180 1690
Wire Wire Line
	2180 1690 2180 1840
Wire Wire Line
	3755 1590 3905 1590
Wire Wire Line
	3905 1590 3905 1665
Wire Wire Line
	3905 1865 3905 1940
$Comp
L Device:L L1
U 1 1 60864964
P 4305 1590
F 0 "L1" V 4495 1590 50  0000 C CNN
F 1 "SWRB120" V 4404 1590 50  0000 C CNN
F 2 "Inductor_SMD:L_12x12mm_H4.5mm" H 4305 1590 50  0001 C CNN
F 3 "~" H 4305 1590 50  0001 C CNN
	1    4305 1590
	0    -1   -1   0   
$EndComp
$Comp
L Device:CP_Small C14
U 1 1 6086530B
P 4630 1740
F 0 "C14" H 4718 1786 50  0000 L CNN
F 1 "1000u 25v" H 4680 1615 50  0000 L CNN
F 2 "Capacitor_SMD:CP_Elec_10x10" H 4630 1740 50  0001 C CNN
F 3 "~" H 4630 1740 50  0001 C CNN
	1    4630 1740
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0105
U 1 1 60867897
P 4630 1940
F 0 "#PWR0105" H 4630 1690 50  0001 C CNN
F 1 "GND" H 4635 1767 50  0000 C CNN
F 2 "" H 4630 1940 50  0001 C CNN
F 3 "" H 4630 1940 50  0001 C CNN
	1    4630 1940
	1    0    0    -1  
$EndComp
Wire Wire Line
	4630 1940 4630 1840
Wire Wire Line
	4630 1590 4455 1590
Wire Wire Line
	4155 1590 3905 1590
Wire Wire Line
	4630 1640 4630 1590
Connection ~ 3905 1590
Wire Wire Line
	3755 1390 4630 1390
Wire Wire Line
	4630 1390 4630 1590
Connection ~ 4630 1590
$Comp
L power:+12V #PWR0106
U 1 1 608CB5D8
P 5055 1415
F 0 "#PWR0106" H 5055 1265 50  0001 C CNN
F 1 "+12V" H 5070 1588 50  0000 C CNN
F 2 "" H 5055 1415 50  0001 C CNN
F 3 "" H 5055 1415 50  0001 C CNN
	1    5055 1415
	1    0    0    -1  
$EndComp
Wire Wire Line
	4630 1590 5055 1590
Wire Wire Line
	5055 1590 5055 1415
$Comp
L power:+12V #PWR0107
U 1 1 608DA3B6
P 6405 1465
F 0 "#PWR0107" H 6405 1315 50  0001 C CNN
F 1 "+12V" H 6420 1638 50  0000 C CNN
F 2 "" H 6405 1465 50  0001 C CNN
F 3 "" H 6405 1465 50  0001 C CNN
	1    6405 1465
	1    0    0    -1  
$EndComp
Wire Wire Line
	6405 1540 6405 1465
Text GLabel 12500 4775 0    50   Input ~ 0
TIM1_CH1
Text GLabel 12500 4975 0    50   Input ~ 0
TIM1_CH2
Text GLabel 12500 5175 0    50   Input ~ 0
TIM1_CH3
Text GLabel 12500 4875 0    50   Input ~ 0
TIM1_CH1N
Text GLabel 12500 5075 0    50   Input ~ 0
TIM1_CH2N
Text GLabel 12500 5275 0    50   Input ~ 0
TIM1_CH3N
Text GLabel 12500 4275 0    50   Input ~ 0
DVDD
Text GLabel 12500 5475 0    50   Input ~ 0
DVDD
Text Notes 11625 5500 0    50   ~ 0
for 1000mA Igs\n
Wire Wire Line
	12500 5775 11590 5775
Text GLabel 11335 5885 0    50   Input ~ 0
DVDD
$Comp
L Device:R_Small R15
U 1 1 608B96E7
P 11400 5375
F 0 "R15" H 11220 5455 50  0000 L CNN
F 1 "18k" H 11220 5345 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" H 11400 5375 50  0001 C CNN
F 3 "~" H 11400 5375 50  0001 C CNN
	1    11400 5375
	0    1    1    0   
$EndComp
Text GLabel 11300 5375 0    50   Input ~ 0
DVDD
Wire Wire Line
	12500 5575 11550 5575
Wire Wire Line
	11550 5375 11500 5375
Text GLabel 12500 6075 0    50   Input ~ 0
SOC
Text GLabel 12500 6175 0    50   Input ~ 0
SOB
Text GLabel 12500 6275 0    50   Input ~ 0
SOA
Text GLabel 12500 6375 0    50   Input ~ 0
CAL
Wire Wire Line
	5420 3575 5320 3575
Wire Wire Line
	5420 3575 5420 3625
Connection ~ 5320 3575
$Comp
L Device:C_Small C22
U 1 1 609C38C5
P 6645 2925
F 0 "C22" H 6737 2971 50  0000 L CNN
F 1 "100nf" H 6737 2880 50  0000 L CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 6645 2925 50  0001 C CNN
F 3 "~" H 6645 2925 50  0001 C CNN
	1    6645 2925
	1    0    0    -1  
$EndComp
Wire Wire Line
	6645 2825 6645 2775
Wire Wire Line
	6645 2775 6245 2775
Connection ~ 6245 2775
Wire Wire Line
	6245 3075 6645 3075
Wire Wire Line
	6645 3075 6645 3025
Connection ~ 6245 3075
$Comp
L Device:C_Small C23
U 1 1 60A6F195
P 3545 4100
F 0 "C23" H 3637 4146 50  0000 L CNN
F 1 "100nf" H 3637 4055 50  0000 L CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 3545 4100 50  0001 C CNN
F 3 "~" H 3545 4100 50  0001 C CNN
	1    3545 4100
	1    0    0    -1  
$EndComp
Wire Wire Line
	3545 4000 3545 3825
Wire Wire Line
	3545 3825 4420 3825
Connection ~ 3545 3825
$Comp
L power:GND #PWR0108
U 1 1 60A995B1
P 3545 4325
F 0 "#PWR0108" H 3545 4075 50  0001 C CNN
F 1 "GND" H 3550 4152 50  0000 C CNN
F 2 "" H 3545 4325 50  0001 C CNN
F 3 "" H 3545 4325 50  0001 C CNN
	1    3545 4325
	1    0    0    -1  
$EndComp
Wire Wire Line
	3545 4200 3545 4325
Wire Wire Line
	2870 3825 3545 3825
Text Notes 1400 7975 0    50   ~ 0
Mosfets\n
$Comp
L Mechanical:MountingHole_Pad H3
U 1 1 60DAE865
P 7900 9125
F 0 "H3" V 7854 9275 50  0000 L CNN
F 1 "A" V 7945 9275 50  0000 L CNN
F 2 "MountingHole:MountingHole_2.7mm_Pad" H 7900 9125 50  0001 C CNN
F 3 "~" H 7900 9125 50  0001 C CNN
	1    7900 9125
	0    1    1    0   
$EndComp
$Comp
L Mechanical:MountingHole_Pad H4
U 1 1 60DAE86B
P 7900 9400
F 0 "H4" V 7854 9550 50  0000 L CNN
F 1 "B" V 7945 9550 50  0000 L CNN
F 2 "MountingHole:MountingHole_2.7mm_Pad" H 7900 9400 50  0001 C CNN
F 3 "~" H 7900 9400 50  0001 C CNN
	1    7900 9400
	0    1    1    0   
$EndComp
$Comp
L Mechanical:MountingHole_Pad H5
U 1 1 60DBB78D
P 7900 9675
F 0 "H5" V 7854 9825 50  0000 L CNN
F 1 "C" V 7945 9825 50  0000 L CNN
F 2 "MountingHole:MountingHole_2.7mm_Pad" H 7900 9675 50  0001 C CNN
F 3 "~" H 7900 9675 50  0001 C CNN
	1    7900 9675
	0    1    1    0   
$EndComp
Text GLabel 7800 9675 0    50   Input ~ 0
SHC
Text GLabel 7800 9400 0    50   Input ~ 0
SHB
Text GLabel 7800 9125 0    50   Input ~ 0
SHA
Text GLabel 5820 4325 2    50   Input ~ 0
encoder_1
Text GLabel 5820 4425 2    50   Input ~ 0
encoder_2
$Comp
L Connector_Generic:Conn_02x02_Odd_Even J1
U 1 1 60658A69
P 12665 1700
F 0 "J1" H 12715 1917 50  0000 C CNN
F 1 "encoder" H 12715 1826 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_2x02_P2.54mm_Horizontal" H 12665 1700 50  0001 C CNN
F 3 "~" H 12665 1700 50  0001 C CNN
	1    12665 1700
	1    0    0    -1  
$EndComp
Text GLabel 12465 1800 0    50   Input ~ 0
encoder_1
Text GLabel 12465 1700 0    50   Input ~ 0
encoder_2
$Comp
L power:GND #PWR0114
U 1 1 6065B094
P 13040 1850
F 0 "#PWR0114" H 13040 1600 50  0001 C CNN
F 1 "GND" H 13045 1677 50  0000 C CNN
F 2 "" H 13040 1850 50  0001 C CNN
F 3 "" H 13040 1850 50  0001 C CNN
	1    13040 1850
	1    0    0    -1  
$EndComp
Wire Wire Line
	12965 1800 13040 1800
Wire Wire Line
	13040 1800 13040 1850
Wire Wire Line
	12965 1700 13040 1700
Wire Wire Line
	13040 1700 13040 1625
$Comp
L power:+3V3 #PWR01
U 1 1 606A5262
P 13065 915
F 0 "#PWR01" H 13065 765 50  0001 C CNN
F 1 "+3V3" H 13080 1088 50  0000 C CNN
F 2 "" H 13065 915 50  0001 C CNN
F 3 "" H 13065 915 50  0001 C CNN
	1    13065 915 
	1    0    0    -1  
$EndComp
Wire Wire Line
	12990 990  13065 990 
Wire Wire Line
	13065 990  13065 915 
Text GLabel 5820 6325 2    50   Input ~ 0
i2c1_SCL
Text GLabel 5820 6425 2    50   Input ~ 0
i2c1_SDA
NoConn ~ 5820 6525
Text GLabel 12490 1090 0    50   Input ~ 0
i2c1_SCL
Text GLabel 12490 990  0    50   Input ~ 0
i2c1_SDA
$Comp
L Transistor_FET:CSD19534Q5A Q4
U 1 1 607BF251
P 2625 9500
F 0 "Q4" H 2830 9546 50  0000 L CNN
F 1 "CSD19534Q5A" H 2830 9455 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:TDSON-8-1" H 2825 9425 50  0001 L CIN
F 3 "http://www.ti.com/lit/gpn/csd19534q5a" V 2625 9500 50  0001 L CNN
	1    2625 9500
	1    0    0    -1  
$EndComp
Wire Wire Line
	6475 9075 6475 9125
$Comp
L Transistor_FET:CSD19534Q5A Q6
U 1 1 607C7D5B
P 6375 9500
F 0 "Q6" H 6580 9546 50  0000 L CNN
F 1 "CSD19534Q5A" H 6580 9455 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:TDSON-8-1" H 6575 9425 50  0001 L CIN
F 3 "http://www.ti.com/lit/gpn/csd19534q5a" V 6375 9500 50  0001 L CNN
	1    6375 9500
	1    0    0    -1  
$EndComp
$Comp
L Transistor_FET:CSD19534Q5A Q3
U 1 1 607CED0A
P 6375 8875
F 0 "Q3" H 6580 8921 50  0000 L CNN
F 1 "CSD19534Q5A" H 6580 8830 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:TDSON-8-1" H 6575 8800 50  0001 L CIN
F 3 "http://www.ti.com/lit/gpn/csd19534q5a" V 6375 8875 50  0001 L CNN
	1    6375 8875
	1    0    0    -1  
$EndComp
$Comp
L Transistor_FET:CSD19534Q5A Q2
U 1 1 607D1378
P 4650 8875
F 0 "Q2" H 4855 8921 50  0000 L CNN
F 1 "CSD19534Q5A" H 4855 8830 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:TDSON-8-1" H 4850 8800 50  0001 L CIN
F 3 "http://www.ti.com/lit/gpn/csd19534q5a" V 4650 8875 50  0001 L CNN
	1    4650 8875
	1    0    0    -1  
$EndComp
$Comp
L Transistor_FET:CSD19534Q5A Q5
U 1 1 607D3E69
P 4650 9500
F 0 "Q5" H 4855 9546 50  0000 L CNN
F 1 "CSD19534Q5A" H 4855 9455 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:TDSON-8-1" H 4850 9425 50  0001 L CIN
F 3 "http://www.ti.com/lit/gpn/csd19534q5a" V 4650 9500 50  0001 L CNN
	1    4650 9500
	1    0    0    -1  
$EndComp
$Comp
L Transistor_FET:CSD19534Q5A Q1
U 1 1 607D77DD
P 2625 8875
F 0 "Q1" H 2830 8921 50  0000 L CNN
F 1 "CSD19534Q5A" H 2830 8830 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:TDSON-8-1" H 2825 8800 50  0001 L CIN
F 3 "http://www.ti.com/lit/gpn/csd19534q5a" V 2625 8875 50  0001 L CNN
	1    2625 8875
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C24
U 1 1 60682AA8
P 8575 9350
F 0 "C24" H 8575 9425 50  0000 L CNN
F 1 "1u 100v" H 8400 9275 50  0000 L CNN
F 2 "Capacitor_SMD:C_1206_3216Metric" H 8575 9350 50  0001 C CNN
F 3 "~" H 8575 9350 50  0001 C CNN
	1    8575 9350
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C25
U 1 1 606834AE
P 8850 9350
F 0 "C25" H 8850 9425 50  0000 L CNN
F 1 "1u 100v" H 8750 9275 50  0000 L CNN
F 2 "Capacitor_SMD:C_1206_3216Metric" H 8850 9350 50  0001 C CNN
F 3 "~" H 8850 9350 50  0001 C CNN
	1    8850 9350
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C26
U 1 1 60683752
P 9125 9350
F 0 "C26" H 9125 9425 50  0000 L CNN
F 1 "1u 100v" H 9125 9275 50  0000 L CNN
F 2 "Capacitor_SMD:C_1206_3216Metric" H 9125 9350 50  0001 C CNN
F 3 "~" H 9125 9350 50  0001 C CNN
	1    9125 9350
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0115
U 1 1 60683B86
P 8850 9575
F 0 "#PWR0115" H 8850 9325 50  0001 C CNN
F 1 "GND" H 8855 9402 50  0000 C CNN
F 2 "" H 8850 9575 50  0001 C CNN
F 3 "" H 8850 9575 50  0001 C CNN
	1    8850 9575
	1    0    0    -1  
$EndComp
Wire Wire Line
	8850 9575 8850 9525
$Comp
L power:+48V #PWR0116
U 1 1 60693794
P 8850 9075
F 0 "#PWR0116" H 8850 8925 50  0001 C CNN
F 1 "+48V" H 8865 9248 50  0000 C CNN
F 2 "" H 8850 9075 50  0001 C CNN
F 3 "" H 8850 9075 50  0001 C CNN
	1    8850 9075
	1    0    0    -1  
$EndComp
Wire Wire Line
	8575 9450 8575 9525
Wire Wire Line
	8575 9525 8850 9525
Connection ~ 8850 9525
Wire Wire Line
	8850 9525 8850 9450
Wire Wire Line
	8850 9525 9125 9525
Wire Wire Line
	9125 9525 9125 9450
Wire Wire Line
	8850 9250 8850 9125
Wire Wire Line
	8850 9125 8575 9125
Wire Wire Line
	8575 9125 8575 9250
Connection ~ 8850 9125
Wire Wire Line
	8850 9125 8850 9075
Wire Wire Line
	8850 9125 9125 9125
Wire Wire Line
	9125 9125 9125 9250
NoConn ~ 5820 4225
NoConn ~ 5820 4125
NoConn ~ 5820 4525
NoConn ~ 5820 5025
NoConn ~ 5820 4925
$Comp
L power:+12V #PWR0113
U 1 1 6065F391
P 13040 1625
F 0 "#PWR0113" H 13040 1475 50  0001 C CNN
F 1 "+12V" H 13055 1798 50  0000 C CNN
F 2 "" H 13040 1625 50  0001 C CNN
F 3 "" H 13040 1625 50  0001 C CNN
	1    13040 1625
	1    0    0    -1  
$EndComp
Wire Notes Line
	9670 8000 9670 10660
Wire Notes Line
	9670 10660 1400 10660
Wire Notes Line
	1400 8000 9670 8000
Wire Notes Line
	1400 8000 1400 10660
NoConn ~ 12500 4675
Wire Wire Line
	11550 5375 11550 5575
Wire Wire Line
	11590 5885 11590 5775
Wire Wire Line
	12500 5675 11525 5675
Text GLabel 11325 5675 0    50   Input ~ 0
DVDD
$Comp
L Device:R_Small R16
U 1 1 6089F2A6
P 11425 5675
F 0 "R16" H 11230 5680 50  0000 L CNN
F 1 "1M" H 11260 5590 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" H 11425 5675 50  0001 C CNN
F 3 "~" H 11425 5675 50  0001 C CNN
	1    11425 5675
	0    1    1    0   
$EndComp
Text GLabel 4420 6025 0    50   Input ~ 0
H1
Text GLabel 4420 6125 0    50   Input ~ 0
H2
Text GLabel 4420 6225 0    50   Input ~ 0
H3
Wire Wire Line
	11535 5885 11590 5885
$Comp
L Device:R_Small R17
U 1 1 6087B27F
P 11435 5885
F 0 "R17" H 11494 5931 50  0000 L CNN
F 1 "1M" H 11494 5840 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" H 11435 5885 50  0001 C CNN
F 3 "~" H 11435 5885 50  0001 C CNN
	1    11435 5885
	0    1    1    0   
$EndComp
NoConn ~ 4420 5825
NoConn ~ 4420 5925
Wire Notes Line
	5250 2195 1855 2195
Wire Notes Line
	1855 2195 1855 985 
Wire Notes Line
	1855 985  5300 985 
Wire Notes Line
	5300 985  5300 2195
Wire Notes Line
	5300 2195 5255 2195
Text Notes 1845 960  0    50   ~ 0
48-12V conv\n
Wire Wire Line
	13065 1090 13065 1140
Wire Wire Line
	12990 1090 13065 1090
$Comp
L power:GND #PWR02
U 1 1 606A5268
P 13065 1140
F 0 "#PWR02" H 13065 890 50  0001 C CNN
F 1 "GND" H 13070 967 50  0000 C CNN
F 2 "" H 13065 1140 50  0001 C CNN
F 3 "" H 13065 1140 50  0001 C CNN
	1    13065 1140
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_02x02_Odd_Even J3
U 1 1 60693514
P 12690 990
F 0 "J3" H 12740 1207 50  0000 C CNN
F 1 "I2C" H 12740 1116 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_2x02_P2.54mm_Horizontal" H 12690 990 50  0001 C CNN
F 3 "~" H 12690 990 50  0001 C CNN
	1    12690 990 
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x02 J2
U 1 1 5EA2EE74
P 11445 780
F 0 "J2" H 11525 772 50  0000 L CNN
F 1 "USART" H 11525 681 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Horizontal" H 11445 780 50  0001 C CNN
F 3 "~" H 11445 780 50  0001 C CNN
	1    11445 780 
	1    0    0    -1  
$EndComp
Text GLabel 11245 880  0    50   Input ~ 0
USART1_TX
Text GLabel 11245 780  0    50   Input ~ 0
USART1_RX
$Comp
L power:+3V3 #PWR09
U 1 1 5EA64AA3
P 10920 1440
F 0 "#PWR09" H 10920 1290 50  0001 C CNN
F 1 "+3V3" H 10935 1613 50  0000 C CNN
F 2 "" H 10920 1440 50  0001 C CNN
F 3 "" H 10920 1440 50  0001 C CNN
	1    10920 1440
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole_Pad H1
U 1 1 60D033E4
P 9270 1080
F 0 "H1" V 9224 1230 50  0000 L CNN
F 1 "48v" V 9315 1230 50  0000 L CNN
F 2 "MountingHole:MountingHole_2.7mm_Pad" H 9270 1080 50  0001 C CNN
F 3 "~" H 9270 1080 50  0001 C CNN
	1    9270 1080
	0    1    1    0   
$EndComp
$Comp
L Mechanical:MountingHole_Pad H2
U 1 1 60D1247F
P 9270 1355
F 0 "H2" V 9224 1505 50  0000 L CNN
F 1 "48vgnd" V 9315 1505 50  0000 L CNN
F 2 "MountingHole:MountingHole_2.7mm_Pad" H 9270 1355 50  0001 C CNN
F 3 "~" H 9270 1355 50  0001 C CNN
	1    9270 1355
	0    1    1    0   
$EndComp
$Comp
L power:+48V #PWR0109
U 1 1 60D2BA45
P 9070 1055
F 0 "#PWR0109" H 9070 905 50  0001 C CNN
F 1 "+48V" H 9085 1228 50  0000 C CNN
F 2 "" H 9070 1055 50  0001 C CNN
F 3 "" H 9070 1055 50  0001 C CNN
	1    9070 1055
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0110
U 1 1 60D2C48F
P 9070 1430
F 0 "#PWR0110" H 9070 1180 50  0001 C CNN
F 1 "GND" H 9075 1257 50  0000 C CNN
F 2 "" H 9070 1430 50  0001 C CNN
F 3 "" H 9070 1430 50  0001 C CNN
	1    9070 1430
	1    0    0    -1  
$EndComp
Wire Wire Line
	9170 1355 9070 1355
Wire Wire Line
	9070 1355 9070 1430
Wire Wire Line
	9170 1080 9070 1080
Wire Wire Line
	9070 1080 9070 1055
$Comp
L power:GND #PWR0111
U 1 1 60E565A8
P 9970 1430
F 0 "#PWR0111" H 9970 1180 50  0001 C CNN
F 1 "GND" H 9975 1257 50  0000 C CNN
F 2 "" H 9970 1430 50  0001 C CNN
F 3 "" H 9970 1430 50  0001 C CNN
	1    9970 1430
	1    0    0    -1  
$EndComp
Wire Wire Line
	10070 1355 9970 1355
Wire Wire Line
	9970 1355 9970 1430
$Comp
L Connector:Conn_01x01_Female J4
U 1 1 606836F9
P 10320 1085
F 0 "J4" H 10348 1111 50  0000 L CNN
F 1 "12v" H 10348 1020 50  0000 L CNN
F 2 "Connector_Wire:SolderWirePad_1x01_SMD_1x2mm" H 10320 1085 50  0001 C CNN
F 3 "~" H 10320 1085 50  0001 C CNN
	1    10320 1085
	1    0    0    -1  
$EndComp
Wire Wire Line
	9970 1055 9970 1085
$Comp
L power:+12V #PWR0112
U 1 1 60E712FA
P 9970 1055
F 0 "#PWR0112" H 9970 905 50  0001 C CNN
F 1 "+12V" H 9985 1228 50  0000 C CNN
F 2 "" H 9970 1055 50  0001 C CNN
F 3 "" H 9970 1055 50  0001 C CNN
	1    9970 1055
	1    0    0    -1  
$EndComp
Wire Wire Line
	10120 1085 9970 1085
$Comp
L Connector:Conn_01x01_Female J7
U 1 1 606C8714
P 10270 1355
F 0 "J7" H 10298 1381 50  0000 L CNN
F 1 "12vgnd" H 10298 1290 50  0000 L CNN
F 2 "Connector_Wire:SolderWirePad_1x01_SMD_1x2mm" H 10270 1355 50  0001 C CNN
F 3 "~" H 10270 1355 50  0001 C CNN
	1    10270 1355
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_02x03_Odd_Even J6
U 1 1 6072557D
P 11310 1650
F 0 "J6" H 11360 1967 50  0000 C CNN
F 1 "Hall_EFF" H 11360 1876 50  0000 C CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_2x03_P2.54mm_Horizontal" H 11310 1650 50  0001 C CNN
F 3 "~" H 11310 1650 50  0001 C CNN
	1    11310 1650
	-1   0    0    -1  
$EndComp
Wire Wire Line
	11010 1550 10920 1550
Wire Wire Line
	10920 1550 10920 1440
$Comp
L power:GND #PWR0117
U 1 1 607468AE
P 10920 1810
F 0 "#PWR0117" H 10920 1560 50  0001 C CNN
F 1 "GND" H 10925 1637 50  0000 C CNN
F 2 "" H 10920 1810 50  0001 C CNN
F 3 "" H 10920 1810 50  0001 C CNN
	1    10920 1810
	1    0    0    -1  
$EndComp
Wire Wire Line
	11010 1750 10920 1750
Wire Wire Line
	10920 1750 10920 1810
Text GLabel 11510 1750 2    50   Input ~ 0
H1
Text GLabel 11510 1650 2    50   Input ~ 0
H2
Text GLabel 11510 1550 2    50   Input ~ 0
H3
NoConn ~ 11010 1650
Wire Notes Line
	14665 2190 8815 2190
Wire Notes Line
	8815 2190 8815 650 
Wire Notes Line
	8815 650  14665 650 
Wire Notes Line
	14665 650  14665 2190
$EndSCHEMATC
