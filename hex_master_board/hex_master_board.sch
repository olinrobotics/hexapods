EESchema Schematic File Version 4
LIBS:hex_master_board-cache
EELAYER 26 0
EELAYER END
$Descr A3 16535 11693
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
L hex_master_sym:Jack_CP-002B-ND U1
U 1 1 5CB747C3
P 1450 1550
F 0 "U1" H 1506 2115 50  0000 C CNN
F 1 "Jack_CP-002B-ND" H 1506 2024 50  0000 C CNN
F 2 "hex_master_board:Jack_CP-002B-ND" H 1450 1500 50  0001 C CNN
F 3 "" H 1450 1500 50  0001 C CNN
	1    1450 1550
	1    0    0    -1  
$EndComp
$Comp
L Regulator_Linear:MCP1700-5002E_SOT23 U3
U 1 1 5CB7486F
P 2800 1250
F 0 "U3" H 2800 1492 50  0000 C CNN
F 1 "MCP1700-5002E_SOT23" H 2800 1401 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 2800 1475 50  0001 C CNN
F 3 "http://ww1.microchip.com/downloads/en/DeviceDoc/20001826C.pdf" H 2800 1250 50  0001 C CNN
	1    2800 1250
	1    0    0    -1  
$EndComp
Wire Wire Line
	1700 1250 2350 1250
$Comp
L hex_master_sym:TPS2113A U5
U 1 1 5CB74C1B
P 4400 2250
F 0 "U5" H 4400 3065 50  0000 C CNN
F 1 "TPS2113A" H 4400 2974 50  0000 C CNN
F 2 "hex_master_board:TPS2113ADRBR" H 4400 2050 50  0001 C CNN
F 3 "" H 4400 2050 50  0001 C CNN
	1    4400 2250
	1    0    0    -1  
$EndComp
Wire Wire Line
	3100 1250 3200 1250
$Comp
L Device:R R9
U 1 1 5CB74FAD
P 3500 1750
F 0 "R9" H 3570 1796 50  0000 L CNN
F 1 "20K" H 3570 1705 50  0000 L CNN
F 2 "" V 3430 1750 50  0001 C CNN
F 3 "~" H 3500 1750 50  0001 C CNN
	1    3500 1750
	1    0    0    -1  
$EndComp
Wire Wire Line
	3500 2050 3950 2050
$Comp
L Device:R R10
U 1 1 5CB75089
P 3500 2400
F 0 "R10" H 3570 2446 50  0000 L CNN
F 1 "10K" H 3570 2355 50  0000 L CNN
F 2 "" V 3430 2400 50  0001 C CNN
F 3 "~" H 3500 2400 50  0001 C CNN
	1    3500 2400
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0101
U 1 1 5CB75211
P 3500 2850
F 0 "#PWR0101" H 3500 2600 50  0001 C CNN
F 1 "GND" H 3505 2677 50  0000 C CNN
F 2 "" H 3500 2850 50  0001 C CNN
F 3 "" H 3500 2850 50  0001 C CNN
	1    3500 2850
	1    0    0    -1  
$EndComp
Wire Wire Line
	3950 2200 3900 2200
$Comp
L Device:R R11
U 1 1 5CB7530A
P 3900 2600
F 0 "R11" H 3970 2646 50  0000 L CNN
F 1 "1K" H 3970 2555 50  0000 L CNN
F 2 "" V 3830 2600 50  0001 C CNN
F 3 "~" H 3900 2600 50  0001 C CNN
	1    3900 2600
	1    0    0    -1  
$EndComp
Wire Wire Line
	3900 2200 3900 2450
Wire Wire Line
	3900 2750 3900 2850
$Comp
L power:GND #PWR0102
U 1 1 5CB75537
P 3900 2850
F 0 "#PWR0102" H 3900 2600 50  0001 C CNN
F 1 "GND" H 3905 2677 50  0000 C CNN
F 2 "" H 3900 2850 50  0001 C CNN
F 3 "" H 3900 2850 50  0001 C CNN
	1    3900 2850
	1    0    0    -1  
$EndComp
Wire Wire Line
	3500 2550 3500 2850
Wire Wire Line
	3950 1900 3750 1900
$Comp
L power:GND #PWR0103
U 1 1 5CB75883
P 3750 2850
F 0 "#PWR0103" H 3750 2600 50  0001 C CNN
F 1 "GND" H 3755 2677 50  0000 C CNN
F 2 "" H 3750 2850 50  0001 C CNN
F 3 "" H 3750 2850 50  0001 C CNN
	1    3750 2850
	1    0    0    -1  
$EndComp
Wire Wire Line
	3750 1900 3750 2850
NoConn ~ 3950 1750
Wire Wire Line
	5550 1900 5550 2100
$Comp
L Device:C C10
U 1 1 5CB75CE1
P 5550 2250
F 0 "C10" H 5665 2296 50  0000 L CNN
F 1 "10uF" H 5665 2205 50  0000 L CNN
F 2 "" H 5588 2100 50  0001 C CNN
F 3 "~" H 5550 2250 50  0001 C CNN
	1    5550 2250
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0104
U 1 1 5CB75D40
P 5550 2850
F 0 "#PWR0104" H 5550 2600 50  0001 C CNN
F 1 "GND" H 5555 2677 50  0000 C CNN
F 2 "" H 5550 2850 50  0001 C CNN
F 3 "" H 5550 2850 50  0001 C CNN
	1    5550 2850
	1    0    0    -1  
$EndComp
Wire Wire Line
	5900 1750 5900 1900
$Comp
L power:+5V #PWR0105
U 1 1 5CB76276
P 5900 1750
F 0 "#PWR0105" H 5900 1600 50  0001 C CNN
F 1 "+5V" H 5915 1923 50  0000 C CNN
F 2 "" H 5900 1750 50  0001 C CNN
F 3 "" H 5900 1750 50  0001 C CNN
	1    5900 1750
	1    0    0    -1  
$EndComp
Wire Wire Line
	4850 2200 5000 2200
$Comp
L power:GND #PWR0106
U 1 1 5CB76553
P 5000 2850
F 0 "#PWR0106" H 5000 2600 50  0001 C CNN
F 1 "GND" H 5005 2677 50  0000 C CNN
F 2 "" H 5000 2850 50  0001 C CNN
F 3 "" H 5000 2850 50  0001 C CNN
	1    5000 2850
	1    0    0    -1  
$EndComp
Wire Wire Line
	5900 2050 5900 1900
Connection ~ 5900 1900
$Comp
L Device:LED D3
U 1 1 5CB76A6A
P 5900 2200
F 0 "D3" V 5938 2083 50  0000 R CNN
F 1 "POWER" V 5847 2083 50  0000 R CNN
F 2 "" H 5900 2200 50  0001 C CNN
F 3 "~" H 5900 2200 50  0001 C CNN
	1    5900 2200
	0    -1   -1   0   
$EndComp
Wire Wire Line
	5900 2350 5900 2450
$Comp
L Device:R R13
U 1 1 5CB76E39
P 5900 2600
F 0 "R13" H 5970 2646 50  0000 L CNN
F 1 "R" H 5970 2555 50  0000 L CNN
F 2 "" V 5830 2600 50  0001 C CNN
F 3 "~" H 5900 2600 50  0001 C CNN
	1    5900 2600
	1    0    0    -1  
$EndComp
Wire Wire Line
	5900 2850 5900 2750
$Comp
L power:GND #PWR0107
U 1 1 5CB7714D
P 5900 2850
F 0 "#PWR0107" H 5900 2600 50  0001 C CNN
F 1 "GND" H 5905 2677 50  0000 C CNN
F 2 "" H 5900 2850 50  0001 C CNN
F 3 "" H 5900 2850 50  0001 C CNN
	1    5900 2850
	1    0    0    -1  
$EndComp
Wire Wire Line
	5000 2200 5000 2850
Wire Wire Line
	5550 2400 5550 2850
$Comp
L Connector:USB_B_Micro J1
U 1 1 5CB75C1C
P 1300 4800
F 0 "J1" H 1355 5267 50  0000 C CNN
F 1 "USB_B_Micro" H 1355 5176 50  0000 C CNN
F 2 "hex_master_board:USB micro 609-4616-1-ND" H 1450 4750 50  0001 C CNN
F 3 "~" H 1450 4750 50  0001 C CNN
	1    1300 4800
	1    0    0    -1  
$EndComp
NoConn ~ 1200 5200
$Comp
L power:GND #PWR0108
U 1 1 5CB7600E
P 1300 5300
F 0 "#PWR0108" H 1300 5050 50  0001 C CNN
F 1 "GND" H 1305 5127 50  0000 C CNN
F 2 "" H 1300 5300 50  0001 C CNN
F 3 "" H 1300 5300 50  0001 C CNN
	1    1300 5300
	1    0    0    -1  
$EndComp
Wire Wire Line
	1300 5300 1300 5200
Text Label 1900 4600 0    50   ~ 0
USB_V+
Wire Wire Line
	1600 4600 1900 4600
$Comp
L Device:C C8
U 1 1 5CB78AB3
P 5200 2500
F 0 "C8" H 5315 2546 50  0000 L CNN
F 1 "10uF" H 5315 2455 50  0000 L CNN
F 2 "" H 5238 2350 50  0001 C CNN
F 3 "~" H 5200 2500 50  0001 C CNN
	1    5200 2500
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0109
U 1 1 5CB78AFD
P 5200 2850
F 0 "#PWR0109" H 5200 2600 50  0001 C CNN
F 1 "GND" H 5205 2677 50  0000 C CNN
F 2 "" H 5200 2850 50  0001 C CNN
F 3 "" H 5200 2850 50  0001 C CNN
	1    5200 2850
	1    0    0    -1  
$EndComp
Wire Wire Line
	5200 2050 5200 2350
Wire Wire Line
	5200 2650 5200 2850
Wire Wire Line
	4850 2050 5200 2050
Wire Wire Line
	3500 1250 5000 1250
Wire Wire Line
	5000 1250 5000 1750
Wire Wire Line
	5000 1750 4850 1750
Connection ~ 3500 1250
Text Label 5200 1600 1    50   ~ 0
USB_V+
Wire Wire Line
	5200 1600 5200 2050
Connection ~ 5200 2050
$Comp
L Device:C C4
U 1 1 5CB7BA87
P 3200 1500
F 0 "C4" H 3315 1546 50  0000 L CNN
F 1 "1uF" H 3315 1455 50  0000 L CNN
F 2 "" H 3238 1350 50  0001 C CNN
F 3 "~" H 3200 1500 50  0001 C CNN
	1    3200 1500
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0110
U 1 1 5CB7BADD
P 3200 1800
F 0 "#PWR0110" H 3200 1550 50  0001 C CNN
F 1 "GND" H 3205 1627 50  0000 C CNN
F 2 "" H 3200 1800 50  0001 C CNN
F 3 "" H 3200 1800 50  0001 C CNN
	1    3200 1800
	1    0    0    -1  
$EndComp
Wire Wire Line
	3200 1350 3200 1250
Wire Wire Line
	3200 1800 3200 1650
Wire Wire Line
	3500 2050 3500 2250
Wire Wire Line
	3500 1250 3500 1600
Wire Wire Line
	3500 1900 3500 2050
Connection ~ 3500 2050
Connection ~ 3200 1250
Wire Wire Line
	3200 1250 3500 1250
$Comp
L power:GND #PWR0111
U 1 1 5CB7F824
P 2800 1800
F 0 "#PWR0111" H 2800 1550 50  0001 C CNN
F 1 "GND" H 2805 1627 50  0000 C CNN
F 2 "" H 2800 1800 50  0001 C CNN
F 3 "" H 2800 1800 50  0001 C CNN
	1    2800 1800
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0112
U 1 1 5CB7F85E
P 2350 1800
F 0 "#PWR0112" H 2350 1550 50  0001 C CNN
F 1 "GND" H 2355 1627 50  0000 C CNN
F 2 "" H 2350 1800 50  0001 C CNN
F 3 "" H 2350 1800 50  0001 C CNN
	1    2350 1800
	1    0    0    -1  
$EndComp
$Comp
L Device:C C1
U 1 1 5CB7F8A6
P 2350 1500
F 0 "C1" H 2465 1546 50  0000 L CNN
F 1 "1uF" H 2465 1455 50  0000 L CNN
F 2 "" H 2388 1350 50  0001 C CNN
F 3 "~" H 2350 1500 50  0001 C CNN
	1    2350 1500
	1    0    0    -1  
$EndComp
Wire Wire Line
	2350 1350 2350 1250
Connection ~ 2350 1250
Wire Wire Line
	2350 1250 2500 1250
Wire Wire Line
	2350 1650 2350 1800
Wire Wire Line
	2800 1550 2800 1800
$Comp
L power:GND #PWR0113
U 1 1 5CB8188E
P 1800 1600
F 0 "#PWR0113" H 1800 1350 50  0001 C CNN
F 1 "GND" H 1805 1427 50  0000 C CNN
F 2 "" H 1800 1600 50  0001 C CNN
F 3 "" H 1800 1600 50  0001 C CNN
	1    1800 1600
	1    0    0    -1  
$EndComp
Wire Wire Line
	1700 1450 1800 1450
Wire Wire Line
	1800 1450 1800 1600
$Comp
L Interface_USB:FT232RL U4
U 1 1 5CBA34B9
P 3350 5200
F 0 "U4" H 3350 6378 50  0000 C CNN
F 1 "FT232RL" H 3350 6287 50  0000 C CNN
F 2 "Package_SO:SSOP-28_5.3x10.2mm_P0.65mm" H 3350 5200 50  0001 C CNN
F 3 "http://www.ftdichip.com/Products/ICs/FT232RL.htm" H 3350 5200 50  0001 C CNN
	1    3350 5200
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR0114
U 1 1 5CBA369C
P 3250 3550
F 0 "#PWR0114" H 3250 3400 50  0001 C CNN
F 1 "+5V" H 3265 3723 50  0000 C CNN
F 2 "" H 3250 3550 50  0001 C CNN
F 3 "" H 3250 3550 50  0001 C CNN
	1    3250 3550
	1    0    0    -1  
$EndComp
Wire Wire Line
	3150 6200 3150 6300
Wire Wire Line
	3150 6300 3350 6300
Wire Wire Line
	3550 6300 3550 6200
Wire Wire Line
	3450 6200 3450 6300
Connection ~ 3450 6300
Wire Wire Line
	3450 6300 3550 6300
Wire Wire Line
	3350 6200 3350 6300
Connection ~ 3350 6300
Wire Wire Line
	3350 6300 3450 6300
Connection ~ 3150 6300
$Comp
L power:+5V #PWR0115
U 1 1 5CBB34E1
P 3450 3550
F 0 "#PWR0115" H 3450 3400 50  0001 C CNN
F 1 "+5V" H 3465 3723 50  0000 C CNN
F 2 "" H 3450 3550 50  0001 C CNN
F 3 "" H 3450 3550 50  0001 C CNN
	1    3450 3550
	1    0    0    -1  
$EndComp
Wire Wire Line
	1600 4800 2550 4800
Wire Wire Line
	1600 4900 2550 4900
Wire Wire Line
	2550 5900 2450 5900
Wire Wire Line
	2450 5900 2450 6300
Wire Wire Line
	2450 6300 3150 6300
NoConn ~ 2550 5400
NoConn ~ 2550 5600
NoConn ~ 2550 5200
NoConn ~ 2550 4500
NoConn ~ 4150 4700
NoConn ~ 4150 4800
NoConn ~ 4150 5000
NoConn ~ 4150 5100
NoConn ~ 4150 5700
NoConn ~ 4150 5800
NoConn ~ 4150 5900
$Comp
L power:GND #PWR0116
U 1 1 5CBC1B3C
P 2450 6400
F 0 "#PWR0116" H 2450 6150 50  0001 C CNN
F 1 "GND" H 2455 6227 50  0000 C CNN
F 2 "" H 2450 6400 50  0001 C CNN
F 3 "" H 2450 6400 50  0001 C CNN
	1    2450 6400
	1    0    0    -1  
$EndComp
Wire Wire Line
	2450 6400 2450 6300
Connection ~ 2450 6300
Text Notes 1050 2550 0    138  ~ 0
Power Multiplexing
Text Notes 1250 4200 0    138  ~ 0
USB/UART\n\n
$Comp
L Device:C C6
U 1 1 5CBCC061
P 4500 4900
F 0 "C6" V 4248 4900 50  0000 C CNN
F 1 "0.1uF" V 4339 4900 50  0000 C CNN
F 2 "" H 4538 4750 50  0001 C CNN
F 3 "~" H 4500 4900 50  0001 C CNN
	1    4500 4900
	0    1    1    0   
$EndComp
Wire Wire Line
	4650 4900 4750 4900
$Comp
L Device:R R12
U 1 1 5CBCEC31
P 4750 4100
F 0 "R12" H 4820 4146 50  0000 L CNN
F 1 "1k" H 4820 4055 50  0000 L CNN
F 2 "" V 4680 4100 50  0001 C CNN
F 3 "~" H 4750 4100 50  0001 C CNN
	1    4750 4100
	1    0    0    -1  
$EndComp
Wire Wire Line
	4750 4900 4750 4850
Connection ~ 4750 4900
Wire Wire Line
	4750 3950 4750 3850
$Comp
L power:+5V #PWR0117
U 1 1 5CBD0F40
P 4750 3850
F 0 "#PWR0117" H 4750 3700 50  0001 C CNN
F 1 "+5V" H 4765 4023 50  0000 C CNN
F 2 "" H 4750 3850 50  0001 C CNN
F 3 "" H 4750 3850 50  0001 C CNN
	1    4750 3850
	1    0    0    -1  
$EndComp
Wire Wire Line
	4750 4850 4800 4850
Wire Wire Line
	4800 4850 4800 4900
Connection ~ 4750 4850
Wire Wire Line
	4750 4250 4750 4850
Wire Wire Line
	3450 3550 3450 3650
Wire Wire Line
	3450 3650 3700 3650
Wire Wire Line
	3700 3650 3700 3750
Connection ~ 3450 3650
Wire Wire Line
	2850 3650 2850 3750
$Comp
L Device:C C5
U 1 1 5CBFDE20
P 3700 3900
F 0 "C5" H 3815 3946 50  0000 L CNN
F 1 "0.1uF" H 3815 3855 50  0000 L CNN
F 2 "" H 3738 3750 50  0001 C CNN
F 3 "~" H 3700 3900 50  0001 C CNN
	1    3700 3900
	1    0    0    -1  
$EndComp
$Comp
L Device:C C2
U 1 1 5CBFDE7E
P 2850 3900
F 0 "C2" H 2965 3946 50  0000 L CNN
F 1 "0.1uF" H 2965 3855 50  0000 L CNN
F 2 "" H 2888 3750 50  0001 C CNN
F 3 "~" H 2850 3900 50  0001 C CNN
	1    2850 3900
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0118
U 1 1 5CBFDED9
P 3700 4050
F 0 "#PWR0118" H 3700 3800 50  0001 C CNN
F 1 "GND" H 3705 3877 50  0000 C CNN
F 2 "" H 3700 4050 50  0001 C CNN
F 3 "" H 3700 4050 50  0001 C CNN
	1    3700 4050
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0119
U 1 1 5CBFDF8B
P 2850 4050
F 0 "#PWR0119" H 2850 3800 50  0001 C CNN
F 1 "GND" H 2855 3877 50  0000 C CNN
F 2 "" H 2850 4050 50  0001 C CNN
F 3 "" H 2850 4050 50  0001 C CNN
	1    2850 4050
	1    0    0    -1  
$EndComp
Wire Wire Line
	3450 3650 3450 4200
Wire Wire Line
	3250 3550 3250 3650
Wire Wire Line
	2850 3650 3250 3650
Connection ~ 3250 3650
Wire Wire Line
	3250 3650 3250 4200
Wire Wire Line
	4850 1900 5550 1900
Wire Wire Line
	5550 1900 5900 1900
Connection ~ 5550 1900
$Comp
L power:+5V #PWR0120
U 1 1 5CC26DED
P 7000 3750
F 0 "#PWR0120" H 7000 3600 50  0001 C CNN
F 1 "+5V" H 7015 3923 50  0000 C CNN
F 2 "" H 7000 3750 50  0001 C CNN
F 3 "" H 7000 3750 50  0001 C CNN
	1    7000 3750
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR0121
U 1 1 5CC26E3B
P 7100 3750
F 0 "#PWR0121" H 7100 3600 50  0001 C CNN
F 1 "+5V" H 7115 3923 50  0000 C CNN
F 2 "" H 7100 3750 50  0001 C CNN
F 3 "" H 7100 3750 50  0001 C CNN
	1    7100 3750
	1    0    0    -1  
$EndComp
$Comp
L Device:C C13
U 1 1 5CC26E90
P 7250 4200
F 0 "C13" H 7365 4246 50  0000 L CNN
F 1 "0.1uF" H 7365 4155 50  0000 L CNN
F 2 "" H 7288 4050 50  0001 C CNN
F 3 "~" H 7250 4200 50  0001 C CNN
	1    7250 4200
	1    0    0    -1  
$EndComp
$Comp
L Device:C C12
U 1 1 5CC26F15
P 6650 4200
F 0 "C12" H 6765 4246 50  0000 L CNN
F 1 "0.1uF" H 6765 4155 50  0000 L CNN
F 2 "" H 6688 4050 50  0001 C CNN
F 3 "~" H 6650 4200 50  0001 C CNN
	1    6650 4200
	1    0    0    -1  
$EndComp
Wire Wire Line
	6650 4050 6650 3950
Wire Wire Line
	7100 3950 7250 3950
Wire Wire Line
	7250 3950 7250 4050
Connection ~ 7100 3950
Wire Wire Line
	7100 3950 7100 3750
$Comp
L power:GND #PWR0122
U 1 1 5CC2B31D
P 6650 4400
F 0 "#PWR0122" H 6650 4150 50  0001 C CNN
F 1 "GND" H 6655 4227 50  0000 C CNN
F 2 "" H 6650 4400 50  0001 C CNN
F 3 "" H 6650 4400 50  0001 C CNN
	1    6650 4400
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0123
U 1 1 5CC2B36F
P 7250 4400
F 0 "#PWR0123" H 7250 4150 50  0001 C CNN
F 1 "GND" H 7255 4227 50  0000 C CNN
F 2 "" H 7250 4400 50  0001 C CNN
F 3 "" H 7250 4400 50  0001 C CNN
	1    7250 4400
	1    0    0    -1  
$EndComp
Wire Wire Line
	7250 4400 7250 4350
Wire Wire Line
	6650 4400 6650 4350
Wire Wire Line
	7000 3750 7000 3950
Wire Wire Line
	6650 3950 7000 3950
Connection ~ 7000 3950
$Comp
L Device:Crystal Y1
U 1 1 5CC47B4E
P 5500 5200
F 0 "Y1" V 5454 5331 50  0000 L CNN
F 1 "Crystal" V 5545 5331 50  0000 L CNN
F 2 "" H 5500 5200 50  0001 C CNN
F 3 "~" H 5500 5200 50  0001 C CNN
	1    5500 5200
	0    1    1    0   
$EndComp
Wire Wire Line
	5800 5100 5800 5000
Wire Wire Line
	5800 5000 5500 5000
Wire Wire Line
	5500 5000 5500 5050
Wire Wire Line
	5800 5300 5800 5400
Wire Wire Line
	5800 5400 5500 5400
Wire Wire Line
	5500 5400 5500 5350
Connection ~ 5500 5000
Wire Wire Line
	5500 5400 5500 5450
Connection ~ 5500 5400
$Comp
L Device:C C9
U 1 1 5CC54314
P 5500 5600
F 0 "C9" H 5615 5646 50  0000 L CNN
F 1 "18pF" H 5615 5555 50  0000 L CNN
F 2 "" H 5538 5450 50  0001 C CNN
F 3 "~" H 5500 5600 50  0001 C CNN
	1    5500 5600
	1    0    0    -1  
$EndComp
$Comp
L Device:C C7
U 1 1 5CC543E5
P 5100 5600
F 0 "C7" H 5215 5646 50  0000 L CNN
F 1 "18pF" H 5215 5555 50  0000 L CNN
F 2 "" H 5138 5450 50  0001 C CNN
F 3 "~" H 5100 5600 50  0001 C CNN
	1    5100 5600
	1    0    0    -1  
$EndComp
Wire Wire Line
	5500 5750 5500 5850
Wire Wire Line
	5500 5850 5100 5850
Wire Wire Line
	5100 5850 5100 5750
$Comp
L power:GND #PWR0124
U 1 1 5CC5F4C3
P 5100 5950
F 0 "#PWR0124" H 5100 5700 50  0001 C CNN
F 1 "GND" H 5105 5777 50  0000 C CNN
F 2 "" H 5100 5950 50  0001 C CNN
F 3 "" H 5100 5950 50  0001 C CNN
	1    5100 5950
	1    0    0    -1  
$EndComp
Wire Wire Line
	5100 5950 5100 5850
Connection ~ 5100 5850
Wire Wire Line
	7000 8600 7000 8900
$Comp
L power:GND #PWR0125
U 1 1 5CC65E4B
P 7000 8900
F 0 "#PWR0125" H 7000 8650 50  0001 C CNN
F 1 "GND" H 7005 8727 50  0000 C CNN
F 2 "" H 7000 8900 50  0001 C CNN
F 3 "" H 7000 8900 50  0001 C CNN
	1    7000 8900
	1    0    0    -1  
$EndComp
$Comp
L Device:C C11
U 1 1 5CC6FC06
P 5850 6150
F 0 "C11" H 5965 6196 50  0000 L CNN
F 1 "0.1uF" H 5965 6105 50  0000 L CNN
F 2 "" H 5888 6000 50  0001 C CNN
F 3 "~" H 5850 6150 50  0001 C CNN
	1    5850 6150
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0126
U 1 1 5CC72DC4
P 5850 6400
F 0 "#PWR0126" H 5850 6150 50  0001 C CNN
F 1 "GND" H 5855 6227 50  0000 C CNN
F 2 "" H 5850 6400 50  0001 C CNN
F 3 "" H 5850 6400 50  0001 C CNN
	1    5850 6400
	1    0    0    -1  
$EndComp
Wire Wire Line
	5850 6300 5850 6400
Wire Wire Line
	4800 4900 6100 4900
Wire Wire Line
	5800 5100 6400 5100
Wire Wire Line
	5800 5300 6400 5300
Wire Wire Line
	5850 5500 6400 5500
Wire Wire Line
	5850 5500 5850 6000
$Comp
L MCU_Microchip_ATmega:ATmega2561-16AU U7
U 1 1 5CBC2F1D
P 7000 6600
F 0 "U7" H 7000 4514 50  0000 C CNN
F 1 "ATmega2561-16AU" H 7000 4423 50  0000 C CNN
F 2 "Package_QFP:TQFP-64_14x14mm_P0.8mm" H 7000 6600 50  0001 C CIN
F 3 "http://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-2549-8-bit-AVR-Microcontroller-ATmega640-1280-1281-2560-2561_datasheet.pdf" H 7000 6600 50  0001 C CNN
	1    7000 6600
	1    0    0    -1  
$EndComp
$Comp
L hex_master_sym:Pushbutton_Stockroom U6
U 1 1 5CBD5BFF
P 4550 6050
F 0 "U6" H 4550 6875 50  0000 C CNN
F 1 "Pushbutton_Stockroom" H 4550 6784 50  0000 C CNN
F 2 "" H 4550 6250 50  0001 C CNN
F 3 "" H 4550 6250 50  0001 C CNN
	1    4550 6050
	1    0    0    -1  
$EndComp
Wire Wire Line
	4750 4900 4750 5450
Wire Wire Line
	4750 5750 4750 5850
NoConn ~ 4350 5450
NoConn ~ 4350 5750
$Comp
L power:GND #PWR0127
U 1 1 5CBEC62B
P 4750 5850
F 0 "#PWR0127" H 4750 5600 50  0001 C CNN
F 1 "GND" H 4755 5677 50  0000 C CNN
F 2 "" H 4750 5850 50  0001 C CNN
F 3 "" H 4750 5850 50  0001 C CNN
	1    4750 5850
	1    0    0    -1  
$EndComp
Text Label 6400 7600 2    50   ~ 0
Atmega_RXD0
Text Label 6400 7700 2    50   ~ 0
Atmega_TXD0
Wire Wire Line
	7000 3950 7000 4600
Wire Wire Line
	7100 3950 7100 4600
Wire Wire Line
	5100 5000 5100 5450
Wire Wire Line
	5100 5000 5500 5000
Text Notes 5450 4050 0    138  ~ 0
Atmega\n
Text Label 8450 7600 0    59   ~ 0
SCL
Text Label 8450 7700 0    59   ~ 0
SDA
$Comp
L Device:R R14
U 1 1 5CC2B6FF
P 8000 7400
F 0 "R14" H 8070 7446 50  0000 L CNN
F 1 "4.7k" H 8070 7355 50  0000 L CNN
F 2 "" V 7930 7400 50  0001 C CNN
F 3 "~" H 8000 7400 50  0001 C CNN
	1    8000 7400
	1    0    0    -1  
$EndComp
$Comp
L Device:R R16
U 1 1 5CC2B798
P 8350 7400
F 0 "R16" H 8420 7446 50  0000 L CNN
F 1 "4.7k" H 8420 7355 50  0000 L CNN
F 2 "" V 8280 7400 50  0001 C CNN
F 3 "~" H 8350 7400 50  0001 C CNN
	1    8350 7400
	1    0    0    -1  
$EndComp
Wire Wire Line
	8000 7550 8000 7600
Wire Wire Line
	8350 7550 8350 7700
Connection ~ 8350 7700
Wire Wire Line
	8350 7700 8450 7700
Wire Wire Line
	8000 7600 8450 7600
Connection ~ 8000 7600
$Comp
L power:+5V #PWR0128
U 1 1 5CC3C4A2
P 8350 7150
F 0 "#PWR0128" H 8350 7000 50  0001 C CNN
F 1 "+5V" H 8365 7323 50  0000 C CNN
F 2 "" H 8350 7150 50  0001 C CNN
F 3 "" H 8350 7150 50  0001 C CNN
	1    8350 7150
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR0129
U 1 1 5CC3C500
P 8000 7150
F 0 "#PWR0129" H 8000 7000 50  0001 C CNN
F 1 "+5V" H 8015 7323 50  0000 C CNN
F 2 "" H 8000 7150 50  0001 C CNN
F 3 "" H 8000 7150 50  0001 C CNN
	1    8000 7150
	1    0    0    -1  
$EndComp
Wire Wire Line
	8000 7250 8000 7150
Wire Wire Line
	8350 7250 8350 7150
Wire Wire Line
	7600 7600 8000 7600
Wire Wire Line
	7600 7700 8350 7700
$Comp
L Driver_LED:PCA9685PW U8
U 1 1 5CC0C68A
P 8500 2500
F 0 "U8" H 8500 3678 50  0000 C CNN
F 1 "PCA9685PW" H 8500 3587 50  0000 C CNN
F 2 "Package_SO:TSSOP-28_4.4x9.7mm_P0.65mm" H 8525 1525 50  0001 L CNN
F 3 "http://www.nxp.com/documents/data_sheet/PCA9685.pdf" H 8100 3200 50  0001 C CNN
	1    8500 2500
	1    0    0    -1  
$EndComp
$Comp
L Driver_LED:PCA9685PW U9
U 1 1 5CC0C70E
P 11100 2500
F 0 "U9" H 11100 3678 50  0000 C CNN
F 1 "PCA9685PW" H 11100 3587 50  0000 C CNN
F 2 "Package_SO:TSSOP-28_4.4x9.7mm_P0.65mm" H 11125 1525 50  0001 L CNN
F 3 "http://www.nxp.com/documents/data_sheet/PCA9685.pdf" H 10700 3200 50  0001 C CNN
	1    11100 2500
	1    0    0    -1  
$EndComp
Text Notes 9350 1000 0    138  ~ 0
PWM Drivers
Text Label 9350 1800 0    50   ~ 0
SSIG_0
Text Label 9350 1900 0    50   ~ 0
SSIG_1
Text Label 9350 2000 0    50   ~ 0
SSIG_2
Text Label 9350 2100 0    50   ~ 0
SSIG_3
Text Label 9350 2200 0    50   ~ 0
SSIG_4
Text Label 9350 2300 0    50   ~ 0
SSIG_5
Text Label 9350 2400 0    50   ~ 0
SSIG_6
Text Label 9350 2500 0    50   ~ 0
SSIG_7
Text Label 9350 2600 0    50   ~ 0
SSIG_8
Text Label 9350 2700 0    50   ~ 0
SSIG_9
Text Label 9350 2800 0    50   ~ 0
SSIG_10
Text Label 9350 2900 0    50   ~ 0
SSIG_11
Text Label 9350 3000 0    50   ~ 0
SSIG_12
Text Label 9350 3100 0    50   ~ 0
SSIG_13
Text Label 9350 3200 0    50   ~ 0
SSIG_14
Text Label 9350 3300 0    50   ~ 0
SSIG_15
Text Label 11950 1800 0    50   ~ 0
SSIG_16
Text Label 11950 1900 0    50   ~ 0
SSIG_17
Text Label 11950 2000 0    50   ~ 0
SSIG_18
Text Label 11950 2100 0    50   ~ 0
SSIG_19
Text Label 11950 2200 0    50   ~ 0
SSIG_20
Text Label 11950 2300 0    50   ~ 0
SSIG_21
Text Label 11950 2400 0    50   ~ 0
SSIG_22
Text Label 11950 2500 0    50   ~ 0
SSIG_23
Text Label 11950 2600 0    50   ~ 0
SSIG_24
Text Label 11950 2700 0    50   ~ 0
SSIG_25
Text Label 11950 2800 0    50   ~ 0
SSIG_26
Text Label 11950 2900 0    50   ~ 0
SSIG_27
Text Label 11950 3000 0    50   ~ 0
SSIG_28
Text Label 11950 3100 0    50   ~ 0
SSIG_29
Text Label 11950 3200 0    50   ~ 0
SSIG_30
Text Label 11950 3300 0    50   ~ 0
SSIG_31
Text Label 10250 1800 2    50   ~ 0
SCL
Text Label 10250 1900 2    50   ~ 0
SDA
Text Label 7650 1800 2    50   ~ 0
SCL
Text Label 7650 1900 2    50   ~ 0
SDA
Wire Wire Line
	7800 2100 7700 2100
Wire Wire Line
	7700 2100 7700 2200
$Comp
L power:GND #PWR0130
U 1 1 5CC36169
P 7700 2200
F 0 "#PWR0130" H 7700 1950 50  0001 C CNN
F 1 "GND" H 7705 2027 50  0000 C CNN
F 2 "" H 7700 2200 50  0001 C CNN
F 3 "" H 7700 2200 50  0001 C CNN
	1    7700 2200
	1    0    0    -1  
$EndComp
Wire Wire Line
	8500 1050 8850 1050
Wire Wire Line
	8850 1050 8850 1100
$Comp
L Device:C C14
U 1 1 5CC45A2E
P 8850 1250
F 0 "C14" H 8965 1296 50  0000 L CNN
F 1 "10uF" H 8965 1205 50  0000 L CNN
F 2 "" H 8888 1100 50  0001 C CNN
F 3 "~" H 8850 1250 50  0001 C CNN
	1    8850 1250
	1    0    0    -1  
$EndComp
Wire Wire Line
	7800 2000 7700 2000
Wire Wire Line
	7700 2000 7700 2100
Connection ~ 7700 2100
$Comp
L power:GND #PWR0131
U 1 1 5CC5CC17
P 8850 1450
F 0 "#PWR0131" H 8850 1200 50  0001 C CNN
F 1 "GND" H 8855 1277 50  0000 C CNN
F 2 "" H 8850 1450 50  0001 C CNN
F 3 "" H 8850 1450 50  0001 C CNN
	1    8850 1450
	1    0    0    -1  
$EndComp
Wire Wire Line
	8500 1050 8500 1500
Wire Wire Line
	8850 1450 8850 1400
$Comp
L power:+5V #PWR0132
U 1 1 5CC647D0
P 8500 950
F 0 "#PWR0132" H 8500 800 50  0001 C CNN
F 1 "+5V" H 8515 1123 50  0000 C CNN
F 2 "" H 8500 950 50  0001 C CNN
F 3 "" H 8500 950 50  0001 C CNN
	1    8500 950 
	1    0    0    -1  
$EndComp
Wire Wire Line
	8500 950  8500 1050
Connection ~ 8500 1050
Wire Wire Line
	11100 1050 11450 1050
Wire Wire Line
	11450 1050 11450 1100
$Comp
L Device:C C15
U 1 1 5CC784E8
P 11450 1250
F 0 "C15" H 11565 1296 50  0000 L CNN
F 1 "10uF" H 11565 1205 50  0000 L CNN
F 2 "" H 11488 1100 50  0001 C CNN
F 3 "~" H 11450 1250 50  0001 C CNN
	1    11450 1250
	1    0    0    -1  
$EndComp
Wire Wire Line
	11100 1050 11100 1500
$Comp
L power:+5V #PWR0133
U 1 1 5CC784F0
P 11100 950
F 0 "#PWR0133" H 11100 800 50  0001 C CNN
F 1 "+5V" H 11115 1123 50  0000 C CNN
F 2 "" H 11100 950 50  0001 C CNN
F 3 "" H 11100 950 50  0001 C CNN
	1    11100 950 
	1    0    0    -1  
$EndComp
Wire Wire Line
	11100 950  11100 1050
Connection ~ 11100 1050
$Comp
L power:GND #PWR0134
U 1 1 5CC7C796
P 11450 1450
F 0 "#PWR0134" H 11450 1200 50  0001 C CNN
F 1 "GND" H 11455 1277 50  0000 C CNN
F 2 "" H 11450 1450 50  0001 C CNN
F 3 "" H 11450 1450 50  0001 C CNN
	1    11450 1450
	1    0    0    -1  
$EndComp
Wire Wire Line
	11450 1450 11450 1400
$Comp
L power:GND #PWR0135
U 1 1 5CC80F2A
P 8500 3650
F 0 "#PWR0135" H 8500 3400 50  0001 C CNN
F 1 "GND" H 8505 3477 50  0000 C CNN
F 2 "" H 8500 3650 50  0001 C CNN
F 3 "" H 8500 3650 50  0001 C CNN
	1    8500 3650
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0136
U 1 1 5CC80F90
P 11100 3650
F 0 "#PWR0136" H 11100 3400 50  0001 C CNN
F 1 "GND" H 11105 3477 50  0000 C CNN
F 2 "" H 11100 3650 50  0001 C CNN
F 3 "" H 11100 3650 50  0001 C CNN
	1    11100 3650
	1    0    0    -1  
$EndComp
Wire Wire Line
	8500 3650 8500 3600
Wire Wire Line
	11100 3650 11100 3600
Wire Wire Line
	11800 1800 11950 1800
Wire Wire Line
	11800 1900 11950 1900
Wire Wire Line
	11800 2000 11950 2000
Wire Wire Line
	11800 2100 11950 2100
Wire Wire Line
	11800 2200 11950 2200
Wire Wire Line
	11800 2300 11950 2300
Wire Wire Line
	11800 2400 11950 2400
Wire Wire Line
	11800 2500 11950 2500
Wire Wire Line
	11800 2600 11950 2600
Wire Wire Line
	11950 2700 11800 2700
Wire Wire Line
	11800 2800 11950 2800
Wire Wire Line
	11950 2900 11800 2900
Wire Wire Line
	11800 3000 11950 3000
Wire Wire Line
	11950 3100 11800 3100
Wire Wire Line
	11800 3200 11950 3200
Wire Wire Line
	11950 3300 11800 3300
Wire Wire Line
	9200 1800 9350 1800
Wire Wire Line
	9350 1900 9200 1900
Wire Wire Line
	9200 2000 9350 2000
Wire Wire Line
	9350 2100 9200 2100
Wire Wire Line
	9200 2200 9350 2200
Wire Wire Line
	9350 2300 9200 2300
Wire Wire Line
	9200 2400 9350 2400
Wire Wire Line
	9350 2500 9200 2500
Wire Wire Line
	9200 2600 9350 2600
Wire Wire Line
	9350 2700 9200 2700
Wire Wire Line
	9350 2900 9200 2900
Wire Wire Line
	9200 2800 9350 2800
Wire Wire Line
	9200 3000 9350 3000
Wire Wire Line
	9350 3100 9200 3100
Wire Wire Line
	9200 3200 9350 3200
Wire Wire Line
	9350 3300 9200 3300
Wire Wire Line
	7650 1800 7800 1800
Wire Wire Line
	7800 1900 7650 1900
Wire Wire Line
	10250 1800 10400 1800
Wire Wire Line
	10400 1900 10250 1900
$Comp
L power:GND #PWR0137
U 1 1 5CD513F6
P 10300 2200
F 0 "#PWR0137" H 10300 1950 50  0001 C CNN
F 1 "GND" H 10305 2027 50  0000 C CNN
F 2 "" H 10300 2200 50  0001 C CNN
F 3 "" H 10300 2200 50  0001 C CNN
	1    10300 2200
	1    0    0    -1  
$EndComp
Wire Wire Line
	10400 2100 10300 2100
Wire Wire Line
	10300 2100 10300 2200
Wire Wire Line
	10400 2000 10300 2000
Wire Wire Line
	10300 2000 10300 2100
Connection ~ 10300 2100
Wire Wire Line
	7800 2700 7700 2700
Wire Wire Line
	7700 2700 7700 2800
Wire Wire Line
	7800 2800 7700 2800
Connection ~ 7700 2800
Wire Wire Line
	7700 2800 7700 2900
Wire Wire Line
	7800 2900 7700 2900
Connection ~ 7700 2900
Wire Wire Line
	7700 2900 7700 3000
Wire Wire Line
	7800 3000 7700 3000
Connection ~ 7700 3000
Wire Wire Line
	7700 3000 7700 3100
Wire Wire Line
	7800 3100 7700 3100
Connection ~ 7700 3100
Wire Wire Line
	7700 3100 7700 3200
Wire Wire Line
	7800 3200 7700 3200
Connection ~ 7700 3200
Wire Wire Line
	7700 3200 7700 3300
$Comp
L power:GND #PWR0138
U 1 1 5CD9F633
P 7700 3300
F 0 "#PWR0138" H 7700 3050 50  0001 C CNN
F 1 "GND" H 7705 3127 50  0000 C CNN
F 2 "" H 7700 3300 50  0001 C CNN
F 3 "" H 7700 3300 50  0001 C CNN
	1    7700 3300
	1    0    0    -1  
$EndComp
Wire Wire Line
	10400 2800 10300 2800
Wire Wire Line
	10300 2800 10300 2900
Wire Wire Line
	10400 3200 10300 3200
Connection ~ 10300 3200
Wire Wire Line
	10300 3200 10300 3300
Wire Wire Line
	10400 3100 10300 3100
Connection ~ 10300 3100
Wire Wire Line
	10300 3100 10300 3200
Wire Wire Line
	10400 3000 10300 3000
Connection ~ 10300 3000
Wire Wire Line
	10300 3000 10300 3100
Wire Wire Line
	10400 2900 10300 2900
Connection ~ 10300 2900
Wire Wire Line
	10300 2900 10300 3000
Wire Wire Line
	10400 2700 10050 2700
$Comp
L power:+5V #PWR0139
U 1 1 5CDD0E87
P 10050 2550
F 0 "#PWR0139" H 10050 2400 50  0001 C CNN
F 1 "+5V" H 10065 2723 50  0000 C CNN
F 2 "" H 10050 2550 50  0001 C CNN
F 3 "" H 10050 2550 50  0001 C CNN
	1    10050 2550
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0140
U 1 1 5CDD1034
P 10300 3300
F 0 "#PWR0140" H 10300 3050 50  0001 C CNN
F 1 "GND" H 10305 3127 50  0000 C CNN
F 2 "" H 10300 3300 50  0001 C CNN
F 3 "" H 10300 3300 50  0001 C CNN
	1    10300 3300
	1    0    0    -1  
$EndComp
Text Notes 9600 1200 0    50   ~ 0
Don't forget to add\nresistors to outputs
Wire Wire Line
	4150 4900 4350 4900
$Comp
L Device:LED D2
U 1 1 5CC864DC
P 2000 6300
F 0 "D2" V 2038 6183 50  0000 R CNN
F 1 "LED" V 1947 6183 50  0000 R CNN
F 2 "" H 2000 6300 50  0001 C CNN
F 3 "~" H 2000 6300 50  0001 C CNN
	1    2000 6300
	0    -1   -1   0   
$EndComp
$Comp
L Device:LED D1
U 1 1 5CC866C6
P 1650 6300
F 0 "D1" V 1688 6183 50  0000 R CNN
F 1 "LED" V 1597 6183 50  0000 R CNN
F 2 "" H 1650 6300 50  0001 C CNN
F 3 "~" H 1650 6300 50  0001 C CNN
	1    1650 6300
	0    -1   -1   0   
$EndComp
Text Label 4150 5500 0    50   ~ 0
~RX
Text Label 4150 5600 0    50   ~ 0
~TX
Text Label 2000 6950 3    50   ~ 0
~RX
Text Label 1650 6950 3    50   ~ 0
~TX
$Comp
L Device:R R1
U 1 1 5CCB425A
P 1650 6700
F 0 "R1" H 1720 6746 50  0000 L CNN
F 1 "1k" H 1720 6655 50  0000 L CNN
F 2 "" V 1580 6700 50  0001 C CNN
F 3 "~" H 1650 6700 50  0001 C CNN
	1    1650 6700
	1    0    0    -1  
$EndComp
$Comp
L Device:R R6
U 1 1 5CCB42D4
P 2000 6700
F 0 "R6" H 2070 6746 50  0000 L CNN
F 1 "1k" H 2070 6655 50  0000 L CNN
F 2 "" V 1930 6700 50  0001 C CNN
F 3 "~" H 2000 6700 50  0001 C CNN
	1    2000 6700
	1    0    0    -1  
$EndComp
Wire Wire Line
	1650 6550 1650 6450
Wire Wire Line
	2000 6550 2000 6450
Wire Wire Line
	2000 6950 2000 6850
Wire Wire Line
	1650 6950 1650 6850
Wire Wire Line
	2000 6150 2000 6050
Wire Wire Line
	1650 6150 1650 6050
$Comp
L power:+5V #PWR0141
U 1 1 5CCECA7A
P 1650 6050
F 0 "#PWR0141" H 1650 5900 50  0001 C CNN
F 1 "+5V" H 1665 6223 50  0000 C CNN
F 2 "" H 1650 6050 50  0001 C CNN
F 3 "" H 1650 6050 50  0001 C CNN
	1    1650 6050
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR0142
U 1 1 5CCECAF6
P 2000 6050
F 0 "#PWR0142" H 2000 5900 50  0001 C CNN
F 1 "+5V" H 2015 6223 50  0000 C CNN
F 2 "" H 2000 6050 50  0001 C CNN
F 3 "" H 2000 6050 50  0001 C CNN
	1    2000 6050
	1    0    0    -1  
$EndComp
NoConn ~ 4150 5200
Text Label 7600 5900 0    50   ~ 0
SCK
Text Label 7600 6100 0    50   ~ 0
MISO
Text Label 7600 6000 0    50   ~ 0
MOSI
Wire Wire Line
	6100 4900 6100 4750
Connection ~ 6100 4900
Wire Wire Line
	6100 4900 6400 4900
Text Label 6100 4750 0    50   ~ 0
~RESET
Wire Wire Line
	10050 2700 10050 2550
Text Label 7600 4900 0    50   ~ 0
D0
Text Label 7600 5000 0    50   ~ 0
D1
Text Label 7600 5100 0    50   ~ 0
D2
Text Label 7600 5200 0    50   ~ 0
D3
Text Label 7600 5300 0    50   ~ 0
D4
Text Label 7600 5400 0    50   ~ 0
D5
Text Label 7600 5500 0    50   ~ 0
D6
Text Label 7600 5600 0    50   ~ 0
D7
$Comp
L Device:LED D4
U 1 1 5CD48B1B
P 8050 4450
F 0 "D4" V 8088 4333 50  0000 R CNN
F 1 "LED" V 7997 4333 50  0000 R CNN
F 2 "" H 8050 4450 50  0001 C CNN
F 3 "~" H 8050 4450 50  0001 C CNN
	1    8050 4450
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R15
U 1 1 5CD48B23
P 8050 4850
F 0 "R15" H 8120 4896 50  0000 L CNN
F 1 "1k" H 8120 4805 50  0000 L CNN
F 2 "" V 7980 4850 50  0001 C CNN
F 3 "~" H 8050 4850 50  0001 C CNN
	1    8050 4850
	1    0    0    -1  
$EndComp
Wire Wire Line
	8050 4700 8050 4600
Wire Wire Line
	8050 5100 8050 5000
Text Label 8050 4100 0    50   ~ 0
D0
Wire Wire Line
	8050 4100 8050 4300
$Comp
L power:GND #PWR0143
U 1 1 5CD87034
P 8050 5100
F 0 "#PWR0143" H 8050 4850 50  0001 C CNN
F 1 "GND" H 8055 4927 50  0000 C CNN
F 2 "" H 8050 5100 50  0001 C CNN
F 3 "" H 8050 5100 50  0001 C CNN
	1    8050 5100
	1    0    0    -1  
$EndComp
Text Notes 8350 4700 0    138  ~ 0
LED\n
Text Notes 13550 1150 0    138  ~ 0
Switch Inputs
Text Notes 13750 1250 0    50   ~ 0
Internal pull-ups, D0-D5\n
$Comp
L Device:R R17
U 1 1 5CC6BFF4
P 13100 1800
F 0 "R17" H 13170 1846 50  0000 L CNN
F 1 "10k" H 13170 1755 50  0000 L CNN
F 2 "" V 13030 1800 50  0001 C CNN
F 3 "~" H 13100 1800 50  0001 C CNN
	1    13100 1800
	1    0    0    -1  
$EndComp
$Comp
L Device:R R18
U 1 1 5CC6C085
P 13400 1800
F 0 "R18" H 13470 1846 50  0000 L CNN
F 1 "10k" H 13470 1755 50  0000 L CNN
F 2 "" V 13330 1800 50  0001 C CNN
F 3 "~" H 13400 1800 50  0001 C CNN
	1    13400 1800
	1    0    0    -1  
$EndComp
$Comp
L Device:R R19
U 1 1 5CC6C118
P 13700 1800
F 0 "R19" H 13770 1846 50  0000 L CNN
F 1 "10k" H 13770 1755 50  0000 L CNN
F 2 "" V 13630 1800 50  0001 C CNN
F 3 "~" H 13700 1800 50  0001 C CNN
	1    13700 1800
	1    0    0    -1  
$EndComp
$Comp
L Device:R R20
U 1 1 5CC6C1AD
P 14000 1800
F 0 "R20" H 14070 1846 50  0000 L CNN
F 1 "10k" H 14070 1755 50  0000 L CNN
F 2 "" V 13930 1800 50  0001 C CNN
F 3 "~" H 14000 1800 50  0001 C CNN
	1    14000 1800
	1    0    0    -1  
$EndComp
$Comp
L Device:R R21
U 1 1 5CC6C238
P 14300 1800
F 0 "R21" H 14370 1846 50  0000 L CNN
F 1 "10k" H 14370 1755 50  0000 L CNN
F 2 "" V 14230 1800 50  0001 C CNN
F 3 "~" H 14300 1800 50  0001 C CNN
	1    14300 1800
	1    0    0    -1  
$EndComp
$Comp
L Device:R R22
U 1 1 5CC6C2C5
P 14600 1800
F 0 "R22" H 14670 1846 50  0000 L CNN
F 1 "10k" H 14670 1755 50  0000 L CNN
F 2 "" V 14530 1800 50  0001 C CNN
F 3 "~" H 14600 1800 50  0001 C CNN
	1    14600 1800
	1    0    0    -1  
$EndComp
Wire Wire Line
	13100 1950 13100 2100
Wire Wire Line
	13400 1950 13400 2100
Wire Wire Line
	13700 1950 13700 2100
Wire Wire Line
	14000 1950 14000 2100
Wire Wire Line
	14300 1950 14300 2100
Wire Wire Line
	14600 1950 14600 2100
$Comp
L power:+5V #PWR0144
U 1 1 5CCE2C04
P 13100 1450
F 0 "#PWR0144" H 13100 1300 50  0001 C CNN
F 1 "+5V" H 13115 1623 50  0000 C CNN
F 2 "" H 13100 1450 50  0001 C CNN
F 3 "" H 13100 1450 50  0001 C CNN
	1    13100 1450
	1    0    0    -1  
$EndComp
Wire Wire Line
	13100 1450 13100 1550
Wire Wire Line
	13100 1550 13400 1550
Wire Wire Line
	14600 1550 14600 1650
Connection ~ 13100 1550
Wire Wire Line
	13100 1550 13100 1650
Wire Wire Line
	14300 1650 14300 1550
Connection ~ 14300 1550
Wire Wire Line
	14300 1550 14600 1550
Wire Wire Line
	14000 1650 14000 1550
Connection ~ 14000 1550
Wire Wire Line
	14000 1550 14300 1550
Wire Wire Line
	13700 1650 13700 1550
Connection ~ 13700 1550
Wire Wire Line
	13700 1550 14000 1550
Wire Wire Line
	13400 1650 13400 1550
Connection ~ 13400 1550
Wire Wire Line
	13400 1550 13700 1550
Text Label 13100 2100 0    50   ~ 0
D0
Text Label 13400 2100 0    50   ~ 0
D1
Text Label 13700 2100 0    50   ~ 0
D2
Text Label 14000 2100 0    50   ~ 0
D3
Text Label 14300 2100 0    50   ~ 0
D4
Text Label 14600 2100 0    50   ~ 0
D5
Text Label 6400 6700 2    50   ~ 0
A0
Text Label 6400 6800 2    50   ~ 0
A1
Text Label 6400 6900 2    50   ~ 0
A2
Text Label 6400 7000 2    50   ~ 0
A3
Text Label 6400 7100 2    50   ~ 0
A4
Text Label 6400 7200 2    50   ~ 0
A5
Text Label 6400 7300 2    50   ~ 0
A6
Text Label 6400 7400 2    50   ~ 0
A7
$Comp
L hex_master_sym:74LVC1G157GV_MUX2 U2
U 1 1 5CCD9426
P 2550 9500
F 0 "U2" H 2891 9921 50  0000 L CNN
F 1 "74LVC1G157GV_MUX2" H 2891 9830 50  0000 L CNN
F 2 "hex_master_board:74LVC1G157_MUX2_TSOP6" H 2550 9250 50  0001 C CNN
F 3 "" H 2550 9250 50  0001 C CNN
	1    2550 9500
	1    0    0    -1  
$EndComp
$Comp
L Device:R R7
U 1 1 5CCE6791
P 2150 9550
F 0 "R7" H 2220 9596 50  0000 L CNN
F 1 "10k" H 2220 9505 50  0000 L CNN
F 2 "" V 2080 9550 50  0001 C CNN
F 3 "~" H 2150 9550 50  0001 C CNN
	1    2150 9550
	1    0    0    -1  
$EndComp
Text Label 1350 8950 2    50   ~ 0
USB_TXD
Text Label 1350 9050 2    50   ~ 0
XBEE_TXD
Text Label 2000 9300 2    50   ~ 0
USB_V+
Wire Wire Line
	2000 9300 2150 9300
Wire Wire Line
	2150 9300 2150 9400
Connection ~ 2150 9300
Wire Wire Line
	2150 9300 2250 9300
Wire Wire Line
	2150 9700 2150 9800
$Comp
L power:GND #PWR0145
U 1 1 5CD46434
P 2150 9800
F 0 "#PWR0145" H 2150 9550 50  0001 C CNN
F 1 "GND" H 2155 9627 50  0000 C CNN
F 2 "" H 2150 9800 50  0001 C CNN
F 3 "" H 2150 9800 50  0001 C CNN
	1    2150 9800
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0146
U 1 1 5CD464CB
P 2550 9800
F 0 "#PWR0146" H 2550 9550 50  0001 C CNN
F 1 "GND" H 2555 9627 50  0000 C CNN
F 2 "" H 2550 9800 50  0001 C CNN
F 3 "" H 2550 9800 50  0001 C CNN
	1    2550 9800
	1    0    0    -1  
$EndComp
Wire Wire Line
	2550 9500 2550 9800
$Comp
L power:+5V #PWR0147
U 1 1 5CD5FD74
P 2550 8300
F 0 "#PWR0147" H 2550 8150 50  0001 C CNN
F 1 "+5V" H 2565 8473 50  0000 C CNN
F 2 "" H 2550 8300 50  0001 C CNN
F 3 "" H 2550 8300 50  0001 C CNN
	1    2550 8300
	1    0    0    -1  
$EndComp
$Comp
L Device:C C3
U 1 1 5CD5FE20
P 2950 8600
F 0 "C3" H 3065 8646 50  0000 L CNN
F 1 "0.1uF" H 3065 8555 50  0000 L CNN
F 2 "" H 2988 8450 50  0001 C CNN
F 3 "~" H 2950 8600 50  0001 C CNN
	1    2950 8600
	1    0    0    -1  
$EndComp
Wire Wire Line
	2950 8400 2950 8450
$Comp
L power:GND #PWR0148
U 1 1 5CD6CF27
P 2950 8750
F 0 "#PWR0148" H 2950 8500 50  0001 C CNN
F 1 "GND" H 2955 8577 50  0000 C CNN
F 2 "" H 2950 8750 50  0001 C CNN
F 3 "" H 2950 8750 50  0001 C CNN
	1    2950 8750
	1    0    0    -1  
$EndComp
Wire Wire Line
	2950 8400 2550 8400
Wire Wire Line
	2550 8400 2550 8750
Wire Wire Line
	2550 8400 2550 8300
Connection ~ 2550 8400
Text Label 3600 9100 0    50   ~ 0
Atmega_RXD0
Text Label 1350 10400 2    50   ~ 0
XBEE_RXD
Text Label 4200 4500 0    50   ~ 0
USB_TXD
Text Label 4200 4600 0    50   ~ 0
USB_RXD
Wire Wire Line
	4200 4500 4150 4500
Wire Wire Line
	4150 4600 4200 4600
$Comp
L Device:R R4
U 1 1 5CE63720
P 1950 8900
F 0 "R4" V 2157 8900 50  0000 C CNN
F 1 "1k" V 2066 8900 50  0000 C CNN
F 2 "" V 1880 8900 50  0001 C CNN
F 3 "~" H 1950 8900 50  0001 C CNN
	1    1950 8900
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R5
U 1 1 5CE63917
P 1950 9100
F 0 "R5" V 1743 9100 50  0000 C CNN
F 1 "1k" V 1834 9100 50  0000 C CNN
F 2 "" V 1880 9100 50  0001 C CNN
F 3 "~" H 1950 9100 50  0001 C CNN
	1    1950 9100
	0    1    1    0   
$EndComp
Wire Wire Line
	2250 8950 2150 8950
Wire Wire Line
	2150 8950 2150 8900
Wire Wire Line
	2150 8900 2100 8900
Wire Wire Line
	2250 9050 2150 9050
Wire Wire Line
	2150 9050 2150 9100
Wire Wire Line
	2150 9100 2100 9100
Wire Wire Line
	1350 8950 1750 8950
Wire Wire Line
	1750 8950 1750 8900
Wire Wire Line
	1750 8900 1800 8900
Wire Wire Line
	1350 9050 1750 9050
Wire Wire Line
	1750 9050 1750 9100
Wire Wire Line
	1750 9100 1800 9100
$Comp
L Device:R R8
U 1 1 5CEC0693
P 3100 9100
F 0 "R8" V 2893 9100 50  0000 C CNN
F 1 "1k" V 2984 9100 50  0000 C CNN
F 2 "" V 3030 9100 50  0001 C CNN
F 3 "~" H 3100 9100 50  0001 C CNN
	1    3100 9100
	0    1    1    0   
$EndComp
Wire Wire Line
	2950 9100 2850 9100
Wire Wire Line
	3250 9100 3600 9100
$Comp
L Device:R R2
U 1 1 5CEEB8C2
P 1750 10400
F 0 "R2" V 1543 10400 50  0000 C CNN
F 1 "1k" V 1634 10400 50  0000 C CNN
F 2 "" V 1680 10400 50  0001 C CNN
F 3 "~" H 1750 10400 50  0001 C CNN
	1    1750 10400
	0    1    1    0   
$EndComp
$Comp
L Device:R R3
U 1 1 5CEEB9C2
P 1750 10700
F 0 "R3" V 1543 10700 50  0000 C CNN
F 1 "1k" V 1634 10700 50  0000 C CNN
F 2 "" V 1680 10700 50  0001 C CNN
F 3 "~" H 1750 10700 50  0001 C CNN
	1    1750 10700
	0    1    1    0   
$EndComp
Text Label 1350 10700 2    50   ~ 0
USB_RXD
Wire Wire Line
	1350 10400 1600 10400
Wire Wire Line
	1350 10700 1600 10700
Wire Wire Line
	1900 10700 2050 10700
Wire Wire Line
	2050 10700 2050 10400
Wire Wire Line
	2050 10400 1900 10400
Text Label 2300 10400 0    50   ~ 0
Atmega_TXD0
Wire Wire Line
	2050 10400 2300 10400
Connection ~ 2050 10400
Text Notes 900  8000 0    138  ~ 0
Serial Multiplexing\n\n
Text Notes 1250 8050 0    50   ~ 0
Make sure using power as a \nlogic level isn't sketchy
$EndSCHEMATC
