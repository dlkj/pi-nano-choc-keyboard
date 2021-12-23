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
L MCU_RaspberryPi_and_Boards:Pico U1
U 1 1 61C780DB
P 9000 3850
F 0 "U1" H 9000 5065 50  0000 C CNN
F 1 "Pico" H 9000 4974 50  0000 C CNN
F 2 "footprints:RPi_Pico_SMD_TH_flipable" V 9000 3850 50  0001 C CNN
F 3 "" H 9000 3850 50  0001 C CNN
	1    9000 3850
	1    0    0    -1  
$EndComp
NoConn ~ 8900 5000
NoConn ~ 9100 5000
Text GLabel 9700 4400 2    50   Input ~ 0
UART_RX
$Comp
L power:GND #PWR013
U 1 1 61C7DD14
P 9000 5000
F 0 "#PWR013" H 9000 4750 50  0001 C CNN
F 1 "GND" V 9000 4850 50  0000 R CNN
F 2 "" H 9000 5000 50  0001 C CNN
F 3 "" H 9000 5000 50  0001 C CNN
	1    9000 5000
	1    0    0    -1  
$EndComp
Wire Wire Line
	9700 2900 9800 2900
Wire Wire Line
	9800 2900 9800 2950
Wire Wire Line
	9800 3000 9700 3000
$Comp
L power:VBUS #PWR019
U 1 1 61C7F092
P 9800 2950
F 0 "#PWR019" H 9800 2800 50  0001 C CNN
F 1 "VBUS" V 9815 3078 50  0000 L CNN
F 2 "" H 9800 2950 50  0001 C CNN
F 3 "" H 9800 2950 50  0001 C CNN
	1    9800 2950
	0    1    1    0   
$EndComp
Connection ~ 9800 2950
Wire Wire Line
	9800 2950 9800 3000
$Comp
L power:+3V3 #PWR015
U 1 1 61C7F8C6
P 9700 3300
F 0 "#PWR015" H 9700 3150 50  0001 C CNN
F 1 "+3V3" V 9715 3428 50  0000 L CNN
F 2 "" H 9700 3300 50  0001 C CNN
F 3 "" H 9700 3300 50  0001 C CNN
	1    9700 3300
	0    1    1    0   
$EndComp
Text GLabel 8300 4700 0    50   Input ~ 0
I2C_SCL_SDA
Text GLabel 8300 4800 0    50   Input ~ 0
I2C_SDA_SCL
Text GLabel 8300 4300 0    50   Input ~ 0
ROW_0
Text GLabel 8300 4000 0    50   Input ~ 0
ROW_2
Text GLabel 8300 4200 0    50   Input ~ 0
ROW_1
Text GLabel 8300 3800 0    50   Input ~ 0
ROW_3
Text GLabel 8300 3700 0    50   Input ~ 0
ROW_4
Text GLabel 8300 3500 0    50   Input ~ 0
ROW_5
Text GLabel 9700 4300 2    50   Input ~ 0
COL_0
Text GLabel 9700 4200 2    50   Input ~ 0
COL_1
Text GLabel 9700 4000 2    50   Input ~ 0
COL_2
Text GLabel 9700 3800 2    50   Input ~ 0
COL_3
Text GLabel 9700 3700 2    50   Input ~ 0
COL_4
Text GLabel 9700 3500 2    50   Input ~ 0
COL_5
$Comp
L Switch:SW_Push SW33
U 1 1 61C871B6
P 10500 2150
F 0 "SW33" H 10500 2435 50  0000 C CNN
F 1 "SW_Push" H 10500 2344 50  0000 C CNN
F 2 "Button_Switch_THT:SW_PUSH_6mm" H 10500 2350 50  0001 C CNN
F 3 "~" H 10500 2350 50  0001 C CNN
	1    10500 2150
	1    0    0    -1  
$EndComp
Text GLabel 10700 2150 2    50   Input ~ 0
Reset
Text GLabel 1100 5300 3    50   Input ~ 0
COL_0
Text GLabel 1800 5300 3    50   Input ~ 0
COL_1
Text GLabel 2500 5300 3    50   Input ~ 0
COL_2
Text GLabel 3200 5300 3    50   Input ~ 0
COL_3
Text GLabel 3900 5300 3    50   Input ~ 0
COL_4
Text GLabel 4600 5300 3    50   Input ~ 0
COL_5
$Comp
L Switch:SW_Push SW11
U 1 1 61CA432E
P 2100 4800
F 0 "SW11" V 2054 4948 50  0000 L CNN
F 1 "SW_Push" V 2145 4948 50  0000 L CNN
F 2 "keyswitches:SW_PG1350_reversible" H 2100 5000 50  0001 C CNN
F 3 "~" H 2100 5000 50  0001 C CNN
	1    2100 4800
	0    1    -1   0   
$EndComp
Text GLabel 5400 4600 2    50   Input ~ 0
ROW_0
$Comp
L Device:D_ALT D11
U 1 1 61CA4353
P 1950 5000
F 0 "D11" H 1950 5217 50  0000 C CNN
F 1 "D_ALT" H 1950 5126 50  0000 C CNN
F 2 "Diode_THT:D_DO-35_SOD27_P7.62mm_Horizontal" H 1950 5000 50  0001 C CNN
F 3 "~" H 1950 5000 50  0001 C CNN
	1    1950 5000
	1    0    0    1   
$EndComp
$Comp
L Device:D_ALT D23
U 1 1 61CA4370
P 3350 5000
F 0 "D23" H 3350 5217 50  0000 C CNN
F 1 "D_ALT" H 3350 5126 50  0000 C CNN
F 2 "Diode_THT:D_DO-35_SOD27_P7.62mm_Horizontal" H 3350 5000 50  0001 C CNN
F 3 "~" H 3350 5000 50  0001 C CNN
	1    3350 5000
	1    0    0    1   
$EndComp
$Comp
L Device:D_ALT D29
U 1 1 61CA4376
P 4050 5000
F 0 "D29" H 4050 5217 50  0000 C CNN
F 1 "D_ALT" H 4050 5126 50  0000 C CNN
F 2 "Diode_THT:D_DO-35_SOD27_P7.62mm_Horizontal" H 4050 5000 50  0001 C CNN
F 3 "~" H 4050 5000 50  0001 C CNN
	1    4050 5000
	1    0    0    1   
$EndComp
$Comp
L Device:D_ALT D35
U 1 1 61CA437C
P 4750 5000
F 0 "D35" H 4750 5217 50  0000 C CNN
F 1 "D_ALT" H 4750 5126 50  0000 C CNN
F 2 "Diode_THT:D_DO-35_SOD27_P7.62mm_Horizontal" H 4750 5000 50  0001 C CNN
F 3 "~" H 4750 5000 50  0001 C CNN
	1    4750 5000
	1    0    0    1   
$EndComp
Wire Wire Line
	3200 5300 3200 5000
Wire Wire Line
	3900 5300 3900 5000
Wire Wire Line
	4600 5300 4600 5000
$Comp
L Switch:SW_Push SW5
U 1 1 61CB00F0
P 1400 4150
F 0 "SW5" V 1354 4298 50  0000 L CNN
F 1 "SW_Push" V 1445 4298 50  0000 L CNN
F 2 "keyswitches:SW_PG1350_reversible" H 1400 4350 50  0001 C CNN
F 3 "~" H 1400 4350 50  0001 C CNN
	1    1400 4150
	0    1    -1   0   
$EndComp
$Comp
L Switch:SW_Push SW10
U 1 1 61CB00F6
P 2100 4150
F 0 "SW10" V 2050 4300 50  0000 L CNN
F 1 "SW_Push" V 2145 4298 50  0000 L CNN
F 2 "keyswitches:SW_PG1350_reversible" H 2100 4350 50  0001 C CNN
F 3 "~" H 2100 4350 50  0001 C CNN
	1    2100 4150
	0    1    -1   0   
$EndComp
Text GLabel 5400 3950 2    50   Input ~ 0
ROW_1
$Comp
L Device:D_ALT D5
U 1 1 61CB0115
P 1250 4350
F 0 "D5" H 1250 4567 50  0000 C CNN
F 1 "D_ALT" H 1250 4476 50  0000 C CNN
F 2 "Diode_THT:D_DO-35_SOD27_P7.62mm_Horizontal" H 1250 4350 50  0001 C CNN
F 3 "~" H 1250 4350 50  0001 C CNN
	1    1250 4350
	1    0    0    1   
$EndComp
Connection ~ 2100 3950
Wire Wire Line
	1400 3950 2100 3950
$Comp
L Device:D_ALT D10
U 1 1 61CB0126
P 1950 4350
F 0 "D10" H 1950 4567 50  0000 C CNN
F 1 "D_ALT" H 1950 4476 50  0000 C CNN
F 2 "Diode_THT:D_DO-35_SOD27_P7.62mm_Horizontal" H 1950 4350 50  0001 C CNN
F 3 "~" H 1950 4350 50  0001 C CNN
	1    1950 4350
	1    0    0    1   
$EndComp
$Comp
L Device:D_ALT D16
U 1 1 61CB012C
P 2650 4350
F 0 "D16" H 2650 4567 50  0000 C CNN
F 1 "D_ALT" H 2650 4476 50  0000 C CNN
F 2 "Diode_THT:D_DO-35_SOD27_P7.62mm_Horizontal" H 2650 4350 50  0001 C CNN
F 3 "~" H 2650 4350 50  0001 C CNN
	1    2650 4350
	1    0    0    1   
$EndComp
$Comp
L Device:D_ALT D22
U 1 1 61CB0132
P 3350 4350
F 0 "D22" H 3350 4567 50  0000 C CNN
F 1 "D_ALT" H 3350 4476 50  0000 C CNN
F 2 "Diode_THT:D_DO-35_SOD27_P7.62mm_Horizontal" H 3350 4350 50  0001 C CNN
F 3 "~" H 3350 4350 50  0001 C CNN
	1    3350 4350
	1    0    0    1   
$EndComp
$Comp
L Device:D_ALT D28
U 1 1 61CB0138
P 4050 4350
F 0 "D28" H 4050 4567 50  0000 C CNN
F 1 "D_ALT" H 4050 4476 50  0000 C CNN
F 2 "Diode_THT:D_DO-35_SOD27_P7.62mm_Horizontal" H 4050 4350 50  0001 C CNN
F 3 "~" H 4050 4350 50  0001 C CNN
	1    4050 4350
	1    0    0    1   
$EndComp
$Comp
L Device:D_ALT D34
U 1 1 61CB013E
P 4750 4350
F 0 "D34" H 4750 4567 50  0000 C CNN
F 1 "D_ALT" H 4750 4476 50  0000 C CNN
F 2 "Diode_THT:D_DO-35_SOD27_P7.62mm_Horizontal" H 4750 4350 50  0001 C CNN
F 3 "~" H 4750 4350 50  0001 C CNN
	1    4750 4350
	1    0    0    1   
$EndComp
$Comp
L Switch:SW_Push SW4
U 1 1 61CB2B23
P 1400 3500
F 0 "SW4" V 1354 3648 50  0000 L CNN
F 1 "SW_Push" V 1445 3648 50  0000 L CNN
F 2 "keyswitches:SW_PG1350_reversible" H 1400 3700 50  0001 C CNN
F 3 "~" H 1400 3700 50  0001 C CNN
	1    1400 3500
	0    1    -1   0   
$EndComp
$Comp
L Switch:SW_Push SW9
U 1 1 61CB2B29
P 2100 3500
F 0 "SW9" V 2054 3648 50  0000 L CNN
F 1 "SW_Push" V 2145 3648 50  0000 L CNN
F 2 "keyswitches:SW_PG1350_reversible" H 2100 3700 50  0001 C CNN
F 3 "~" H 2100 3700 50  0001 C CNN
	1    2100 3500
	0    1    -1   0   
$EndComp
$Comp
L Switch:SW_Push SW15
U 1 1 61CB2B2F
P 2800 3500
F 0 "SW15" V 2754 3648 50  0000 L CNN
F 1 "SW_Push" V 2845 3648 50  0000 L CNN
F 2 "keyswitches:SW_PG1350_reversible" H 2800 3700 50  0001 C CNN
F 3 "~" H 2800 3700 50  0001 C CNN
	1    2800 3500
	0    1    -1   0   
$EndComp
$Comp
L Switch:SW_Push SW20
U 1 1 61CB2B35
P 3500 3500
F 0 "SW20" V 3454 3648 50  0000 L CNN
F 1 "SW_Push" V 3545 3648 50  0000 L CNN
F 2 "keyswitches:SW_PG1350_reversible" H 3500 3700 50  0001 C CNN
F 3 "~" H 3500 3700 50  0001 C CNN
	1    3500 3500
	0    1    -1   0   
$EndComp
$Comp
L Switch:SW_Push SW25
U 1 1 61CB2B3B
P 4200 3500
F 0 "SW25" V 4154 3648 50  0000 L CNN
F 1 "SW_Push" V 4245 3648 50  0000 L CNN
F 2 "keyswitches:SW_PG1350_reversible" H 4200 3700 50  0001 C CNN
F 3 "~" H 4200 3700 50  0001 C CNN
	1    4200 3500
	0    1    -1   0   
$EndComp
$Comp
L Switch:SW_Push SW30
U 1 1 61CB2B41
P 4900 3500
F 0 "SW30" V 4854 3648 50  0000 L CNN
F 1 "SW_Push" V 4945 3648 50  0000 L CNN
F 2 "keyswitches:SW_PG1350_reversible" H 4900 3700 50  0001 C CNN
F 3 "~" H 4900 3700 50  0001 C CNN
	1    4900 3500
	0    1    -1   0   
$EndComp
Text GLabel 5400 3300 2    50   Input ~ 0
ROW_2
$Comp
L Device:D_ALT D4
U 1 1 61CB2B48
P 1250 3700
F 0 "D4" H 1250 3917 50  0000 C CNN
F 1 "D_ALT" H 1250 3826 50  0000 C CNN
F 2 "Diode_THT:D_DO-35_SOD27_P7.62mm_Horizontal" H 1250 3700 50  0001 C CNN
F 3 "~" H 1250 3700 50  0001 C CNN
	1    1250 3700
	1    0    0    1   
$EndComp
Connection ~ 2100 3300
Connection ~ 2800 3300
Connection ~ 3500 3300
Connection ~ 4200 3300
Connection ~ 4900 3300
Wire Wire Line
	2100 3300 2800 3300
Wire Wire Line
	2800 3300 3500 3300
Wire Wire Line
	3500 3300 4200 3300
Wire Wire Line
	4200 3300 4900 3300
Wire Wire Line
	4900 3300 5400 3300
Wire Wire Line
	1400 3300 2100 3300
$Comp
L Device:D_ALT D9
U 1 1 61CB2B59
P 1950 3700
F 0 "D9" H 1950 3917 50  0000 C CNN
F 1 "D_ALT" H 1950 3826 50  0000 C CNN
F 2 "Diode_THT:D_DO-35_SOD27_P7.62mm_Horizontal" H 1950 3700 50  0001 C CNN
F 3 "~" H 1950 3700 50  0001 C CNN
	1    1950 3700
	1    0    0    1   
$EndComp
$Comp
L Device:D_ALT D15
U 1 1 61CB2B5F
P 2650 3700
F 0 "D15" H 2650 3917 50  0000 C CNN
F 1 "D_ALT" H 2650 3826 50  0000 C CNN
F 2 "Diode_THT:D_DO-35_SOD27_P7.62mm_Horizontal" H 2650 3700 50  0001 C CNN
F 3 "~" H 2650 3700 50  0001 C CNN
	1    2650 3700
	1    0    0    1   
$EndComp
$Comp
L Device:D_ALT D21
U 1 1 61CB2B65
P 3350 3700
F 0 "D21" H 3350 3917 50  0000 C CNN
F 1 "D_ALT" H 3350 3826 50  0000 C CNN
F 2 "Diode_THT:D_DO-35_SOD27_P7.62mm_Horizontal" H 3350 3700 50  0001 C CNN
F 3 "~" H 3350 3700 50  0001 C CNN
	1    3350 3700
	1    0    0    1   
$EndComp
$Comp
L Device:D_ALT D27
U 1 1 61CB2B6B
P 4050 3700
F 0 "D27" H 4050 3917 50  0000 C CNN
F 1 "D_ALT" H 4050 3826 50  0000 C CNN
F 2 "Diode_THT:D_DO-35_SOD27_P7.62mm_Horizontal" H 4050 3700 50  0001 C CNN
F 3 "~" H 4050 3700 50  0001 C CNN
	1    4050 3700
	1    0    0    1   
$EndComp
$Comp
L Device:D_ALT D33
U 1 1 61CB2B71
P 4750 3700
F 0 "D33" H 4750 3917 50  0000 C CNN
F 1 "D_ALT" H 4750 3826 50  0000 C CNN
F 2 "Diode_THT:D_DO-35_SOD27_P7.62mm_Horizontal" H 4750 3700 50  0001 C CNN
F 3 "~" H 4750 3700 50  0001 C CNN
	1    4750 3700
	1    0    0    1   
$EndComp
$Comp
L Switch:SW_Push SW3
U 1 1 61CB5869
P 1400 2850
F 0 "SW3" V 1354 2998 50  0000 L CNN
F 1 "SW_Push" V 1445 2998 50  0000 L CNN
F 2 "keyswitches:SW_PG1350_reversible" H 1400 3050 50  0001 C CNN
F 3 "~" H 1400 3050 50  0001 C CNN
	1    1400 2850
	0    1    -1   0   
$EndComp
$Comp
L Switch:SW_Push SW8
U 1 1 61CB586F
P 2100 2850
F 0 "SW8" V 2054 2998 50  0000 L CNN
F 1 "SW_Push" V 2145 2998 50  0000 L CNN
F 2 "keyswitches:SW_PG1350_reversible" H 2100 3050 50  0001 C CNN
F 3 "~" H 2100 3050 50  0001 C CNN
	1    2100 2850
	0    1    -1   0   
$EndComp
$Comp
L Switch:SW_Push SW14
U 1 1 61CB5875
P 2800 2850
F 0 "SW14" V 2754 2998 50  0000 L CNN
F 1 "SW_Push" V 2845 2998 50  0000 L CNN
F 2 "keyswitches:SW_PG1350_reversible" H 2800 3050 50  0001 C CNN
F 3 "~" H 2800 3050 50  0001 C CNN
	1    2800 2850
	0    1    -1   0   
$EndComp
$Comp
L Switch:SW_Push SW19
U 1 1 61CB587B
P 3500 2850
F 0 "SW19" V 3454 2998 50  0000 L CNN
F 1 "SW_Push" V 3545 2998 50  0000 L CNN
F 2 "keyswitches:SW_PG1350_reversible" H 3500 3050 50  0001 C CNN
F 3 "~" H 3500 3050 50  0001 C CNN
	1    3500 2850
	0    1    -1   0   
$EndComp
$Comp
L Switch:SW_Push SW24
U 1 1 61CB5881
P 4200 2850
F 0 "SW24" V 4154 2998 50  0000 L CNN
F 1 "SW_Push" V 4245 2998 50  0000 L CNN
F 2 "keyswitches:SW_PG1350_reversible" H 4200 3050 50  0001 C CNN
F 3 "~" H 4200 3050 50  0001 C CNN
	1    4200 2850
	0    1    -1   0   
$EndComp
$Comp
L Switch:SW_Push SW29
U 1 1 61CB5887
P 4900 2850
F 0 "SW29" V 4854 2998 50  0000 L CNN
F 1 "SW_Push" V 4945 2998 50  0000 L CNN
F 2 "keyswitches:SW_PG1350_reversible" H 4900 3050 50  0001 C CNN
F 3 "~" H 4900 3050 50  0001 C CNN
	1    4900 2850
	0    1    -1   0   
$EndComp
Text GLabel 5400 2650 2    50   Input ~ 0
ROW_3
$Comp
L Device:D_ALT D3
U 1 1 61CB588E
P 1250 3050
F 0 "D3" H 1250 3267 50  0000 C CNN
F 1 "D_ALT" H 1250 3176 50  0000 C CNN
F 2 "Diode_THT:D_DO-35_SOD27_P7.62mm_Horizontal" H 1250 3050 50  0001 C CNN
F 3 "~" H 1250 3050 50  0001 C CNN
	1    1250 3050
	1    0    0    1   
$EndComp
Connection ~ 2100 2650
Connection ~ 2800 2650
Connection ~ 3500 2650
Connection ~ 4200 2650
Connection ~ 4900 2650
Wire Wire Line
	2100 2650 2800 2650
Wire Wire Line
	2800 2650 3500 2650
Wire Wire Line
	3500 2650 4200 2650
Wire Wire Line
	4200 2650 4900 2650
Wire Wire Line
	4900 2650 5400 2650
Wire Wire Line
	1400 2650 2100 2650
$Comp
L Device:D_ALT D8
U 1 1 61CB589F
P 1950 3050
F 0 "D8" H 1950 3267 50  0000 C CNN
F 1 "D_ALT" H 1950 3176 50  0000 C CNN
F 2 "Diode_THT:D_DO-35_SOD27_P7.62mm_Horizontal" H 1950 3050 50  0001 C CNN
F 3 "~" H 1950 3050 50  0001 C CNN
	1    1950 3050
	1    0    0    1   
$EndComp
$Comp
L Device:D_ALT D14
U 1 1 61CB58A5
P 2650 3050
F 0 "D14" H 2650 3267 50  0000 C CNN
F 1 "D_ALT" H 2650 3176 50  0000 C CNN
F 2 "Diode_THT:D_DO-35_SOD27_P7.62mm_Horizontal" H 2650 3050 50  0001 C CNN
F 3 "~" H 2650 3050 50  0001 C CNN
	1    2650 3050
	1    0    0    1   
$EndComp
$Comp
L Device:D_ALT D20
U 1 1 61CB58AB
P 3350 3050
F 0 "D20" H 3350 3267 50  0000 C CNN
F 1 "D_ALT" H 3350 3176 50  0000 C CNN
F 2 "Diode_THT:D_DO-35_SOD27_P7.62mm_Horizontal" H 3350 3050 50  0001 C CNN
F 3 "~" H 3350 3050 50  0001 C CNN
	1    3350 3050
	1    0    0    1   
$EndComp
$Comp
L Device:D_ALT D26
U 1 1 61CB58B1
P 4050 3050
F 0 "D26" H 4050 3267 50  0000 C CNN
F 1 "D_ALT" H 4050 3176 50  0000 C CNN
F 2 "Diode_THT:D_DO-35_SOD27_P7.62mm_Horizontal" H 4050 3050 50  0001 C CNN
F 3 "~" H 4050 3050 50  0001 C CNN
	1    4050 3050
	1    0    0    1   
$EndComp
$Comp
L Device:D_ALT D32
U 1 1 61CB58B7
P 4750 3050
F 0 "D32" H 4750 3267 50  0000 C CNN
F 1 "D_ALT" H 4750 3176 50  0000 C CNN
F 2 "Diode_THT:D_DO-35_SOD27_P7.62mm_Horizontal" H 4750 3050 50  0001 C CNN
F 3 "~" H 4750 3050 50  0001 C CNN
	1    4750 3050
	1    0    0    1   
$EndComp
$Comp
L Switch:SW_Push SW2
U 1 1 61CBF1AE
P 1400 2200
F 0 "SW2" V 1354 2348 50  0000 L CNN
F 1 "SW_Push" V 1445 2348 50  0000 L CNN
F 2 "keyswitches:SW_PG1350_reversible" H 1400 2400 50  0001 C CNN
F 3 "~" H 1400 2400 50  0001 C CNN
	1    1400 2200
	0    1    -1   0   
$EndComp
$Comp
L Switch:SW_Push SW7
U 1 1 61CBF1B4
P 2100 2200
F 0 "SW7" V 2054 2348 50  0000 L CNN
F 1 "SW_Push" V 2145 2348 50  0000 L CNN
F 2 "keyswitches:SW_PG1350_reversible" H 2100 2400 50  0001 C CNN
F 3 "~" H 2100 2400 50  0001 C CNN
	1    2100 2200
	0    1    -1   0   
$EndComp
$Comp
L Switch:SW_Push SW13
U 1 1 61CBF1BA
P 2800 2200
F 0 "SW13" V 2754 2348 50  0000 L CNN
F 1 "SW_Push" V 2845 2348 50  0000 L CNN
F 2 "keyswitches:SW_PG1350_reversible" H 2800 2400 50  0001 C CNN
F 3 "~" H 2800 2400 50  0001 C CNN
	1    2800 2200
	0    1    -1   0   
$EndComp
$Comp
L Switch:SW_Push SW18
U 1 1 61CBF1C0
P 3500 2200
F 0 "SW18" V 3454 2348 50  0000 L CNN
F 1 "SW_Push" V 3545 2348 50  0000 L CNN
F 2 "keyswitches:SW_PG1350_reversible" H 3500 2400 50  0001 C CNN
F 3 "~" H 3500 2400 50  0001 C CNN
	1    3500 2200
	0    1    -1   0   
$EndComp
$Comp
L Switch:SW_Push SW23
U 1 1 61CBF1C6
P 4200 2200
F 0 "SW23" V 4154 2348 50  0000 L CNN
F 1 "SW_Push" V 4245 2348 50  0000 L CNN
F 2 "keyswitches:SW_PG1350_reversible" H 4200 2400 50  0001 C CNN
F 3 "~" H 4200 2400 50  0001 C CNN
	1    4200 2200
	0    1    -1   0   
$EndComp
$Comp
L Switch:SW_Push SW28
U 1 1 61CBF1CC
P 4900 2200
F 0 "SW28" V 4854 2348 50  0000 L CNN
F 1 "SW_Push" V 4945 2348 50  0000 L CNN
F 2 "keyswitches:SW_PG1350_reversible" H 4900 2400 50  0001 C CNN
F 3 "~" H 4900 2400 50  0001 C CNN
	1    4900 2200
	0    1    -1   0   
$EndComp
Text GLabel 5400 2000 2    50   Input ~ 0
ROW_4
$Comp
L Device:D_ALT D2
U 1 1 61CBF1D3
P 1250 2400
F 0 "D2" H 1250 2617 50  0000 C CNN
F 1 "D_ALT" H 1250 2526 50  0000 C CNN
F 2 "Diode_THT:D_DO-35_SOD27_P7.62mm_Horizontal" H 1250 2400 50  0001 C CNN
F 3 "~" H 1250 2400 50  0001 C CNN
	1    1250 2400
	1    0    0    1   
$EndComp
Connection ~ 2100 2000
Connection ~ 2800 2000
Connection ~ 3500 2000
Connection ~ 4200 2000
Connection ~ 4900 2000
Wire Wire Line
	2100 2000 2800 2000
Wire Wire Line
	2800 2000 3500 2000
Wire Wire Line
	3500 2000 4200 2000
Wire Wire Line
	4200 2000 4900 2000
Wire Wire Line
	4900 2000 5400 2000
Wire Wire Line
	1400 2000 2100 2000
$Comp
L Device:D_ALT D7
U 1 1 61CBF1E4
P 1950 2400
F 0 "D7" H 1950 2617 50  0000 C CNN
F 1 "D_ALT" H 1950 2526 50  0000 C CNN
F 2 "Diode_THT:D_DO-35_SOD27_P7.62mm_Horizontal" H 1950 2400 50  0001 C CNN
F 3 "~" H 1950 2400 50  0001 C CNN
	1    1950 2400
	1    0    0    1   
$EndComp
$Comp
L Device:D_ALT D13
U 1 1 61CBF1EA
P 2650 2400
F 0 "D13" H 2650 2617 50  0000 C CNN
F 1 "D_ALT" H 2650 2526 50  0000 C CNN
F 2 "Diode_THT:D_DO-35_SOD27_P7.62mm_Horizontal" H 2650 2400 50  0001 C CNN
F 3 "~" H 2650 2400 50  0001 C CNN
	1    2650 2400
	1    0    0    1   
$EndComp
$Comp
L Device:D_ALT D19
U 1 1 61CBF1F0
P 3350 2400
F 0 "D19" H 3350 2617 50  0000 C CNN
F 1 "D_ALT" H 3350 2526 50  0000 C CNN
F 2 "Diode_THT:D_DO-35_SOD27_P7.62mm_Horizontal" H 3350 2400 50  0001 C CNN
F 3 "~" H 3350 2400 50  0001 C CNN
	1    3350 2400
	1    0    0    1   
$EndComp
$Comp
L Device:D_ALT D25
U 1 1 61CBF1F6
P 4050 2400
F 0 "D25" H 4050 2617 50  0000 C CNN
F 1 "D_ALT" H 4050 2526 50  0000 C CNN
F 2 "Diode_THT:D_DO-35_SOD27_P7.62mm_Horizontal" H 4050 2400 50  0001 C CNN
F 3 "~" H 4050 2400 50  0001 C CNN
	1    4050 2400
	1    0    0    1   
$EndComp
$Comp
L Device:D_ALT D31
U 1 1 61CBF1FC
P 4750 2400
F 0 "D31" H 4750 2617 50  0000 C CNN
F 1 "D_ALT" H 4750 2526 50  0000 C CNN
F 2 "Diode_THT:D_DO-35_SOD27_P7.62mm_Horizontal" H 4750 2400 50  0001 C CNN
F 3 "~" H 4750 2400 50  0001 C CNN
	1    4750 2400
	1    0    0    1   
$EndComp
Wire Wire Line
	1100 3050 1100 2400
Connection ~ 1100 3050
Wire Wire Line
	1100 3700 1100 3050
Connection ~ 1100 3700
Wire Wire Line
	1100 4350 1100 3700
Connection ~ 1100 4350
Connection ~ 1800 3050
Wire Wire Line
	1800 3050 1800 2400
Connection ~ 1800 4350
Wire Wire Line
	1800 4350 1800 3700
Connection ~ 1800 3700
Wire Wire Line
	1800 3700 1800 3050
Connection ~ 2500 4350
Wire Wire Line
	2500 4350 2500 3700
Connection ~ 2500 3700
Wire Wire Line
	2500 3700 2500 3050
Connection ~ 2500 3050
Wire Wire Line
	2500 3050 2500 2400
Wire Wire Line
	3200 5000 3200 4350
Connection ~ 3200 5000
Connection ~ 3200 4350
Wire Wire Line
	3200 4350 3200 3700
Connection ~ 3200 3700
Wire Wire Line
	3200 3700 3200 3050
Connection ~ 3200 3050
Wire Wire Line
	3200 3050 3200 2400
Wire Wire Line
	3900 5000 3900 4350
Connection ~ 3900 5000
Connection ~ 3900 4350
Wire Wire Line
	3900 4350 3900 3700
Connection ~ 3900 3700
Wire Wire Line
	3900 3700 3900 3050
Connection ~ 3900 3050
Wire Wire Line
	3900 3050 3900 2400
Wire Wire Line
	4600 5000 4600 4350
Connection ~ 4600 5000
Connection ~ 4600 4350
Wire Wire Line
	4600 4350 4600 3700
Connection ~ 4600 3700
Wire Wire Line
	4600 3700 4600 3050
Connection ~ 4600 3050
Wire Wire Line
	4600 3050 4600 2400
$Comp
L Switch:SW_Push SW1
U 1 1 61CDDA5C
P 1400 1600
F 0 "SW1" V 1354 1748 50  0000 L CNN
F 1 "SW_Push" V 1445 1748 50  0000 L CNN
F 2 "keyswitches:SW_PG1350_reversible" H 1400 1800 50  0001 C CNN
F 3 "~" H 1400 1800 50  0001 C CNN
	1    1400 1600
	0    1    -1   0   
$EndComp
$Comp
L Switch:SW_Push SW6
U 1 1 61CDDA62
P 2100 1600
F 0 "SW6" V 2054 1748 50  0000 L CNN
F 1 "SW_Push" V 2145 1748 50  0000 L CNN
F 2 "keyswitches:SW_PG1350_reversible" H 2100 1800 50  0001 C CNN
F 3 "~" H 2100 1800 50  0001 C CNN
	1    2100 1600
	0    1    -1   0   
$EndComp
$Comp
L Switch:SW_Push SW12
U 1 1 61CDDA68
P 2800 1600
F 0 "SW12" V 2754 1748 50  0000 L CNN
F 1 "SW_Push" V 2845 1748 50  0000 L CNN
F 2 "keyswitches:SW_PG1350_reversible" H 2800 1800 50  0001 C CNN
F 3 "~" H 2800 1800 50  0001 C CNN
	1    2800 1600
	0    1    -1   0   
$EndComp
$Comp
L Switch:SW_Push SW17
U 1 1 61CDDA6E
P 3500 1600
F 0 "SW17" V 3454 1748 50  0000 L CNN
F 1 "SW_Push" V 3545 1748 50  0000 L CNN
F 2 "keyswitches:SW_PG1350_reversible" H 3500 1800 50  0001 C CNN
F 3 "~" H 3500 1800 50  0001 C CNN
	1    3500 1600
	0    1    -1   0   
$EndComp
$Comp
L Switch:SW_Push SW22
U 1 1 61CDDA74
P 4200 1600
F 0 "SW22" V 4154 1748 50  0000 L CNN
F 1 "SW_Push" V 4245 1748 50  0000 L CNN
F 2 "keyswitches:SW_PG1350_reversible" H 4200 1800 50  0001 C CNN
F 3 "~" H 4200 1800 50  0001 C CNN
	1    4200 1600
	0    1    -1   0   
$EndComp
$Comp
L Switch:SW_Push SW27
U 1 1 61CDDA7A
P 4900 1600
F 0 "SW27" V 4854 1748 50  0000 L CNN
F 1 "SW_Push" V 4945 1748 50  0000 L CNN
F 2 "keyswitches:SW_PG1350_reversible" H 4900 1800 50  0001 C CNN
F 3 "~" H 4900 1800 50  0001 C CNN
	1    4900 1600
	0    1    -1   0   
$EndComp
Text GLabel 5400 1400 2    50   Input ~ 0
ROW_5
$Comp
L Device:D_ALT D1
U 1 1 61CDDA81
P 1250 1800
F 0 "D1" H 1250 2017 50  0000 C CNN
F 1 "D_ALT" H 1250 1926 50  0000 C CNN
F 2 "Diode_THT:D_DO-35_SOD27_P7.62mm_Horizontal" H 1250 1800 50  0001 C CNN
F 3 "~" H 1250 1800 50  0001 C CNN
	1    1250 1800
	1    0    0    1   
$EndComp
Connection ~ 2100 1400
Connection ~ 2800 1400
Connection ~ 3500 1400
Connection ~ 4200 1400
Connection ~ 4900 1400
Wire Wire Line
	2100 1400 2800 1400
Wire Wire Line
	2800 1400 3500 1400
Wire Wire Line
	3500 1400 4200 1400
Wire Wire Line
	4200 1400 4900 1400
Wire Wire Line
	4900 1400 5400 1400
Wire Wire Line
	1400 1400 2100 1400
$Comp
L Device:D_ALT D6
U 1 1 61CDDA92
P 1950 1800
F 0 "D6" H 1950 2017 50  0000 C CNN
F 1 "D_ALT" H 1950 1926 50  0000 C CNN
F 2 "Diode_THT:D_DO-35_SOD27_P7.62mm_Horizontal" H 1950 1800 50  0001 C CNN
F 3 "~" H 1950 1800 50  0001 C CNN
	1    1950 1800
	1    0    0    1   
$EndComp
$Comp
L Device:D_ALT D12
U 1 1 61CDDA98
P 2650 1800
F 0 "D12" H 2650 2017 50  0000 C CNN
F 1 "D_ALT" H 2650 1926 50  0000 C CNN
F 2 "Diode_THT:D_DO-35_SOD27_P7.62mm_Horizontal" H 2650 1800 50  0001 C CNN
F 3 "~" H 2650 1800 50  0001 C CNN
	1    2650 1800
	1    0    0    1   
$EndComp
$Comp
L Device:D_ALT D18
U 1 1 61CDDA9E
P 3350 1800
F 0 "D18" H 3350 2017 50  0000 C CNN
F 1 "D_ALT" H 3350 1926 50  0000 C CNN
F 2 "Diode_THT:D_DO-35_SOD27_P7.62mm_Horizontal" H 3350 1800 50  0001 C CNN
F 3 "~" H 3350 1800 50  0001 C CNN
	1    3350 1800
	1    0    0    1   
$EndComp
$Comp
L Device:D_ALT D24
U 1 1 61CDDAA4
P 4050 1800
F 0 "D24" H 4050 2017 50  0000 C CNN
F 1 "D_ALT" H 4050 1926 50  0000 C CNN
F 2 "Diode_THT:D_DO-35_SOD27_P7.62mm_Horizontal" H 4050 1800 50  0001 C CNN
F 3 "~" H 4050 1800 50  0001 C CNN
	1    4050 1800
	1    0    0    1   
$EndComp
$Comp
L Device:D_ALT D30
U 1 1 61CDDAAA
P 4750 1800
F 0 "D30" H 4750 2017 50  0000 C CNN
F 1 "D_ALT" H 4750 1926 50  0000 C CNN
F 2 "Diode_THT:D_DO-35_SOD27_P7.62mm_Horizontal" H 4750 1800 50  0001 C CNN
F 3 "~" H 4750 1800 50  0001 C CNN
	1    4750 1800
	1    0    0    1   
$EndComp
Wire Wire Line
	1100 2400 1100 1800
Connection ~ 1100 2400
Wire Wire Line
	1800 2400 1800 1800
Connection ~ 1800 2400
Wire Wire Line
	2500 2400 2500 1800
Connection ~ 2500 2400
Wire Wire Line
	3200 2400 3200 1800
Connection ~ 3200 2400
Wire Wire Line
	3900 2400 3900 1800
Connection ~ 3900 2400
Wire Wire Line
	4600 2400 4600 1800
Connection ~ 4600 2400
$Comp
L Connector_Generic:Conn_01x04 e1
U 1 1 61CF213E
P 8950 2050
F 0 "e1" H 9030 2042 50  0000 L CNN
F 1 "I2C OLED" H 9030 1951 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x04_P2.54mm_Vertical" H 8950 2050 50  0001 C CNN
F 3 "~" H 8950 2050 50  0001 C CNN
	1    8950 2050
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR06
U 1 1 61CF3E48
P 8250 1950
F 0 "#PWR06" H 8250 1700 50  0001 C CNN
F 1 "GND" V 8250 1800 50  0000 R CNN
F 2 "" H 8250 1950 50  0001 C CNN
F 3 "" H 8250 1950 50  0001 C CNN
	1    8250 1950
	0    1    1    0   
$EndComp
$Comp
L power:+3V3 #PWR07
U 1 1 61CF4238
P 8250 2050
F 0 "#PWR07" H 8250 1900 50  0001 C CNN
F 1 "+3V3" V 8250 2200 50  0000 L CNN
F 2 "" H 8250 2050 50  0001 C CNN
F 3 "" H 8250 2050 50  0001 C CNN
	1    8250 2050
	0    -1   -1   0   
$EndComp
Text GLabel 8250 2250 0    50   Input ~ 0
I2C_SDA_SCL
Text GLabel 8250 2150 0    50   Input ~ 0
I2C_SCL_SDA
$Comp
L Connector:AudioJack4 J1
U 1 1 61CF6C8E
P 8050 1250
F 0 "J1" H 8007 1575 50  0000 C CNN
F 1 "AudioJack4" H 8007 1484 50  0000 C CNN
F 2 "footprints:TRRS-PJ-320A_reversable" H 8050 1250 50  0001 C CNN
F 3 "~" H 8050 1250 50  0001 C CNN
	1    8050 1250
	1    0    0    -1  
$EndComp
$Comp
L power:VBUS #PWR05
U 1 1 61CF862D
P 8250 1450
F 0 "#PWR05" H 8250 1300 50  0001 C CNN
F 1 "VBUS" V 8265 1578 50  0000 L CNN
F 2 "" H 8250 1450 50  0001 C CNN
F 3 "" H 8250 1450 50  0001 C CNN
	1    8250 1450
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR04
U 1 1 61CF9D64
P 8250 1150
F 0 "#PWR04" H 8250 900 50  0001 C CNN
F 1 "GND" V 8255 1022 50  0000 R CNN
F 2 "" H 8250 1150 50  0001 C CNN
F 3 "" H 8250 1150 50  0001 C CNN
	1    8250 1150
	0    -1   -1   0   
$EndComp
Text GLabel 8250 1250 2    50   Input ~ 0
UART_TX
Text GLabel 8250 1350 2    50   Input ~ 0
UART_RX
Wire Wire Line
	2500 5300 2500 5000
$Comp
L Device:Rotary_Encoder_Switch SW32
U 1 1 61D11DCE
P 6750 2450
F 0 "SW32" H 6750 2817 50  0000 C CNN
F 1 "Rotary_Encoder_Switch" H 6750 2726 50  0000 C CNN
F 2 "Rotary_Encoder:RotaryEncoder_Alps_EC11E-Switch_Vertical_H20mm" H 6600 2610 50  0001 C CNN
F 3 "~" H 6750 2710 50  0001 C CNN
	1    6750 2450
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR020
U 1 1 61D25AB2
P 10300 2150
F 0 "#PWR020" H 10300 1900 50  0001 C CNN
F 1 "GND" V 10305 2022 50  0000 R CNN
F 2 "" H 10300 2150 50  0001 C CNN
F 3 "" H 10300 2150 50  0001 C CNN
	1    10300 2150
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR01
U 1 1 61D272AC
P 6450 2450
F 0 "#PWR01" H 6450 2200 50  0001 C CNN
F 1 "GND" V 6450 2300 50  0000 R CNN
F 2 "" H 6450 2450 50  0001 C CNN
F 3 "" H 6450 2450 50  0001 C CNN
	1    6450 2450
	0    1    1    0   
$EndComp
Text GLabel 9700 4800 2    50   Input ~ 0
ROT_A
$Comp
L power:GND #PWR02
U 1 1 61D2A748
P 7050 2350
F 0 "#PWR02" H 7050 2100 50  0001 C CNN
F 1 "GND" V 7050 2200 50  0000 R CNN
F 2 "" H 7050 2350 50  0001 C CNN
F 3 "" H 7050 2350 50  0001 C CNN
	1    7050 2350
	0    -1   -1   0   
$EndComp
Text GLabel 6450 2350 0    50   Input ~ 0
ROT_A
Text GLabel 6450 2550 0    50   Input ~ 0
ROT_B
Text GLabel 7050 2550 2    50   Input ~ 0
ROT_SW
Text GLabel 8950 5900 3    50   Input ~ 0
ROT_SW
$Comp
L Switch:SW_Push_Dual_x2 SW31
U 1 1 61C33F3C
P 4900 4150
F 0 "SW31" V 4854 4298 50  0000 L CNN
F 1 "SW_Push_Dual_x2" V 4945 4298 50  0000 L CNN
F 2 "footprints:SW_PG1350_reversible_double_rotatable" H 4900 4350 50  0001 C CNN
F 3 "~" H 4900 4350 50  0001 C CNN
	1    4900 4150
	0    1    -1   0   
$EndComp
$Comp
L Switch:SW_Push_Dual_x2 SW31
U 2 1 61C34F41
P 4900 4800
F 0 "SW31" V 4854 4948 50  0000 L CNN
F 1 "SW_Push_Dual_x2" V 4945 4948 50  0000 L CNN
F 2 "footprints:SW_PG1350_reversible_double_rotatable" H 4900 5000 50  0001 C CNN
F 3 "~" H 4900 5000 50  0001 C CNN
	2    4900 4800
	0    1    -1   0   
$EndComp
Connection ~ 4900 4600
Wire Wire Line
	4900 4600 5400 4600
Connection ~ 4900 3950
Wire Wire Line
	4900 3950 5400 3950
$Comp
L Switch:SW_Push_Dual_x2 SW26
U 1 1 61C4159B
P 4200 4150
F 0 "SW26" V 4154 4298 50  0000 L CNN
F 1 "SW_Push_Dual_x2" V 4245 4298 50  0000 L CNN
F 2 "footprints:SW_PG1350_reversible_double_rotatable" H 4200 4350 50  0001 C CNN
F 3 "~" H 4200 4350 50  0001 C CNN
	1    4200 4150
	0    1    -1   0   
$EndComp
Connection ~ 4200 3950
Wire Wire Line
	4200 3950 4900 3950
$Comp
L Switch:SW_Push_Dual_x2 SW21
U 1 1 61C41CA2
P 3500 4150
F 0 "SW21" V 3454 4298 50  0000 L CNN
F 1 "SW_Push_Dual_x2" V 3545 4298 50  0000 L CNN
F 2 "footprints:SW_PG1350_reversible_double_rotatable" H 3500 4350 50  0001 C CNN
F 3 "~" H 3500 4350 50  0001 C CNN
	1    3500 4150
	0    1    -1   0   
$EndComp
Connection ~ 3500 3950
Wire Wire Line
	3500 3950 4200 3950
$Comp
L Switch:SW_Push_Dual_x2 SW26
U 2 1 61C42512
P 4200 4800
F 0 "SW26" V 4154 4948 50  0000 L CNN
F 1 "SW_Push_Dual_x2" V 4245 4948 50  0000 L CNN
F 2 "footprints:SW_PG1350_reversible_double_rotatable" H 4200 5000 50  0001 C CNN
F 3 "~" H 4200 5000 50  0001 C CNN
	2    4200 4800
	0    1    -1   0   
$EndComp
Connection ~ 4200 4600
Wire Wire Line
	4200 4600 4900 4600
$Comp
L Switch:SW_Push_Dual_x2 SW21
U 2 1 61C42B7A
P 3500 4800
F 0 "SW21" V 3454 4948 50  0000 L CNN
F 1 "SW_Push_Dual_x2" V 3545 4948 50  0000 L CNN
F 2 "footprints:SW_PG1350_reversible_double_rotatable" H 3500 5000 50  0001 C CNN
F 3 "~" H 3500 5000 50  0001 C CNN
	2    3500 4800
	0    1    -1   0   
$EndComp
Connection ~ 3500 4600
Wire Wire Line
	3500 4600 4200 4600
Wire Wire Line
	2100 3950 2800 3950
$Comp
L Device:D_ALT D17
U 1 1 61C34EA8
P 2650 5000
F 0 "D17" H 2650 5217 50  0000 C CNN
F 1 "D_ALT" H 2650 5126 50  0000 C CNN
F 2 "Diode_THT:D_DO-35_SOD27_P7.62mm_Horizontal" H 2650 5000 50  0001 C CNN
F 3 "~" H 2650 5000 50  0001 C CNN
	1    2650 5000
	1    0    0    1   
$EndComp
Connection ~ 2500 5000
Wire Wire Line
	2500 5000 2500 4350
$Comp
L Switch:SW_Push_Dual_x2 SW16
U 2 1 61C35305
P 2800 4800
F 0 "SW16" V 2754 4948 50  0000 L CNN
F 1 "SW_Push_Dual_x2" V 2845 4948 50  0000 L CNN
F 2 "footprints:SW_PG1350_reversible_double_rotatable" H 2800 5000 50  0001 C CNN
F 3 "~" H 2800 5000 50  0001 C CNN
	2    2800 4800
	0    1    -1   0   
$EndComp
Connection ~ 2800 4600
Wire Wire Line
	2800 4600 3500 4600
$Comp
L Switch:SW_Push_Dual_x2 SW16
U 1 1 61C35B95
P 2800 4150
F 0 "SW16" V 2754 4298 50  0000 L CNN
F 1 "SW_Push_Dual_x2" V 2845 4298 50  0000 L CNN
F 2 "footprints:SW_PG1350_reversible_double_rotatable" H 2800 4350 50  0001 C CNN
F 3 "~" H 2800 4350 50  0001 C CNN
	1    2800 4150
	0    1    -1   0   
$EndComp
Connection ~ 2800 3950
Wire Wire Line
	2800 3950 3500 3950
$Comp
L power:GND #PWR08
U 1 1 61CBF1FB
P 8300 3100
F 0 "#PWR08" H 8300 2850 50  0001 C CNN
F 1 "GND" V 8300 2950 50  0000 R CNN
F 2 "" H 8300 3100 50  0001 C CNN
F 3 "" H 8300 3100 50  0001 C CNN
	1    8300 3100
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR010
U 1 1 61CC13E9
P 8300 3600
F 0 "#PWR010" H 8300 3350 50  0001 C CNN
F 1 "GND" V 8300 3450 50  0000 R CNN
F 2 "" H 8300 3600 50  0001 C CNN
F 3 "" H 8300 3600 50  0001 C CNN
	1    8300 3600
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR011
U 1 1 61CC1677
P 8300 4100
F 0 "#PWR011" H 8300 3850 50  0001 C CNN
F 1 "GND" V 8300 3950 50  0000 R CNN
F 2 "" H 8300 4100 50  0001 C CNN
F 3 "" H 8300 4100 50  0001 C CNN
	1    8300 4100
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR012
U 1 1 61CC1AD9
P 8300 4600
F 0 "#PWR012" H 8300 4350 50  0001 C CNN
F 1 "GND" V 8300 4450 50  0000 R CNN
F 2 "" H 8300 4600 50  0001 C CNN
F 3 "" H 8300 4600 50  0001 C CNN
	1    8300 4600
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR018
U 1 1 61CC1DF0
P 9700 4600
F 0 "#PWR018" H 9700 4350 50  0001 C CNN
F 1 "GND" V 9700 4450 50  0000 R CNN
F 2 "" H 9700 4600 50  0001 C CNN
F 3 "" H 9700 4600 50  0001 C CNN
	1    9700 4600
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR017
U 1 1 61CC27D8
P 9700 4100
F 0 "#PWR017" H 9700 3850 50  0001 C CNN
F 1 "GND" V 9700 3950 50  0000 R CNN
F 2 "" H 9700 4100 50  0001 C CNN
F 3 "" H 9700 4100 50  0001 C CNN
	1    9700 4100
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR014
U 1 1 61CC2DE6
P 9700 3100
F 0 "#PWR014" H 9700 2850 50  0001 C CNN
F 1 "GND" V 9700 2950 50  0000 R CNN
F 2 "" H 9700 3100 50  0001 C CNN
F 3 "" H 9700 3100 50  0001 C CNN
	1    9700 3100
	0    -1   -1   0   
$EndComp
Wire Wire Line
	1800 5300 1800 5000
Wire Wire Line
	1400 4600 2100 4600
Connection ~ 2100 4600
Wire Wire Line
	2100 4600 2800 4600
Connection ~ 1800 5000
Wire Wire Line
	1800 5000 1800 4350
Wire Wire Line
	1100 4350 1100 5300
Wire Wire Line
	8300 2900 8100 2900
Wire Wire Line
	8100 2900 8100 2950
Wire Wire Line
	8100 3000 8300 3000
$Comp
L power:VBUS #PWR03
U 1 1 61CA0109
P 8100 2950
F 0 "#PWR03" H 8100 2800 50  0001 C CNN
F 1 "VBUS" V 8115 3078 50  0000 L CNN
F 2 "" H 8100 2950 50  0001 C CNN
F 3 "" H 8100 2950 50  0001 C CNN
	1    8100 2950
	0    -1   -1   0   
$EndComp
Connection ~ 8100 2950
Wire Wire Line
	8100 2950 8100 3000
$Comp
L power:+3V3 #PWR09
U 1 1 61CA0743
P 8300 3300
F 0 "#PWR09" H 8300 3150 50  0001 C CNN
F 1 "+3V3" V 8315 3428 50  0000 L CNN
F 2 "" H 8300 3300 50  0001 C CNN
F 3 "" H 8300 3300 50  0001 C CNN
	1    8300 3300
	0    -1   -1   0   
$EndComp
Text GLabel 9050 5900 3    50   Input ~ 0
Reset
Text GLabel 9700 4700 2    50   Input ~ 0
ROT_B
Text GLabel 8300 4500 0    50   Input ~ 0
UART_RX
Text GLabel 8300 4400 0    50   Input ~ 0
UART_TX
Text GLabel 9700 4500 2    50   Input ~ 0
UART_TX
$Comp
L power:GND #PWR016
U 1 1 61CB063E
P 9700 3600
F 0 "#PWR016" H 9700 3350 50  0001 C CNN
F 1 "GND" V 9700 3450 50  0000 R CNN
F 2 "" H 9700 3600 50  0001 C CNN
F 3 "" H 9700 3600 50  0001 C CNN
	1    9700 3600
	0    -1   -1   0   
$EndComp
NoConn ~ 9700 3200
NoConn ~ 8300 3200
NoConn ~ 9700 3400
NoConn ~ 8300 3400
$Comp
L Connector_Generic:Conn_02x02_Top_Bottom J3
U 1 1 61C4D91B
P 9050 5600
F 0 "J3" V 9054 5680 50  0000 L CNN
F 1 "Conn_02x02_Top_Bottom" V 9145 5680 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_2x02_P2.54mm_Vertical" H 9050 5600 50  0001 C CNN
F 3 "~" H 9050 5600 50  0001 C CNN
	1    9050 5600
	0    1    1    0   
$EndComp
Wire Wire Line
	9700 3900 10250 3900
Wire Wire Line
	10250 3900 10250 5400
Wire Wire Line
	10250 5400 9050 5400
Wire Wire Line
	8950 5400 7600 5400
Wire Wire Line
	7600 5400 7600 3900
Wire Wire Line
	7600 3900 8300 3900
$Comp
L Connector_Generic:Conn_02x04_Top_Bottom J2
U 1 1 61C5EAE6
P 8450 2050
F 0 "J2" H 8500 2367 50  0000 C CNN
F 1 "Conn_02x04_Top_Bottom" H 8500 2276 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_2x04_P2.54mm_Vertical" H 8450 2050 50  0001 C CNN
F 3 "~" H 8450 2050 50  0001 C CNN
	1    8450 2050
	1    0    0    -1  
$EndComp
$EndSCHEMATC
