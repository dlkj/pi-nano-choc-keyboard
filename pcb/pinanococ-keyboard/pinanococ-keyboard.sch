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
F 2 "MCU_RaspberryPi_and_Boards:RPi_Pico_SMD_TH" V 9000 3850 50  0001 C CNN
F 3 "" H 9000 3850 50  0001 C CNN
	1    9000 3850
	1    0    0    -1  
$EndComp
NoConn ~ 8900 5000
NoConn ~ 9100 5000
NoConn ~ 9700 3200
NoConn ~ 9700 3400
NoConn ~ 8300 3100
NoConn ~ 8300 4600
NoConn ~ 8300 4100
NoConn ~ 8300 3600
NoConn ~ 9700 3600
Text GLabel 9700 4800 2    50   Input ~ 0
UART_TX
Text GLabel 9700 4700 2    50   Input ~ 0
UART_RX
$Comp
L power:GND #PWR05
U 1 1 61C7DD14
P 9000 5000
F 0 "#PWR05" H 9000 4750 50  0001 C CNN
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
L power:VBUS #PWR09
U 1 1 61C7F092
P 9800 2950
F 0 "#PWR09" H 9800 2800 50  0001 C CNN
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
L power:+3V3 #PWR07
U 1 1 61C7F8C6
P 9700 3300
F 0 "#PWR07" H 9700 3150 50  0001 C CNN
F 1 "+3V3" V 9715 3428 50  0000 L CNN
F 2 "" H 9700 3300 50  0001 C CNN
F 3 "" H 9700 3300 50  0001 C CNN
	1    9700 3300
	0    1    1    0   
$EndComp
Text GLabel 8300 4700 0    50   Input ~ 0
I2C_SDA
Text GLabel 8300 4800 0    50   Input ~ 0
I2C_SCL
Text GLabel 8300 3500 0    50   Input ~ 0
ROW_0
Text GLabel 8300 3300 0    50   Input ~ 0
ROW_2
Text GLabel 9700 3900 2    50   Input ~ 0
Reset
Text GLabel 8300 3400 0    50   Input ~ 0
ROW_1
Text GLabel 8300 3200 0    50   Input ~ 0
ROW_3
Text GLabel 8300 3000 0    50   Input ~ 0
ROW_4
Text GLabel 8300 2900 0    50   Input ~ 0
ROW_5
Text GLabel 8300 3700 0    50   Input ~ 0
COL_0
Text GLabel 8300 3800 0    50   Input ~ 0
COL_1
Text GLabel 8300 3900 0    50   Input ~ 0
COL_2
Text GLabel 8300 4000 0    50   Input ~ 0
COL_3
Text GLabel 8300 4200 0    50   Input ~ 0
COL_4
Text GLabel 8300 4300 0    50   Input ~ 0
COL_5
$Comp
L Switch:SW_Push SW37
U 1 1 61C871B6
P 9800 2100
F 0 "SW37" H 9800 2385 50  0000 C CNN
F 1 "SW_Push" H 9800 2294 50  0000 C CNN
F 2 "Button_Switch_THT:SW_PUSH_6mm" H 9800 2300 50  0001 C CNN
F 3 "~" H 9800 2300 50  0001 C CNN
	1    9800 2100
	1    0    0    -1  
$EndComp
Text GLabel 10000 2100 2    50   Input ~ 0
Reset
Text GLabel 1100 2000 1    50   Input ~ 0
COL_0
Text GLabel 1800 2000 1    50   Input ~ 0
COL_1
Text GLabel 2500 2000 1    50   Input ~ 0
COL_2
Text GLabel 3200 2000 1    50   Input ~ 0
COL_3
Text GLabel 3900 2000 1    50   Input ~ 0
COL_4
Text GLabel 4600 2000 1    50   Input ~ 0
COL_5
$Comp
L Switch:SW_Push SW1
U 1 1 61CA432E
P 1400 2500
F 0 "SW1" V 1354 2648 50  0000 L CNN
F 1 "SW_Push" V 1445 2648 50  0000 L CNN
F 2 "keyswitches:SW_PG1350" H 1400 2700 50  0001 C CNN
F 3 "~" H 1400 2700 50  0001 C CNN
	1    1400 2500
	0    1    1    0   
$EndComp
Text GLabel 5400 2700 2    50   Input ~ 0
ROW_0
$Comp
L Device:D_ALT D1
U 1 1 61CA4353
P 1250 2300
F 0 "D1" H 1250 2517 50  0000 C CNN
F 1 "D_ALT" H 1250 2426 50  0000 C CNN
F 2 "Diode_THT:D_DO-34_SOD68_P7.62mm_Horizontal" H 1250 2300 50  0001 C CNN
F 3 "~" H 1250 2300 50  0001 C CNN
	1    1250 2300
	1    0    0    -1  
$EndComp
$Comp
L Device:D_ALT D19
U 1 1 61CA4370
P 3350 2300
F 0 "D19" H 3350 2517 50  0000 C CNN
F 1 "D_ALT" H 3350 2426 50  0000 C CNN
F 2 "Diode_THT:D_DO-34_SOD68_P7.62mm_Horizontal" H 3350 2300 50  0001 C CNN
F 3 "~" H 3350 2300 50  0001 C CNN
	1    3350 2300
	1    0    0    -1  
$EndComp
$Comp
L Device:D_ALT D25
U 1 1 61CA4376
P 4050 2300
F 0 "D25" H 4050 2517 50  0000 C CNN
F 1 "D_ALT" H 4050 2426 50  0000 C CNN
F 2 "Diode_THT:D_DO-34_SOD68_P7.62mm_Horizontal" H 4050 2300 50  0001 C CNN
F 3 "~" H 4050 2300 50  0001 C CNN
	1    4050 2300
	1    0    0    -1  
$EndComp
$Comp
L Device:D_ALT D31
U 1 1 61CA437C
P 4750 2300
F 0 "D31" H 4750 2517 50  0000 C CNN
F 1 "D_ALT" H 4750 2426 50  0000 C CNN
F 2 "Diode_THT:D_DO-34_SOD68_P7.62mm_Horizontal" H 4750 2300 50  0001 C CNN
F 3 "~" H 4750 2300 50  0001 C CNN
	1    4750 2300
	1    0    0    -1  
$EndComp
Wire Wire Line
	1100 2000 1100 2300
Wire Wire Line
	3200 2000 3200 2300
Wire Wire Line
	3900 2000 3900 2300
Wire Wire Line
	4600 2000 4600 2300
$Comp
L Switch:SW_Push SW2
U 1 1 61CB00F0
P 1400 3150
F 0 "SW2" V 1354 3298 50  0000 L CNN
F 1 "SW_Push" V 1445 3298 50  0000 L CNN
F 2 "keyswitches:SW_PG1350" H 1400 3350 50  0001 C CNN
F 3 "~" H 1400 3350 50  0001 C CNN
	1    1400 3150
	0    1    1    0   
$EndComp
$Comp
L Switch:SW_Push SW8
U 1 1 61CB00F6
P 2100 3150
F 0 "SW8" V 2054 3298 50  0000 L CNN
F 1 "SW_Push" V 2145 3298 50  0000 L CNN
F 2 "keyswitches:SW_PG1350" H 2100 3350 50  0001 C CNN
F 3 "~" H 2100 3350 50  0001 C CNN
	1    2100 3150
	0    1    1    0   
$EndComp
$Comp
L Switch:SW_Push SW14
U 1 1 61CB00FC
P 2800 3150
F 0 "SW14" V 2754 3298 50  0000 L CNN
F 1 "SW_Push" V 2845 3298 50  0000 L CNN
F 2 "keyswitches:SW_PG1350" H 2800 3350 50  0001 C CNN
F 3 "~" H 2800 3350 50  0001 C CNN
	1    2800 3150
	0    1    1    0   
$EndComp
Text GLabel 5400 3350 2    50   Input ~ 0
ROW_1
$Comp
L Device:D_ALT D2
U 1 1 61CB0115
P 1250 2950
F 0 "D2" H 1250 3167 50  0000 C CNN
F 1 "D_ALT" H 1250 3076 50  0000 C CNN
F 2 "Diode_THT:D_DO-34_SOD68_P7.62mm_Horizontal" H 1250 2950 50  0001 C CNN
F 3 "~" H 1250 2950 50  0001 C CNN
	1    1250 2950
	1    0    0    -1  
$EndComp
Connection ~ 2100 3350
Connection ~ 2800 3350
Wire Wire Line
	2100 3350 2800 3350
Wire Wire Line
	1400 3350 2100 3350
$Comp
L Device:D_ALT D8
U 1 1 61CB0126
P 1950 2950
F 0 "D8" H 1950 3167 50  0000 C CNN
F 1 "D_ALT" H 1950 3076 50  0000 C CNN
F 2 "Diode_THT:D_DO-34_SOD68_P7.62mm_Horizontal" H 1950 2950 50  0001 C CNN
F 3 "~" H 1950 2950 50  0001 C CNN
	1    1950 2950
	1    0    0    -1  
$EndComp
$Comp
L Device:D_ALT D14
U 1 1 61CB012C
P 2650 2950
F 0 "D14" H 2650 3167 50  0000 C CNN
F 1 "D_ALT" H 2650 3076 50  0000 C CNN
F 2 "Diode_THT:D_DO-34_SOD68_P7.62mm_Horizontal" H 2650 2950 50  0001 C CNN
F 3 "~" H 2650 2950 50  0001 C CNN
	1    2650 2950
	1    0    0    -1  
$EndComp
$Comp
L Device:D_ALT D20
U 1 1 61CB0132
P 3350 2950
F 0 "D20" H 3350 3167 50  0000 C CNN
F 1 "D_ALT" H 3350 3076 50  0000 C CNN
F 2 "Diode_THT:D_DO-34_SOD68_P7.62mm_Horizontal" H 3350 2950 50  0001 C CNN
F 3 "~" H 3350 2950 50  0001 C CNN
	1    3350 2950
	1    0    0    -1  
$EndComp
$Comp
L Device:D_ALT D26
U 1 1 61CB0138
P 4050 2950
F 0 "D26" H 4050 3167 50  0000 C CNN
F 1 "D_ALT" H 4050 3076 50  0000 C CNN
F 2 "Diode_THT:D_DO-34_SOD68_P7.62mm_Horizontal" H 4050 2950 50  0001 C CNN
F 3 "~" H 4050 2950 50  0001 C CNN
	1    4050 2950
	1    0    0    -1  
$EndComp
$Comp
L Device:D_ALT D32
U 1 1 61CB013E
P 4750 2950
F 0 "D32" H 4750 3167 50  0000 C CNN
F 1 "D_ALT" H 4750 3076 50  0000 C CNN
F 2 "Diode_THT:D_DO-34_SOD68_P7.62mm_Horizontal" H 4750 2950 50  0001 C CNN
F 3 "~" H 4750 2950 50  0001 C CNN
	1    4750 2950
	1    0    0    -1  
$EndComp
$Comp
L Switch:SW_Push SW3
U 1 1 61CB2B23
P 1400 3800
F 0 "SW3" V 1354 3948 50  0000 L CNN
F 1 "SW_Push" V 1445 3948 50  0000 L CNN
F 2 "keyswitches:SW_PG1350" H 1400 4000 50  0001 C CNN
F 3 "~" H 1400 4000 50  0001 C CNN
	1    1400 3800
	0    1    1    0   
$EndComp
$Comp
L Switch:SW_Push SW9
U 1 1 61CB2B29
P 2100 3800
F 0 "SW9" V 2054 3948 50  0000 L CNN
F 1 "SW_Push" V 2145 3948 50  0000 L CNN
F 2 "keyswitches:SW_PG1350" H 2100 4000 50  0001 C CNN
F 3 "~" H 2100 4000 50  0001 C CNN
	1    2100 3800
	0    1    1    0   
$EndComp
$Comp
L Switch:SW_Push SW15
U 1 1 61CB2B2F
P 2800 3800
F 0 "SW15" V 2754 3948 50  0000 L CNN
F 1 "SW_Push" V 2845 3948 50  0000 L CNN
F 2 "keyswitches:SW_PG1350" H 2800 4000 50  0001 C CNN
F 3 "~" H 2800 4000 50  0001 C CNN
	1    2800 3800
	0    1    1    0   
$EndComp
$Comp
L Switch:SW_Push SW21
U 1 1 61CB2B35
P 3500 3800
F 0 "SW21" V 3454 3948 50  0000 L CNN
F 1 "SW_Push" V 3545 3948 50  0000 L CNN
F 2 "keyswitches:SW_PG1350" H 3500 4000 50  0001 C CNN
F 3 "~" H 3500 4000 50  0001 C CNN
	1    3500 3800
	0    1    1    0   
$EndComp
$Comp
L Switch:SW_Push SW27
U 1 1 61CB2B3B
P 4200 3800
F 0 "SW27" V 4154 3948 50  0000 L CNN
F 1 "SW_Push" V 4245 3948 50  0000 L CNN
F 2 "keyswitches:SW_PG1350" H 4200 4000 50  0001 C CNN
F 3 "~" H 4200 4000 50  0001 C CNN
	1    4200 3800
	0    1    1    0   
$EndComp
$Comp
L Switch:SW_Push SW33
U 1 1 61CB2B41
P 4900 3800
F 0 "SW33" V 4854 3948 50  0000 L CNN
F 1 "SW_Push" V 4945 3948 50  0000 L CNN
F 2 "keyswitches:SW_PG1350" H 4900 4000 50  0001 C CNN
F 3 "~" H 4900 4000 50  0001 C CNN
	1    4900 3800
	0    1    1    0   
$EndComp
Text GLabel 5400 4000 2    50   Input ~ 0
ROW_2
$Comp
L Device:D_ALT D3
U 1 1 61CB2B48
P 1250 3600
F 0 "D3" H 1250 3817 50  0000 C CNN
F 1 "D_ALT" H 1250 3726 50  0000 C CNN
F 2 "Diode_THT:D_DO-34_SOD68_P7.62mm_Horizontal" H 1250 3600 50  0001 C CNN
F 3 "~" H 1250 3600 50  0001 C CNN
	1    1250 3600
	1    0    0    -1  
$EndComp
Connection ~ 2100 4000
Connection ~ 2800 4000
Connection ~ 3500 4000
Connection ~ 4200 4000
Connection ~ 4900 4000
Wire Wire Line
	2100 4000 2800 4000
Wire Wire Line
	2800 4000 3500 4000
Wire Wire Line
	3500 4000 4200 4000
Wire Wire Line
	4200 4000 4900 4000
Wire Wire Line
	4900 4000 5400 4000
Wire Wire Line
	1400 4000 2100 4000
$Comp
L Device:D_ALT D9
U 1 1 61CB2B59
P 1950 3600
F 0 "D9" H 1950 3817 50  0000 C CNN
F 1 "D_ALT" H 1950 3726 50  0000 C CNN
F 2 "Diode_THT:D_DO-34_SOD68_P7.62mm_Horizontal" H 1950 3600 50  0001 C CNN
F 3 "~" H 1950 3600 50  0001 C CNN
	1    1950 3600
	1    0    0    -1  
$EndComp
$Comp
L Device:D_ALT D15
U 1 1 61CB2B5F
P 2650 3600
F 0 "D15" H 2650 3817 50  0000 C CNN
F 1 "D_ALT" H 2650 3726 50  0000 C CNN
F 2 "Diode_THT:D_DO-34_SOD68_P7.62mm_Horizontal" H 2650 3600 50  0001 C CNN
F 3 "~" H 2650 3600 50  0001 C CNN
	1    2650 3600
	1    0    0    -1  
$EndComp
$Comp
L Device:D_ALT D21
U 1 1 61CB2B65
P 3350 3600
F 0 "D21" H 3350 3817 50  0000 C CNN
F 1 "D_ALT" H 3350 3726 50  0000 C CNN
F 2 "Diode_THT:D_DO-34_SOD68_P7.62mm_Horizontal" H 3350 3600 50  0001 C CNN
F 3 "~" H 3350 3600 50  0001 C CNN
	1    3350 3600
	1    0    0    -1  
$EndComp
$Comp
L Device:D_ALT D27
U 1 1 61CB2B6B
P 4050 3600
F 0 "D27" H 4050 3817 50  0000 C CNN
F 1 "D_ALT" H 4050 3726 50  0000 C CNN
F 2 "Diode_THT:D_DO-34_SOD68_P7.62mm_Horizontal" H 4050 3600 50  0001 C CNN
F 3 "~" H 4050 3600 50  0001 C CNN
	1    4050 3600
	1    0    0    -1  
$EndComp
$Comp
L Device:D_ALT D33
U 1 1 61CB2B71
P 4750 3600
F 0 "D33" H 4750 3817 50  0000 C CNN
F 1 "D_ALT" H 4750 3726 50  0000 C CNN
F 2 "Diode_THT:D_DO-34_SOD68_P7.62mm_Horizontal" H 4750 3600 50  0001 C CNN
F 3 "~" H 4750 3600 50  0001 C CNN
	1    4750 3600
	1    0    0    -1  
$EndComp
$Comp
L Switch:SW_Push SW4
U 1 1 61CB5869
P 1400 4450
F 0 "SW4" V 1354 4598 50  0000 L CNN
F 1 "SW_Push" V 1445 4598 50  0000 L CNN
F 2 "keyswitches:SW_PG1350" H 1400 4650 50  0001 C CNN
F 3 "~" H 1400 4650 50  0001 C CNN
	1    1400 4450
	0    1    1    0   
$EndComp
$Comp
L Switch:SW_Push SW10
U 1 1 61CB586F
P 2100 4450
F 0 "SW10" V 2054 4598 50  0000 L CNN
F 1 "SW_Push" V 2145 4598 50  0000 L CNN
F 2 "keyswitches:SW_PG1350" H 2100 4650 50  0001 C CNN
F 3 "~" H 2100 4650 50  0001 C CNN
	1    2100 4450
	0    1    1    0   
$EndComp
$Comp
L Switch:SW_Push SW16
U 1 1 61CB5875
P 2800 4450
F 0 "SW16" V 2754 4598 50  0000 L CNN
F 1 "SW_Push" V 2845 4598 50  0000 L CNN
F 2 "keyswitches:SW_PG1350" H 2800 4650 50  0001 C CNN
F 3 "~" H 2800 4650 50  0001 C CNN
	1    2800 4450
	0    1    1    0   
$EndComp
$Comp
L Switch:SW_Push SW22
U 1 1 61CB587B
P 3500 4450
F 0 "SW22" V 3454 4598 50  0000 L CNN
F 1 "SW_Push" V 3545 4598 50  0000 L CNN
F 2 "keyswitches:SW_PG1350" H 3500 4650 50  0001 C CNN
F 3 "~" H 3500 4650 50  0001 C CNN
	1    3500 4450
	0    1    1    0   
$EndComp
$Comp
L Switch:SW_Push SW28
U 1 1 61CB5881
P 4200 4450
F 0 "SW28" V 4154 4598 50  0000 L CNN
F 1 "SW_Push" V 4245 4598 50  0000 L CNN
F 2 "keyswitches:SW_PG1350" H 4200 4650 50  0001 C CNN
F 3 "~" H 4200 4650 50  0001 C CNN
	1    4200 4450
	0    1    1    0   
$EndComp
$Comp
L Switch:SW_Push SW34
U 1 1 61CB5887
P 4900 4450
F 0 "SW34" V 4854 4598 50  0000 L CNN
F 1 "SW_Push" V 4945 4598 50  0000 L CNN
F 2 "keyswitches:SW_PG1350" H 4900 4650 50  0001 C CNN
F 3 "~" H 4900 4650 50  0001 C CNN
	1    4900 4450
	0    1    1    0   
$EndComp
Text GLabel 5400 4650 2    50   Input ~ 0
ROW_3
$Comp
L Device:D_ALT D4
U 1 1 61CB588E
P 1250 4250
F 0 "D4" H 1250 4467 50  0000 C CNN
F 1 "D_ALT" H 1250 4376 50  0000 C CNN
F 2 "Diode_THT:D_DO-34_SOD68_P7.62mm_Horizontal" H 1250 4250 50  0001 C CNN
F 3 "~" H 1250 4250 50  0001 C CNN
	1    1250 4250
	1    0    0    -1  
$EndComp
Connection ~ 2100 4650
Connection ~ 2800 4650
Connection ~ 3500 4650
Connection ~ 4200 4650
Connection ~ 4900 4650
Wire Wire Line
	2100 4650 2800 4650
Wire Wire Line
	2800 4650 3500 4650
Wire Wire Line
	3500 4650 4200 4650
Wire Wire Line
	4200 4650 4900 4650
Wire Wire Line
	4900 4650 5400 4650
Wire Wire Line
	1400 4650 2100 4650
$Comp
L Device:D_ALT D10
U 1 1 61CB589F
P 1950 4250
F 0 "D10" H 1950 4467 50  0000 C CNN
F 1 "D_ALT" H 1950 4376 50  0000 C CNN
F 2 "Diode_THT:D_DO-34_SOD68_P7.62mm_Horizontal" H 1950 4250 50  0001 C CNN
F 3 "~" H 1950 4250 50  0001 C CNN
	1    1950 4250
	1    0    0    -1  
$EndComp
$Comp
L Device:D_ALT D16
U 1 1 61CB58A5
P 2650 4250
F 0 "D16" H 2650 4467 50  0000 C CNN
F 1 "D_ALT" H 2650 4376 50  0000 C CNN
F 2 "Diode_THT:D_DO-34_SOD68_P7.62mm_Horizontal" H 2650 4250 50  0001 C CNN
F 3 "~" H 2650 4250 50  0001 C CNN
	1    2650 4250
	1    0    0    -1  
$EndComp
$Comp
L Device:D_ALT D22
U 1 1 61CB58AB
P 3350 4250
F 0 "D22" H 3350 4467 50  0000 C CNN
F 1 "D_ALT" H 3350 4376 50  0000 C CNN
F 2 "Diode_THT:D_DO-34_SOD68_P7.62mm_Horizontal" H 3350 4250 50  0001 C CNN
F 3 "~" H 3350 4250 50  0001 C CNN
	1    3350 4250
	1    0    0    -1  
$EndComp
$Comp
L Device:D_ALT D28
U 1 1 61CB58B1
P 4050 4250
F 0 "D28" H 4050 4467 50  0000 C CNN
F 1 "D_ALT" H 4050 4376 50  0000 C CNN
F 2 "Diode_THT:D_DO-34_SOD68_P7.62mm_Horizontal" H 4050 4250 50  0001 C CNN
F 3 "~" H 4050 4250 50  0001 C CNN
	1    4050 4250
	1    0    0    -1  
$EndComp
$Comp
L Device:D_ALT D34
U 1 1 61CB58B7
P 4750 4250
F 0 "D34" H 4750 4467 50  0000 C CNN
F 1 "D_ALT" H 4750 4376 50  0000 C CNN
F 2 "Diode_THT:D_DO-34_SOD68_P7.62mm_Horizontal" H 4750 4250 50  0001 C CNN
F 3 "~" H 4750 4250 50  0001 C CNN
	1    4750 4250
	1    0    0    -1  
$EndComp
$Comp
L Switch:SW_Push SW5
U 1 1 61CBF1AE
P 1400 5100
F 0 "SW5" V 1354 5248 50  0000 L CNN
F 1 "SW_Push" V 1445 5248 50  0000 L CNN
F 2 "keyswitches:SW_PG1350" H 1400 5300 50  0001 C CNN
F 3 "~" H 1400 5300 50  0001 C CNN
	1    1400 5100
	0    1    1    0   
$EndComp
$Comp
L Switch:SW_Push SW11
U 1 1 61CBF1B4
P 2100 5100
F 0 "SW11" V 2054 5248 50  0000 L CNN
F 1 "SW_Push" V 2145 5248 50  0000 L CNN
F 2 "keyswitches:SW_PG1350" H 2100 5300 50  0001 C CNN
F 3 "~" H 2100 5300 50  0001 C CNN
	1    2100 5100
	0    1    1    0   
$EndComp
$Comp
L Switch:SW_Push SW17
U 1 1 61CBF1BA
P 2800 5100
F 0 "SW17" V 2754 5248 50  0000 L CNN
F 1 "SW_Push" V 2845 5248 50  0000 L CNN
F 2 "keyswitches:SW_PG1350" H 2800 5300 50  0001 C CNN
F 3 "~" H 2800 5300 50  0001 C CNN
	1    2800 5100
	0    1    1    0   
$EndComp
$Comp
L Switch:SW_Push SW23
U 1 1 61CBF1C0
P 3500 5100
F 0 "SW23" V 3454 5248 50  0000 L CNN
F 1 "SW_Push" V 3545 5248 50  0000 L CNN
F 2 "keyswitches:SW_PG1350" H 3500 5300 50  0001 C CNN
F 3 "~" H 3500 5300 50  0001 C CNN
	1    3500 5100
	0    1    1    0   
$EndComp
$Comp
L Switch:SW_Push SW29
U 1 1 61CBF1C6
P 4200 5100
F 0 "SW29" V 4154 5248 50  0000 L CNN
F 1 "SW_Push" V 4245 5248 50  0000 L CNN
F 2 "keyswitches:SW_PG1350" H 4200 5300 50  0001 C CNN
F 3 "~" H 4200 5300 50  0001 C CNN
	1    4200 5100
	0    1    1    0   
$EndComp
$Comp
L Switch:SW_Push SW35
U 1 1 61CBF1CC
P 4900 5100
F 0 "SW35" V 4854 5248 50  0000 L CNN
F 1 "SW_Push" V 4945 5248 50  0000 L CNN
F 2 "keyswitches:SW_PG1350" H 4900 5300 50  0001 C CNN
F 3 "~" H 4900 5300 50  0001 C CNN
	1    4900 5100
	0    1    1    0   
$EndComp
Text GLabel 5400 5300 2    50   Input ~ 0
ROW_4
$Comp
L Device:D_ALT D5
U 1 1 61CBF1D3
P 1250 4900
F 0 "D5" H 1250 5117 50  0000 C CNN
F 1 "D_ALT" H 1250 5026 50  0000 C CNN
F 2 "Diode_THT:D_DO-34_SOD68_P7.62mm_Horizontal" H 1250 4900 50  0001 C CNN
F 3 "~" H 1250 4900 50  0001 C CNN
	1    1250 4900
	1    0    0    -1  
$EndComp
Connection ~ 2100 5300
Connection ~ 2800 5300
Connection ~ 3500 5300
Connection ~ 4200 5300
Connection ~ 4900 5300
Wire Wire Line
	2100 5300 2800 5300
Wire Wire Line
	2800 5300 3500 5300
Wire Wire Line
	3500 5300 4200 5300
Wire Wire Line
	4200 5300 4900 5300
Wire Wire Line
	4900 5300 5400 5300
Wire Wire Line
	1400 5300 2100 5300
$Comp
L Device:D_ALT D11
U 1 1 61CBF1E4
P 1950 4900
F 0 "D11" H 1950 5117 50  0000 C CNN
F 1 "D_ALT" H 1950 5026 50  0000 C CNN
F 2 "Diode_THT:D_DO-34_SOD68_P7.62mm_Horizontal" H 1950 4900 50  0001 C CNN
F 3 "~" H 1950 4900 50  0001 C CNN
	1    1950 4900
	1    0    0    -1  
$EndComp
$Comp
L Device:D_ALT D17
U 1 1 61CBF1EA
P 2650 4900
F 0 "D17" H 2650 5117 50  0000 C CNN
F 1 "D_ALT" H 2650 5026 50  0000 C CNN
F 2 "Diode_THT:D_DO-34_SOD68_P7.62mm_Horizontal" H 2650 4900 50  0001 C CNN
F 3 "~" H 2650 4900 50  0001 C CNN
	1    2650 4900
	1    0    0    -1  
$EndComp
$Comp
L Device:D_ALT D23
U 1 1 61CBF1F0
P 3350 4900
F 0 "D23" H 3350 5117 50  0000 C CNN
F 1 "D_ALT" H 3350 5026 50  0000 C CNN
F 2 "Diode_THT:D_DO-34_SOD68_P7.62mm_Horizontal" H 3350 4900 50  0001 C CNN
F 3 "~" H 3350 4900 50  0001 C CNN
	1    3350 4900
	1    0    0    -1  
$EndComp
$Comp
L Device:D_ALT D29
U 1 1 61CBF1F6
P 4050 4900
F 0 "D29" H 4050 5117 50  0000 C CNN
F 1 "D_ALT" H 4050 5026 50  0000 C CNN
F 2 "Diode_THT:D_DO-34_SOD68_P7.62mm_Horizontal" H 4050 4900 50  0001 C CNN
F 3 "~" H 4050 4900 50  0001 C CNN
	1    4050 4900
	1    0    0    -1  
$EndComp
$Comp
L Device:D_ALT D35
U 1 1 61CBF1FC
P 4750 4900
F 0 "D35" H 4750 5117 50  0000 C CNN
F 1 "D_ALT" H 4750 5026 50  0000 C CNN
F 2 "Diode_THT:D_DO-34_SOD68_P7.62mm_Horizontal" H 4750 4900 50  0001 C CNN
F 3 "~" H 4750 4900 50  0001 C CNN
	1    4750 4900
	1    0    0    -1  
$EndComp
Wire Wire Line
	1100 4250 1100 4900
Connection ~ 1100 4250
Wire Wire Line
	1100 3600 1100 4250
Connection ~ 1100 3600
Wire Wire Line
	1100 2950 1100 3600
Connection ~ 1100 2950
Wire Wire Line
	1100 2300 1100 2950
Connection ~ 1100 2300
Connection ~ 1800 4250
Wire Wire Line
	1800 4250 1800 4900
Connection ~ 1800 2950
Wire Wire Line
	1800 2950 1800 3600
Connection ~ 1800 3600
Wire Wire Line
	1800 3600 1800 4250
Connection ~ 2500 2950
Wire Wire Line
	2500 2950 2500 3600
Connection ~ 2500 3600
Wire Wire Line
	2500 3600 2500 4250
Connection ~ 2500 4250
Wire Wire Line
	2500 4250 2500 4900
Wire Wire Line
	3200 2300 3200 2950
Connection ~ 3200 2300
Connection ~ 3200 2950
Wire Wire Line
	3200 2950 3200 3600
Connection ~ 3200 3600
Wire Wire Line
	3200 3600 3200 4250
Connection ~ 3200 4250
Wire Wire Line
	3200 4250 3200 4900
Wire Wire Line
	3900 2300 3900 2950
Connection ~ 3900 2300
Connection ~ 3900 2950
Wire Wire Line
	3900 2950 3900 3600
Connection ~ 3900 3600
Wire Wire Line
	3900 3600 3900 4250
Connection ~ 3900 4250
Wire Wire Line
	3900 4250 3900 4900
Wire Wire Line
	4600 2300 4600 2950
Connection ~ 4600 2300
Connection ~ 4600 2950
Wire Wire Line
	4600 2950 4600 3600
Connection ~ 4600 3600
Wire Wire Line
	4600 3600 4600 4250
Connection ~ 4600 4250
Wire Wire Line
	4600 4250 4600 4900
$Comp
L Switch:SW_Push SW6
U 1 1 61CDDA5C
P 1400 5700
F 0 "SW6" V 1354 5848 50  0000 L CNN
F 1 "SW_Push" V 1445 5848 50  0000 L CNN
F 2 "keyswitches:SW_PG1350" H 1400 5900 50  0001 C CNN
F 3 "~" H 1400 5900 50  0001 C CNN
	1    1400 5700
	0    1    1    0   
$EndComp
$Comp
L Switch:SW_Push SW12
U 1 1 61CDDA62
P 2100 5700
F 0 "SW12" V 2054 5848 50  0000 L CNN
F 1 "SW_Push" V 2145 5848 50  0000 L CNN
F 2 "keyswitches:SW_PG1350" H 2100 5900 50  0001 C CNN
F 3 "~" H 2100 5900 50  0001 C CNN
	1    2100 5700
	0    1    1    0   
$EndComp
$Comp
L Switch:SW_Push SW18
U 1 1 61CDDA68
P 2800 5700
F 0 "SW18" V 2754 5848 50  0000 L CNN
F 1 "SW_Push" V 2845 5848 50  0000 L CNN
F 2 "keyswitches:SW_PG1350" H 2800 5900 50  0001 C CNN
F 3 "~" H 2800 5900 50  0001 C CNN
	1    2800 5700
	0    1    1    0   
$EndComp
$Comp
L Switch:SW_Push SW24
U 1 1 61CDDA6E
P 3500 5700
F 0 "SW24" V 3454 5848 50  0000 L CNN
F 1 "SW_Push" V 3545 5848 50  0000 L CNN
F 2 "keyswitches:SW_PG1350" H 3500 5900 50  0001 C CNN
F 3 "~" H 3500 5900 50  0001 C CNN
	1    3500 5700
	0    1    1    0   
$EndComp
$Comp
L Switch:SW_Push SW30
U 1 1 61CDDA74
P 4200 5700
F 0 "SW30" V 4154 5848 50  0000 L CNN
F 1 "SW_Push" V 4245 5848 50  0000 L CNN
F 2 "keyswitches:SW_PG1350" H 4200 5900 50  0001 C CNN
F 3 "~" H 4200 5900 50  0001 C CNN
	1    4200 5700
	0    1    1    0   
$EndComp
$Comp
L Switch:SW_Push SW36
U 1 1 61CDDA7A
P 4900 5700
F 0 "SW36" V 4854 5848 50  0000 L CNN
F 1 "SW_Push" V 4945 5848 50  0000 L CNN
F 2 "keyswitches:SW_PG1350" H 4900 5900 50  0001 C CNN
F 3 "~" H 4900 5900 50  0001 C CNN
	1    4900 5700
	0    1    1    0   
$EndComp
Text GLabel 5400 5900 2    50   Input ~ 0
ROW_5
$Comp
L Device:D_ALT D6
U 1 1 61CDDA81
P 1250 5500
F 0 "D6" H 1250 5717 50  0000 C CNN
F 1 "D_ALT" H 1250 5626 50  0000 C CNN
F 2 "Diode_THT:D_DO-34_SOD68_P7.62mm_Horizontal" H 1250 5500 50  0001 C CNN
F 3 "~" H 1250 5500 50  0001 C CNN
	1    1250 5500
	1    0    0    -1  
$EndComp
Connection ~ 2100 5900
Connection ~ 2800 5900
Connection ~ 3500 5900
Connection ~ 4200 5900
Connection ~ 4900 5900
Wire Wire Line
	2100 5900 2800 5900
Wire Wire Line
	2800 5900 3500 5900
Wire Wire Line
	3500 5900 4200 5900
Wire Wire Line
	4200 5900 4900 5900
Wire Wire Line
	4900 5900 5400 5900
Wire Wire Line
	1400 5900 2100 5900
$Comp
L Device:D_ALT D12
U 1 1 61CDDA92
P 1950 5500
F 0 "D12" H 1950 5717 50  0000 C CNN
F 1 "D_ALT" H 1950 5626 50  0000 C CNN
F 2 "Diode_THT:D_DO-34_SOD68_P7.62mm_Horizontal" H 1950 5500 50  0001 C CNN
F 3 "~" H 1950 5500 50  0001 C CNN
	1    1950 5500
	1    0    0    -1  
$EndComp
$Comp
L Device:D_ALT D18
U 1 1 61CDDA98
P 2650 5500
F 0 "D18" H 2650 5717 50  0000 C CNN
F 1 "D_ALT" H 2650 5626 50  0000 C CNN
F 2 "Diode_THT:D_DO-34_SOD68_P7.62mm_Horizontal" H 2650 5500 50  0001 C CNN
F 3 "~" H 2650 5500 50  0001 C CNN
	1    2650 5500
	1    0    0    -1  
$EndComp
$Comp
L Device:D_ALT D24
U 1 1 61CDDA9E
P 3350 5500
F 0 "D24" H 3350 5717 50  0000 C CNN
F 1 "D_ALT" H 3350 5626 50  0000 C CNN
F 2 "Diode_THT:D_DO-34_SOD68_P7.62mm_Horizontal" H 3350 5500 50  0001 C CNN
F 3 "~" H 3350 5500 50  0001 C CNN
	1    3350 5500
	1    0    0    -1  
$EndComp
$Comp
L Device:D_ALT D30
U 1 1 61CDDAA4
P 4050 5500
F 0 "D30" H 4050 5717 50  0000 C CNN
F 1 "D_ALT" H 4050 5626 50  0000 C CNN
F 2 "Diode_THT:D_DO-34_SOD68_P7.62mm_Horizontal" H 4050 5500 50  0001 C CNN
F 3 "~" H 4050 5500 50  0001 C CNN
	1    4050 5500
	1    0    0    -1  
$EndComp
$Comp
L Device:D_ALT D36
U 1 1 61CDDAAA
P 4750 5500
F 0 "D36" H 4750 5717 50  0000 C CNN
F 1 "D_ALT" H 4750 5626 50  0000 C CNN
F 2 "Diode_THT:D_DO-34_SOD68_P7.62mm_Horizontal" H 4750 5500 50  0001 C CNN
F 3 "~" H 4750 5500 50  0001 C CNN
	1    4750 5500
	1    0    0    -1  
$EndComp
Wire Wire Line
	1100 4900 1100 5500
Connection ~ 1100 4900
Wire Wire Line
	1800 4900 1800 5500
Connection ~ 1800 4900
Wire Wire Line
	2500 4900 2500 5500
Connection ~ 2500 4900
Wire Wire Line
	3200 4900 3200 5500
Connection ~ 3200 4900
Wire Wire Line
	3900 4900 3900 5500
Connection ~ 3900 4900
Wire Wire Line
	4600 4900 4600 5500
Connection ~ 4600 4900
$Comp
L Connector_Generic:Conn_01x04 e1
U 1 1 61CF213E
P 8300 2000
F 0 "e1" H 8380 1992 50  0000 L CNN
F 1 "I2C OLED" H 8380 1901 50  0000 L CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_1x04_P2.54mm_Vertical" H 8300 2000 50  0001 C CNN
F 3 "~" H 8300 2000 50  0001 C CNN
	1    8300 2000
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR02
U 1 1 61CF3E48
P 8100 1900
F 0 "#PWR02" H 8100 1650 50  0001 C CNN
F 1 "GND" V 8105 1772 50  0000 R CNN
F 2 "" H 8100 1900 50  0001 C CNN
F 3 "" H 8100 1900 50  0001 C CNN
	1    8100 1900
	0    1    1    0   
$EndComp
$Comp
L power:+3V3 #PWR01
U 1 1 61CF4238
P 8100 2000
F 0 "#PWR01" H 8100 1850 50  0001 C CNN
F 1 "+3V3" V 8100 2150 50  0000 L CNN
F 2 "" H 8100 2000 50  0001 C CNN
F 3 "" H 8100 2000 50  0001 C CNN
	1    8100 2000
	0    -1   -1   0   
$EndComp
Text GLabel 8100 2200 0    50   Input ~ 0
I2C_SDA
Text GLabel 8100 2100 0    50   Input ~ 0
I2C_SCL
$Comp
L Connector:AudioJack4 J1
U 1 1 61CF6C8E
P 8050 1250
F 0 "J1" H 8007 1575 50  0000 C CNN
F 1 "AudioJack4" H 8007 1484 50  0000 C CNN
F 2 "Connector_Audio:Jack_3.5mm_PJ320E_Horizontal" H 8050 1250 50  0001 C CNN
F 3 "~" H 8050 1250 50  0001 C CNN
	1    8050 1250
	1    0    0    -1  
$EndComp
$Comp
L power:VBUS #PWR04
U 1 1 61CF862D
P 8250 1450
F 0 "#PWR04" H 8250 1300 50  0001 C CNN
F 1 "VBUS" V 8265 1578 50  0000 L CNN
F 2 "" H 8250 1450 50  0001 C CNN
F 3 "" H 8250 1450 50  0001 C CNN
	1    8250 1450
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR03
U 1 1 61CF9D64
P 8250 1150
F 0 "#PWR03" H 8250 900 50  0001 C CNN
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
NoConn ~ 9700 4400
NoConn ~ 9700 4300
NoConn ~ 9700 4200
NoConn ~ 9700 4000
NoConn ~ 9700 3800
NoConn ~ 9700 3700
NoConn ~ 9700 3500
Wire Wire Line
	1800 2000 1800 2950
Wire Wire Line
	2500 2000 2500 2950
$Comp
L Device:Rotary_Encoder_Switch SW38
U 1 1 61D11DCE
P 7100 2950
F 0 "SW38" H 7100 3317 50  0000 C CNN
F 1 "Rotary_Encoder_Switch" H 7100 3226 50  0000 C CNN
F 2 "Rotary_Encoder:RotaryEncoder_Alps_EC12E-Switch_Vertical_H20mm" H 6950 3110 50  0001 C CNN
F 3 "~" H 7100 3210 50  0001 C CNN
	1    7100 2950
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0101
U 1 1 61D25AB2
P 9600 2100
F 0 "#PWR0101" H 9600 1850 50  0001 C CNN
F 1 "GND" V 9605 1972 50  0000 R CNN
F 2 "" H 9600 2100 50  0001 C CNN
F 3 "" H 9600 2100 50  0001 C CNN
	1    9600 2100
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR0102
U 1 1 61D272AC
P 6800 2950
F 0 "#PWR0102" H 6800 2700 50  0001 C CNN
F 1 "GND" V 6800 2800 50  0000 R CNN
F 2 "" H 6800 2950 50  0001 C CNN
F 3 "" H 6800 2950 50  0001 C CNN
	1    6800 2950
	0    1    1    0   
$EndComp
Text GLabel 8300 4400 0    50   Input ~ 0
ROT_A
Text GLabel 8300 4500 0    50   Input ~ 0
ROT_B
$Comp
L power:GND #PWR0103
U 1 1 61D2A748
P 7400 2850
F 0 "#PWR0103" H 7400 2600 50  0001 C CNN
F 1 "GND" V 7400 2700 50  0000 R CNN
F 2 "" H 7400 2850 50  0001 C CNN
F 3 "" H 7400 2850 50  0001 C CNN
	1    7400 2850
	0    -1   -1   0   
$EndComp
Text GLabel 6800 2850 0    50   Input ~ 0
ROT_A
Text GLabel 6800 3050 0    50   Input ~ 0
ROT_B
Text GLabel 7400 3050 2    50   Input ~ 0
ROT_SW
Text GLabel 9700 4500 2    50   Input ~ 0
ROT_SW
$Comp
L Switch:SW_Push_Dual_x2 SW32
U 1 1 61C33F3C
P 4900 3150
F 0 "SW32" V 4854 3298 50  0000 L CNN
F 1 "SW_Push_Dual_x2" V 4945 3298 50  0000 L CNN
F 2 "keyswitches:SW_PG1350_double_rotate" H 4900 3350 50  0001 C CNN
F 3 "~" H 4900 3350 50  0001 C CNN
	1    4900 3150
	0    1    1    0   
$EndComp
$Comp
L Switch:SW_Push_Dual_x2 SW32
U 2 1 61C34F41
P 4900 2500
F 0 "SW32" V 4854 2648 50  0000 L CNN
F 1 "SW_Push_Dual_x2" V 4945 2648 50  0000 L CNN
F 2 "keyswitches:SW_PG1350_double_rotate" H 4900 2700 50  0001 C CNN
F 3 "~" H 4900 2700 50  0001 C CNN
	2    4900 2500
	0    1    1    0   
$EndComp
Connection ~ 4900 2700
Wire Wire Line
	4900 2700 5400 2700
Connection ~ 4900 3350
Wire Wire Line
	4900 3350 5400 3350
Wire Wire Line
	1400 2700 3500 2700
Wire Wire Line
	2800 3350 3500 3350
$Comp
L Switch:SW_Push_Dual_x2 SW26
U 1 1 61C4159B
P 4200 3150
F 0 "SW26" V 4154 3298 50  0000 L CNN
F 1 "SW_Push_Dual_x2" V 4245 3298 50  0000 L CNN
F 2 "keyswitches:SW_PG1350_double_rotate" H 4200 3350 50  0001 C CNN
F 3 "~" H 4200 3350 50  0001 C CNN
	1    4200 3150
	0    1    1    0   
$EndComp
Connection ~ 4200 3350
Wire Wire Line
	4200 3350 4900 3350
$Comp
L Switch:SW_Push_Dual_x2 SW20
U 1 1 61C41CA2
P 3500 3150
F 0 "SW20" V 3454 3298 50  0000 L CNN
F 1 "SW_Push_Dual_x2" V 3545 3298 50  0000 L CNN
F 2 "keyswitches:SW_PG1350_double_rotate" H 3500 3350 50  0001 C CNN
F 3 "~" H 3500 3350 50  0001 C CNN
	1    3500 3150
	0    1    1    0   
$EndComp
Connection ~ 3500 3350
Wire Wire Line
	3500 3350 4200 3350
$Comp
L Switch:SW_Push_Dual_x2 SW26
U 2 1 61C42512
P 4200 2500
F 0 "SW26" V 4154 2648 50  0000 L CNN
F 1 "SW_Push_Dual_x2" V 4245 2648 50  0000 L CNN
F 2 "keyswitches:SW_PG1350_double_rotate" H 4200 2700 50  0001 C CNN
F 3 "~" H 4200 2700 50  0001 C CNN
	2    4200 2500
	0    1    1    0   
$EndComp
Connection ~ 4200 2700
Wire Wire Line
	4200 2700 4900 2700
$Comp
L Switch:SW_Push_Dual_x2 SW20
U 2 1 61C42B7A
P 3500 2500
F 0 "SW20" V 3454 2648 50  0000 L CNN
F 1 "SW_Push_Dual_x2" V 3545 2648 50  0000 L CNN
F 2 "keyswitches:SW_PG1350_double_rotate" H 3500 2700 50  0001 C CNN
F 3 "~" H 3500 2700 50  0001 C CNN
	2    3500 2500
	0    1    1    0   
$EndComp
Connection ~ 3500 2700
Wire Wire Line
	3500 2700 4200 2700
$EndSCHEMATC
