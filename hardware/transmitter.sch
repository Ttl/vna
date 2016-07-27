EESchema Schematic File Version 2
LIBS:power
LIBS:device
LIBS:transistors
LIBS:conn
LIBS:linear
LIBS:regul
LIBS:74xx
LIBS:cmos4000
LIBS:adc-dac
LIBS:memory
LIBS:xilinx
LIBS:microcontrollers
LIBS:dsp
LIBS:microchip
LIBS:analog_switches
LIBS:motorola
LIBS:texas
LIBS:intel
LIBS:audio
LIBS:interface
LIBS:digital-audio
LIBS:philips
LIBS:display
LIBS:cypress
LIBS:siliconi
LIBS:opto
LIBS:atmel
LIBS:contrib
LIBS:valves
LIBS:vna
LIBS:vna-cache
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 4 8
Title "VNA"
Date "2016-07-27"
Rev "1"
Comp "Henrik Forstén"
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L MAX2870 U5
U 1 1 562E8D63
P 5300 3650
AR Path="/562E7B12/562E8D63" Ref="U5"  Part="1" 
AR Path="/562EEB8A/563156F1/562E8D63" Ref="U9"  Part="1" 
F 0 "U9" H 5300 3500 60  0000 C CNN
F 1 "MAX2871" H 5300 3950 60  0000 C CNN
F 2 "VNA:TQFN-32" H 3950 4500 60  0001 C CNN
F 3 "" H 3950 4500 60  0000 C CNN
	1    5300 3650
	1    0    0    -1  
$EndComp
$Comp
L C C19
U 1 1 562E8D6A
P 4150 4600
AR Path="/562E7B12/562E8D6A" Ref="C19"  Part="1" 
AR Path="/562EEB8A/563156F1/562E8D6A" Ref="C61"  Part="1" 
F 0 "C61" H 4175 4700 50  0000 L CNN
F 1 "1u" H 4175 4500 50  0000 L CNN
F 2 "VNA:C_0603b" H 4188 4450 30  0001 C CNN
F 3 "" H 4150 4600 60  0000 C CNN
	1    4150 4600
	1    0    0    -1  
$EndComp
$Comp
L C C17
U 1 1 562E8D71
P 3950 4600
AR Path="/562E7B12/562E8D71" Ref="C17"  Part="1" 
AR Path="/562EEB8A/563156F1/562E8D71" Ref="C59"  Part="1" 
F 0 "C59" H 3975 4700 50  0000 L CNN
F 1 "1u" H 3975 4500 50  0000 L CNN
F 2 "VNA:C_0603b" H 3988 4450 30  0001 C CNN
F 3 "" H 3950 4600 60  0000 C CNN
	1    3950 4600
	1    0    0    -1  
$EndComp
$Comp
L C C15
U 1 1 562E8D78
P 3750 4600
AR Path="/562E7B12/562E8D78" Ref="C15"  Part="1" 
AR Path="/562EEB8A/563156F1/562E8D78" Ref="C57"  Part="1" 
F 0 "C57" H 3775 4700 50  0000 L CNN
F 1 "1u" H 3775 4500 50  0000 L CNN
F 2 "VNA:C_0603b" H 3788 4450 30  0001 C CNN
F 3 "" H 3750 4600 60  0000 C CNN
	1    3750 4600
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR86
U 1 1 562E8D87
P 4950 4900
AR Path="/562E7B12/562E8D87" Ref="#PWR86"  Part="1" 
AR Path="/562EEB8A/563156F1/562E8D87" Ref="#PWR85"  Part="1" 
F 0 "#PWR85" H 4950 4650 50  0001 C CNN
F 1 "GND" H 4950 4750 50  0000 C CNN
F 2 "" H 4950 4900 60  0000 C CNN
F 3 "" H 4950 4900 60  0000 C CNN
	1    4950 4900
	1    0    0    -1  
$EndComp
$Comp
L C C26
U 1 1 562E8DA7
P 5800 2250
AR Path="/562E7B12/562E8DA7" Ref="C26"  Part="1" 
AR Path="/562EEB8A/563156F1/562E8DA7" Ref="C68"  Part="1" 
F 0 "C68" H 5825 2350 50  0000 L CNN
F 1 "100n" H 5825 2150 50  0000 L CNN
F 2 "VNA:C_0402b" H 5838 2100 30  0001 C CNN
F 3 "" H 5800 2250 60  0000 C CNN
	1    5800 2250
	1    0    0    -1  
$EndComp
$Comp
L R R11
U 1 1 562E8DAE
P 3550 4600
AR Path="/562E7B12/562E8DAE" Ref="R11"  Part="1" 
AR Path="/562EEB8A/563156F1/562E8DAE" Ref="R29"  Part="1" 
F 0 "R29" V 3630 4600 50  0000 C CNN
F 1 "5.1k" V 3550 4600 50  0000 C CNN
F 2 "VNA:R_0402b" V 3480 4600 30  0001 C CNN
F 3 "" H 3550 4600 30  0000 C CNN
	1    3550 4600
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR49
U 1 1 562E8DB5
P 3550 4800
AR Path="/562E7B12/562E8DB5" Ref="#PWR49"  Part="1" 
AR Path="/562EEB8A/563156F1/562E8DB5" Ref="#PWR48"  Part="1" 
F 0 "#PWR48" H 3550 4550 50  0001 C CNN
F 1 "GND" H 3550 4650 50  0000 C CNN
F 2 "" H 3550 4800 60  0000 C CNN
F 3 "" H 3550 4800 60  0000 C CNN
	1    3550 4800
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR52
U 1 1 562E8DBB
P 3750 4800
AR Path="/562E7B12/562E8DBB" Ref="#PWR52"  Part="1" 
AR Path="/562EEB8A/563156F1/562E8DBB" Ref="#PWR51"  Part="1" 
F 0 "#PWR51" H 3750 4550 50  0001 C CNN
F 1 "GND" H 3750 4650 50  0000 C CNN
F 2 "" H 3750 4800 60  0000 C CNN
F 3 "" H 3750 4800 60  0000 C CNN
	1    3750 4800
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR61
U 1 1 562E8DC1
P 3950 4800
AR Path="/562E7B12/562E8DC1" Ref="#PWR61"  Part="1" 
AR Path="/562EEB8A/563156F1/562E8DC1" Ref="#PWR62"  Part="1" 
F 0 "#PWR62" H 3950 4550 50  0001 C CNN
F 1 "GND" H 3950 4650 50  0000 C CNN
F 2 "" H 3950 4800 60  0000 C CNN
F 3 "" H 3950 4800 60  0000 C CNN
	1    3950 4800
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR66
U 1 1 562E8DC7
P 4150 4800
AR Path="/562E7B12/562E8DC7" Ref="#PWR66"  Part="1" 
AR Path="/562EEB8A/563156F1/562E8DC7" Ref="#PWR65"  Part="1" 
F 0 "#PWR65" H 4150 4550 50  0001 C CNN
F 1 "GND" H 4150 4650 50  0000 C CNN
F 2 "" H 4150 4800 60  0000 C CNN
F 3 "" H 4150 4800 60  0000 C CNN
	1    4150 4800
	1    0    0    -1  
$EndComp
$Comp
L C C32
U 1 1 562E8DD1
P 7650 3950
AR Path="/562E7B12/562E8DD1" Ref="C32"  Part="1" 
AR Path="/562EEB8A/563156F1/562E8DD1" Ref="C74"  Part="1" 
F 0 "C74" H 7675 4050 50  0000 L CNN
F 1 "820p" H 7675 3850 50  0000 L CNN
F 2 "VNA:C_0402b" H 7688 3800 30  0001 C CNN
F 3 "" H 7650 3950 60  0000 C CNN
	1    7650 3950
	1    0    0    -1  
$EndComp
$Comp
L R R18
U 1 1 562E8DD8
P 8000 3800
AR Path="/562E7B12/562E8DD8" Ref="R18"  Part="1" 
AR Path="/562EEB8A/563156F1/562E8DD8" Ref="R36"  Part="1" 
F 0 "R36" V 8080 3800 50  0000 C CNN
F 1 "240" V 8000 3800 50  0000 C CNN
F 2 "VNA:R_0402b" V 7930 3800 30  0001 C CNN
F 3 "" H 8000 3800 30  0000 C CNN
	1    8000 3800
	0    1    1    0   
$EndComp
$Comp
L GND #PWR149
U 1 1 562E8DE0
P 7650 4150
AR Path="/562E7B12/562E8DE0" Ref="#PWR149"  Part="1" 
AR Path="/562EEB8A/563156F1/562E8DE0" Ref="#PWR148"  Part="1" 
F 0 "#PWR148" H 7650 3900 50  0001 C CNN
F 1 "GND" H 7650 4000 50  0000 C CNN
F 2 "" H 7650 4150 60  0000 C CNN
F 3 "" H 7650 4150 60  0000 C CNN
	1    7650 4150
	1    0    0    -1  
$EndComp
$Comp
L C C31
U 1 1 562E8DE8
P 7650 3400
AR Path="/562E7B12/562E8DE8" Ref="C31"  Part="1" 
AR Path="/562EEB8A/563156F1/562E8DE8" Ref="C73"  Part="1" 
F 0 "C73" H 7675 3500 50  0000 L CNN
F 1 "12n" H 7675 3300 50  0000 L CNN
F 2 "VNA:C_0402b" H 7688 3250 30  0001 C CNN
F 3 "" H 7650 3400 60  0000 C CNN
	1    7650 3400
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR147
U 1 1 562E8DF0
P 7650 3550
AR Path="/562E7B12/562E8DF0" Ref="#PWR147"  Part="1" 
AR Path="/562EEB8A/563156F1/562E8DF0" Ref="#PWR146"  Part="1" 
F 0 "#PWR146" H 7650 3300 50  0001 C CNN
F 1 "GND" H 7650 3400 50  0000 C CNN
F 2 "" H 7650 3550 60  0000 C CNN
F 3 "" H 7650 3550 60  0000 C CNN
	1    7650 3550
	1    0    0    -1  
$EndComp
$Comp
L C C30
U 1 1 562E8DF6
P 7650 3050
AR Path="/562E7B12/562E8DF6" Ref="C30"  Part="1" 
AR Path="/562EEB8A/563156F1/562E8DF6" Ref="C72"  Part="1" 
F 0 "C72" H 7675 3150 50  0000 L CNN
F 1 "100n" H 7675 2950 50  0000 L CNN
F 2 "VNA:C_0402b" H 7688 2900 30  0001 C CNN
F 3 "" H 7650 3050 60  0000 C CNN
	1    7650 3050
	1    0    0    -1  
$EndComp
$Comp
L R R17
U 1 1 562E8DFE
P 7450 2900
AR Path="/562E7B12/562E8DFE" Ref="R17"  Part="1" 
AR Path="/562EEB8A/563156F1/562E8DFE" Ref="R35"  Part="1" 
F 0 "R35" V 7530 2900 50  0000 C CNN
F 1 "30.1" V 7450 2900 50  0000 C CNN
F 2 "VNA:R_0402b" V 7380 2900 30  0001 C CNN
F 3 "" H 7450 2900 30  0000 C CNN
	1    7450 2900
	0    1    1    0   
$EndComp
$Comp
L R R15
U 1 1 562E8E05
P 7250 3050
AR Path="/562E7B12/562E8E05" Ref="R15"  Part="1" 
AR Path="/562EEB8A/563156F1/562E8E05" Ref="R33"  Part="1" 
F 0 "R33" V 7330 3050 50  0000 C CNN
F 1 "90.9" V 7250 3050 50  0000 C CNN
F 2 "VNA:R_0402b" V 7180 3050 30  0001 C CNN
F 3 "" H 7250 3050 30  0000 C CNN
	1    7250 3050
	-1   0    0    1   
$EndComp
$Comp
L GND #PWR139
U 1 1 562E8E0D
P 7250 3200
AR Path="/562E7B12/562E8E0D" Ref="#PWR139"  Part="1" 
AR Path="/562EEB8A/563156F1/562E8E0D" Ref="#PWR138"  Part="1" 
F 0 "#PWR138" H 7250 2950 50  0001 C CNN
F 1 "GND" H 7250 3050 50  0000 C CNN
F 2 "" H 7250 3200 60  0000 C CNN
F 3 "" H 7250 3200 60  0000 C CNN
	1    7250 3200
	1    0    0    -1  
$EndComp
$Comp
L R R19
U 1 1 562E8E27
P 8850 1550
AR Path="/562E7B12/562E8E27" Ref="R19"  Part="1" 
AR Path="/562EEB8A/563156F1/562E8E27" Ref="R37"  Part="1" 
F 0 "R37" V 8930 1550 50  0000 C CNN
F 1 "49.9" V 8850 1550 50  0000 C CNN
F 2 "VNA:R_0402b" V 8780 1550 30  0001 C CNN
F 3 "" H 8850 1550 30  0000 C CNN
	1    8850 1550
	1    0    0    -1  
$EndComp
$Comp
L C C33
U 1 1 562E8E2E
P 8700 1300
AR Path="/562E7B12/562E8E2E" Ref="C33"  Part="1" 
AR Path="/562EEB8A/563156F1/562E8E2E" Ref="C75"  Part="1" 
F 0 "C75" H 8725 1400 50  0000 L CNN
F 1 "100p" H 8725 1200 50  0000 L CNN
F 2 "VNA:C_0402b" H 8738 1150 30  0001 C CNN
F 3 "" H 8700 1300 60  0000 C CNN
	1    8700 1300
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR160
U 1 1 562E8E36
P 8450 1300
AR Path="/562E7B12/562E8E36" Ref="#PWR160"  Part="1" 
AR Path="/562EEB8A/563156F1/562E8E36" Ref="#PWR159"  Part="1" 
F 0 "#PWR159" H 8450 1050 50  0001 C CNN
F 1 "GND" H 8450 1150 50  0000 C CNN
F 2 "" H 8450 1300 60  0000 C CNN
F 3 "" H 8450 1300 60  0000 C CNN
	1    8450 1300
	1    0    0    -1  
$EndComp
Text Label 6450 3850 0    60   ~ 0
RF_OUT_P
Text Label 8850 1850 2    60   ~ 0
RF_OUT_P
Text HLabel 9200 1850 2    60   Output ~ 0
RF_OUT_P
$Comp
L GND #PWR105
U 1 1 562E8E72
P 5800 2400
AR Path="/562E7B12/562E8E72" Ref="#PWR105"  Part="1" 
AR Path="/562EEB8A/563156F1/562E8E72" Ref="#PWR104"  Part="1" 
F 0 "#PWR104" H 5800 2150 50  0001 C CNN
F 1 "GND" H 5800 2250 50  0000 C CNN
F 2 "" H 5800 2400 60  0000 C CNN
F 3 "" H 5800 2400 60  0000 C CNN
	1    5800 2400
	1    0    0    -1  
$EndComp
Text HLabel 5500 1450 1    60   Input ~ 0
3V3
Text HLabel 3850 3200 0    60   Input ~ 0
CLK
Text HLabel 3850 3300 0    60   Input ~ 0
DATA
Text HLabel 3850 3500 0    60   Input ~ 0
CE
Text HLabel 3850 3400 0    60   Input ~ 0
LE
Text HLabel 3850 3650 0    60   Input ~ 0
RFOUT_EN
Text HLabel 6500 3000 2    60   Output ~ 0
MUXOUT
Text HLabel 6500 3100 2    60   Output ~ 0
LD
$Comp
L C C23
U 1 1 562EA29E
P 4650 1650
AR Path="/562E7B12/562EA29E" Ref="C23"  Part="1" 
AR Path="/562EEB8A/563156F1/562EA29E" Ref="C65"  Part="1" 
F 0 "C65" H 4675 1750 50  0000 L CNN
F 1 "100n" H 4675 1550 50  0000 L CNN
F 2 "VNA:C_0402b" H 4688 1500 30  0001 C CNN
F 3 "" H 4650 1650 60  0000 C CNN
	1    4650 1650
	1    0    0    -1  
$EndComp
$Comp
L C C25
U 1 1 562EA2EE
P 4850 1650
AR Path="/562E7B12/562EA2EE" Ref="C25"  Part="1" 
AR Path="/562EEB8A/563156F1/562EA2EE" Ref="C67"  Part="1" 
F 0 "C67" H 4875 1750 50  0000 L CNN
F 1 "100p" H 4875 1550 50  0000 L CNN
F 2 "VNA:C_0402b" H 4888 1500 30  0001 C CNN
F 3 "" H 4850 1650 60  0000 C CNN
	1    4850 1650
	1    0    0    -1  
$EndComp
$Comp
L C C22
U 1 1 562EA341
P 4450 2200
AR Path="/562E7B12/562EA341" Ref="C22"  Part="1" 
AR Path="/562EEB8A/563156F1/562EA341" Ref="C64"  Part="1" 
F 0 "C64" H 4475 2300 50  0000 L CNN
F 1 "100p" H 4475 2100 50  0000 L CNN
F 2 "VNA:C_0402b" H 4488 2050 30  0001 C CNN
F 3 "" H 4450 2200 60  0000 C CNN
	1    4450 2200
	1    0    0    -1  
$EndComp
$Comp
L C C24
U 1 1 562EA397
P 4700 2200
AR Path="/562E7B12/562EA397" Ref="C24"  Part="1" 
AR Path="/562EEB8A/563156F1/562EA397" Ref="C66"  Part="1" 
F 0 "C66" H 4725 2300 50  0000 L CNN
F 1 "100p" H 4725 2100 50  0000 L CNN
F 2 "VNA:C_0402b" H 4738 2050 30  0001 C CNN
F 3 "" H 4700 2200 60  0000 C CNN
	1    4700 2200
	1    0    0    -1  
$EndComp
$Comp
L C C27
U 1 1 562FFBC7
P 6100 2250
AR Path="/562E7B12/562FFBC7" Ref="C27"  Part="1" 
AR Path="/562EEB8A/563156F1/562FFBC7" Ref="C69"  Part="1" 
F 0 "C69" H 6125 2350 50  0000 L CNN
F 1 "100n" H 6125 2150 50  0000 L CNN
F 2 "VNA:C_0402b" H 6138 2100 30  0001 C CNN
F 3 "" H 6100 2250 60  0000 C CNN
	1    6100 2250
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR116
U 1 1 562FFDAC
P 6100 2400
AR Path="/562E7B12/562FFDAC" Ref="#PWR116"  Part="1" 
AR Path="/562EEB8A/563156F1/562FFDAC" Ref="#PWR117"  Part="1" 
F 0 "#PWR117" H 6100 2150 50  0001 C CNN
F 1 "GND" H 6100 2250 50  0000 C CNN
F 2 "" H 6100 2400 60  0000 C CNN
F 3 "" H 6100 2400 60  0000 C CNN
	1    6100 2400
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR79
U 1 1 563001F5
P 4700 2350
AR Path="/562E7B12/563001F5" Ref="#PWR79"  Part="1" 
AR Path="/562EEB8A/563156F1/563001F5" Ref="#PWR80"  Part="1" 
F 0 "#PWR80" H 4700 2100 50  0001 C CNN
F 1 "GND" H 4700 2200 50  0000 C CNN
F 2 "" H 4700 2350 60  0000 C CNN
F 3 "" H 4700 2350 60  0000 C CNN
	1    4700 2350
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR75
U 1 1 5630024B
P 4450 2350
AR Path="/562E7B12/5630024B" Ref="#PWR75"  Part="1" 
AR Path="/562EEB8A/563156F1/5630024B" Ref="#PWR76"  Part="1" 
F 0 "#PWR76" H 4450 2100 50  0001 C CNN
F 1 "GND" H 4450 2200 50  0000 C CNN
F 2 "" H 4450 2350 60  0000 C CNN
F 3 "" H 4450 2350 60  0000 C CNN
	1    4450 2350
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR82
U 1 1 5630076D
P 4850 1800
AR Path="/562E7B12/5630076D" Ref="#PWR82"  Part="1" 
AR Path="/562EEB8A/563156F1/5630076D" Ref="#PWR83"  Part="1" 
F 0 "#PWR83" H 4850 1550 50  0001 C CNN
F 1 "GND" H 4850 1650 50  0000 C CNN
F 2 "" H 4850 1800 60  0000 C CNN
F 3 "" H 4850 1800 60  0000 C CNN
	1    4850 1800
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR77
U 1 1 563007C0
P 4650 1800
AR Path="/562E7B12/563007C0" Ref="#PWR77"  Part="1" 
AR Path="/562EEB8A/563156F1/563007C0" Ref="#PWR78"  Part="1" 
F 0 "#PWR78" H 4650 1550 50  0001 C CNN
F 1 "GND" H 4650 1650 50  0000 C CNN
F 2 "" H 4650 1800 60  0000 C CNN
F 3 "" H 4650 1800 60  0000 C CNN
	1    4650 1800
	1    0    0    -1  
$EndComp
$Comp
L C C21
U 1 1 56300BCA
P 4400 1650
AR Path="/562E7B12/56300BCA" Ref="C21"  Part="1" 
AR Path="/562EEB8A/563156F1/56300BCA" Ref="C63"  Part="1" 
F 0 "C63" H 4425 1750 50  0000 L CNN
F 1 "100n" H 4425 1550 50  0000 L CNN
F 2 "VNA:C_0402b" H 4438 1500 30  0001 C CNN
F 3 "" H 4400 1650 60  0000 C CNN
	1    4400 1650
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR73
U 1 1 56300C2C
P 4400 1800
AR Path="/562E7B12/56300C2C" Ref="#PWR73"  Part="1" 
AR Path="/562EEB8A/563156F1/56300C2C" Ref="#PWR72"  Part="1" 
F 0 "#PWR72" H 4400 1550 50  0001 C CNN
F 1 "GND" H 4400 1650 50  0000 C CNN
F 2 "" H 4400 1800 60  0000 C CNN
F 3 "" H 4400 1800 60  0000 C CNN
	1    4400 1800
	1    0    0    -1  
$EndComp
$Comp
L C C20
U 1 1 56300C7F
P 4200 2200
AR Path="/562E7B12/56300C7F" Ref="C20"  Part="1" 
AR Path="/562EEB8A/563156F1/56300C7F" Ref="C62"  Part="1" 
F 0 "C62" H 4225 2300 50  0000 L CNN
F 1 "100n" H 4225 2100 50  0000 L CNN
F 2 "VNA:C_0402b" H 4238 2050 30  0001 C CNN
F 3 "" H 4200 2200 60  0000 C CNN
	1    4200 2200
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR69
U 1 1 56300D73
P 4200 2350
AR Path="/562E7B12/56300D73" Ref="#PWR69"  Part="1" 
AR Path="/562EEB8A/563156F1/56300D73" Ref="#PWR68"  Part="1" 
F 0 "#PWR68" H 4200 2100 50  0001 C CNN
F 1 "GND" H 4200 2200 50  0000 C CNN
F 2 "" H 4200 2350 60  0000 C CNN
F 3 "" H 4200 2350 60  0000 C CNN
	1    4200 2350
	1    0    0    -1  
$EndComp
$Comp
L C C16
U 1 1 563015C1
P 3950 2200
AR Path="/562E7B12/563015C1" Ref="C16"  Part="1" 
AR Path="/562EEB8A/563156F1/563015C1" Ref="C58"  Part="1" 
F 0 "C58" H 3975 2300 50  0000 L CNN
F 1 "100n" H 3975 2100 50  0000 L CNN
F 2 "VNA:C_0402b" H 3988 2050 30  0001 C CNN
F 3 "" H 3950 2200 60  0000 C CNN
	1    3950 2200
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR60
U 1 1 5630169B
P 3950 2350
AR Path="/562E7B12/5630169B" Ref="#PWR60"  Part="1" 
AR Path="/562EEB8A/563156F1/5630169B" Ref="#PWR59"  Part="1" 
F 0 "#PWR59" H 3950 2100 50  0001 C CNN
F 1 "GND" H 3950 2200 50  0000 C CNN
F 2 "" H 3950 2350 60  0000 C CNN
F 3 "" H 3950 2350 60  0000 C CNN
	1    3950 2350
	1    0    0    -1  
$EndComp
$Comp
L FILTER FB1
U 1 1 563022EE
P 3450 2050
AR Path="/562E7B12/563022EE" Ref="FB1"  Part="1" 
AR Path="/562EEB8A/563156F1/563022EE" Ref="FB3"  Part="1" 
F 0 "FB3" H 3450 2200 50  0000 C CNN
F 1 "BLM18PG181SN1D" H 3450 1950 50  0000 C CNN
F 2 "VNA:C_0603b" H 3450 2050 60  0001 C CNN
F 3 "" H 3450 2050 60  0000 C CNN
	1    3450 2050
	1    0    0    -1  
$EndComp
$Comp
L FILTER FB2
U 1 1 56302497
P 3950 1500
AR Path="/562E7B12/56302497" Ref="FB2"  Part="1" 
AR Path="/562EEB8A/563156F1/56302497" Ref="FB4"  Part="1" 
F 0 "FB4" H 3950 1650 50  0000 C CNN
F 1 "BLM18PG181SN1D" H 3950 1400 50  0000 C CNN
F 2 "VNA:C_0603b" H 3950 1500 60  0001 C CNN
F 3 "" H 3950 1500 60  0000 C CNN
	1    3950 1500
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR25
U 1 1 5630339E
P 2000 1900
AR Path="/562E7B12/5630339E" Ref="#PWR25"  Part="1" 
AR Path="/562EEB8A/563156F1/5630339E" Ref="#PWR24"  Part="1" 
F 0 "#PWR24" H 2000 1650 50  0001 C CNN
F 1 "GND" H 2000 1750 50  0000 C CNN
F 2 "" H 2000 1900 60  0000 C CNN
F 3 "" H 2000 1900 60  0000 C CNN
	1    2000 1900
	1    0    0    -1  
$EndComp
$Comp
L C C14
U 1 1 563036EA
P 2700 1700
AR Path="/562E7B12/563036EA" Ref="C14"  Part="1" 
AR Path="/562EEB8A/563156F1/563036EA" Ref="C56"  Part="1" 
F 0 "C56" H 2725 1800 50  0000 L CNN
F 1 "1u" H 2725 1600 50  0000 L CNN
F 2 "VNA:C_0603b" H 2738 1550 30  0001 C CNN
F 3 "" H 2700 1700 60  0000 C CNN
	1    2700 1700
	1    0    0    -1  
$EndComp
$Comp
L C C13
U 1 1 56303797
P 1350 1700
AR Path="/562E7B12/56303797" Ref="C13"  Part="1" 
AR Path="/562EEB8A/563156F1/56303797" Ref="C55"  Part="1" 
F 0 "C55" H 1375 1800 50  0000 L CNN
F 1 "1u" H 1375 1600 50  0000 L CNN
F 2 "VNA:C_0603b" H 1388 1550 30  0001 C CNN
F 3 "" H 1350 1700 60  0000 C CNN
	1    1350 1700
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR37
U 1 1 563039E8
P 2700 1850
AR Path="/562E7B12/563039E8" Ref="#PWR37"  Part="1" 
AR Path="/562EEB8A/563156F1/563039E8" Ref="#PWR36"  Part="1" 
F 0 "#PWR36" H 2700 1600 50  0001 C CNN
F 1 "GND" H 2700 1700 50  0000 C CNN
F 2 "" H 2700 1850 60  0000 C CNN
F 3 "" H 2700 1850 60  0000 C CNN
	1    2700 1850
	1    0    0    -1  
$EndComp
Text Notes 2700 1450 0    60   ~ 0
180mA
Text HLabel 1250 1500 0    60   Input ~ 0
3V6
$Comp
L GND #PWR10
U 1 1 5630432A
P 1350 1850
AR Path="/562E7B12/5630432A" Ref="#PWR10"  Part="1" 
AR Path="/562EEB8A/563156F1/5630432A" Ref="#PWR11"  Part="1" 
F 0 "#PWR11" H 1350 1600 50  0001 C CNN
F 1 "GND" H 1350 1700 50  0000 C CNN
F 2 "" H 1350 1850 60  0000 C CNN
F 3 "" H 1350 1850 60  0000 C CNN
	1    1350 1850
	1    0    0    -1  
$EndComp
Text HLabel 3400 3050 0    60   Input ~ 0
REF_IN
$Comp
L LP5907 U4
U 1 1 56306D73
P 2000 1550
AR Path="/562E7B12/56306D73" Ref="U4"  Part="1" 
AR Path="/562EEB8A/563156F1/56306D73" Ref="U8"  Part="1" 
F 0 "U8" H 2200 1350 60  0000 C CNN
F 1 "LP5907" H 2100 1750 60  0000 C CNN
F 2 "VNA:SOT-23-5" H 2000 1600 60  0001 C CNN
F 3 "" H 2000 1600 60  0000 C CNN
	1    2000 1550
	1    0    0    -1  
$EndComp
Text Label 6450 3950 0    60   ~ 0
RF_OUT_N
Text Label 8850 2600 2    60   ~ 0
RF_OUT_N
$Comp
L R R20
U 1 1 56312C62
P 8850 2350
AR Path="/562E7B12/56312C62" Ref="R20"  Part="1" 
AR Path="/562EEB8A/563156F1/56312C62" Ref="R38"  Part="1" 
F 0 "R38" V 8930 2350 50  0000 C CNN
F 1 "49.9" V 8850 2350 50  0000 C CNN
F 2 "VNA:R_0402b" V 8780 2350 30  0001 C CNN
F 3 "" H 8850 2350 30  0000 C CNN
	1    8850 2350
	1    0    0    -1  
$EndComp
$Comp
L C C34
U 1 1 56312DF7
P 8700 2200
AR Path="/562E7B12/56312DF7" Ref="C34"  Part="1" 
AR Path="/562EEB8A/563156F1/56312DF7" Ref="C76"  Part="1" 
F 0 "C76" H 8725 2300 50  0000 L CNN
F 1 "100p" H 8725 2100 50  0000 L CNN
F 2 "VNA:C_0402b" H 8738 2050 30  0001 C CNN
F 3 "" H 8700 2200 60  0000 C CNN
	1    8700 2200
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR161
U 1 1 56312EAA
P 8450 2200
AR Path="/562E7B12/56312EAA" Ref="#PWR161"  Part="1" 
AR Path="/562EEB8A/563156F1/56312EAA" Ref="#PWR162"  Part="1" 
F 0 "#PWR162" H 8450 1950 50  0001 C CNN
F 1 "GND" H 8450 2050 50  0000 C CNN
F 2 "" H 8450 2200 60  0000 C CNN
F 3 "" H 8450 2200 60  0000 C CNN
	1    8450 2200
	1    0    0    -1  
$EndComp
Text HLabel 8950 2600 2    60   Output ~ 0
RF_OUT_N
$Comp
L R R12
U 1 1 56352F1E
P 6700 4700
AR Path="/562E7B12/56352F1E" Ref="R12"  Part="1" 
AR Path="/562EEB8A/563156F1/56352F1E" Ref="R30"  Part="1" 
F 0 "R30" V 6780 4700 50  0000 C CNN
F 1 "0" V 6700 4700 50  0000 C CNN
F 2 "VNA:R_0402b" V 6630 4700 30  0001 C CNN
F 3 "" H 6700 4700 30  0000 C CNN
	1    6700 4700
	1    0    0    -1  
$EndComp
$Comp
L R R13
U 1 1 56352FA2
P 6900 4700
AR Path="/562E7B12/56352FA2" Ref="R13"  Part="1" 
AR Path="/562EEB8A/563156F1/56352FA2" Ref="R31"  Part="1" 
F 0 "R31" V 6980 4700 50  0000 C CNN
F 1 "0" V 6900 4700 50  0000 C CNN
F 2 "VNA:R_0402b" V 6830 4700 30  0001 C CNN
F 3 "" H 6900 4700 30  0000 C CNN
	1    6900 4700
	1    0    0    -1  
$EndComp
Wire Wire Line
	4300 4200 4150 4200
Wire Wire Line
	4150 4200 4150 4450
Wire Wire Line
	4300 4100 3950 4100
Wire Wire Line
	3950 4100 3950 4450
Wire Wire Line
	4300 4000 3750 4000
Wire Wire Line
	3750 4000 3750 4450
Wire Wire Line
	4300 3900 3550 3900
Wire Wire Line
	3550 3900 3550 4450
Wire Wire Line
	4950 4750 4950 4900
Wire Wire Line
	4950 4850 5650 4850
Wire Wire Line
	5050 4850 5050 4750
Connection ~ 4950 4850
Wire Wire Line
	5150 4850 5150 4750
Connection ~ 5050 4850
Wire Wire Line
	5250 4850 5250 4750
Connection ~ 5150 4850
Wire Wire Line
	5350 4850 5350 4750
Connection ~ 5250 4850
Wire Wire Line
	5450 4850 5450 4750
Connection ~ 5350 4850
Wire Wire Line
	5550 4850 5550 4750
Connection ~ 5450 4850
Wire Wire Line
	5650 4850 5650 4750
Connection ~ 5550 4850
Wire Wire Line
	4300 3050 4150 3050
Wire Wire Line
	3850 3050 3750 3050
Wire Wire Line
	4150 4750 4150 4800
Wire Wire Line
	3950 4750 3950 4800
Wire Wire Line
	3750 4750 3750 4800
Wire Wire Line
	3550 4750 3550 4800
Wire Wire Line
	7500 3800 7850 3800
Wire Wire Line
	7650 4100 7650 4150
Connection ~ 7650 3800
Wire Wire Line
	7500 3250 8150 3250
Wire Wire Line
	7650 3250 7650 3200
Connection ~ 7250 2900
Wire Wire Line
	7600 2900 7650 2900
Wire Wire Line
	7050 2900 7300 2900
Wire Wire Line
	7050 2900 7050 3300
Wire Wire Line
	7050 3300 6350 3300
Wire Wire Line
	6350 3500 7500 3500
Wire Wire Line
	7500 3500 7500 3250
Wire Wire Line
	7500 3800 7500 3600
Wire Wire Line
	7500 3600 6350 3600
Wire Wire Line
	8150 3250 8150 3800
Connection ~ 7650 3250
Wire Wire Line
	3850 3200 4300 3200
Wire Wire Line
	3850 3300 4300 3300
Wire Wire Line
	3850 3400 4300 3400
Wire Wire Line
	3850 3500 4300 3500
Wire Wire Line
	8850 1150 8850 1400
Wire Wire Line
	8550 1300 8450 1300
Wire Wire Line
	8850 1700 8850 1850
Wire Wire Line
	6350 3850 6450 3850
Wire Wire Line
	6350 3950 6450 3950
Wire Wire Line
	8850 1850 9200 1850
Connection ~ 8850 1300
Wire Wire Line
	3850 3650 4300 3650
Wire Wire Line
	6350 3000 6500 3000
Wire Wire Line
	6500 3100 6350 3100
Wire Wire Line
	5500 1450 5500 2550
Connection ~ 5500 1550
Wire Wire Line
	5400 2550 5400 2150
Wire Wire Line
	5600 2550 5600 2250
Wire Wire Line
	5600 2250 5500 2250
Connection ~ 5500 2250
Wire Wire Line
	6100 2000 6100 2100
Wire Wire Line
	5500 2000 6100 2000
Wire Wire Line
	5800 2000 5800 2100
Connection ~ 5500 2000
Connection ~ 5800 2000
Wire Wire Line
	5200 2350 5200 2550
Wire Wire Line
	4950 2350 5200 2350
Wire Wire Line
	5100 2350 5100 2550
Wire Wire Line
	4950 2050 4950 2350
Wire Wire Line
	3800 2050 4950 2050
Connection ~ 5100 2350
Connection ~ 4700 2050
Wire Wire Line
	5300 1500 5300 2550
Connection ~ 5300 2150
Wire Wire Line
	5400 2150 5300 2150
Wire Wire Line
	4300 1500 5300 1500
Connection ~ 4850 1500
Connection ~ 4650 1500
Connection ~ 4450 2050
Connection ~ 4200 2050
Connection ~ 4400 1500
Connection ~ 3950 2050
Wire Wire Line
	3100 2050 2950 2050
Wire Wire Line
	2500 1500 3600 1500
Wire Wire Line
	2950 2050 2950 1500
Connection ~ 2950 1500
Wire Wire Line
	1250 1500 1500 1500
Wire Wire Line
	2700 1500 2700 1550
Connection ~ 2700 1500
Wire Wire Line
	1350 1500 1350 1550
Connection ~ 1350 1500
Wire Wire Line
	1500 1500 1500 1600
Wire Wire Line
	8550 2200 8450 2200
Wire Wire Line
	8850 2500 8850 2600
Wire Wire Line
	8850 2600 8950 2600
Wire Wire Line
	6350 4100 6550 4100
Wire Wire Line
	6550 4100 6550 4950
Wire Wire Line
	6350 4200 6450 4200
Wire Wire Line
	6450 4200 6450 5050
Wire Wire Line
	6700 4950 6700 4850
Wire Wire Line
	6900 5050 6900 4850
Wire Wire Line
	6700 4550 6700 4450
Wire Wire Line
	6900 4550 6900 4450
Wire Wire Line
	6900 4450 6700 4450
Wire Wire Line
	6550 4950 6700 4950
Wire Wire Line
	6450 5050 6900 5050
$Comp
L C C18
U 1 1 562E8D9D
P 4000 3050
AR Path="/562E7B12/562E8D9D" Ref="C18"  Part="1" 
AR Path="/562EEB8A/563156F1/562E8D9D" Ref="C60"  Part="1" 
F 0 "C60" H 4025 3150 50  0000 L CNN
F 1 "10n" H 4025 2950 50  0000 L CNN
F 2 "VNA:C_0402b" H 4038 2900 30  0001 C CNN
F 3 "" H 4000 3050 60  0000 C CNN
	1    4000 3050
	0    1    1    0   
$EndComp
Text Label 3200 1500 0    60   ~ 0
3V3_RF
Text Label 8850 1150 0    60   ~ 0
3V3_RF
Text Label 8850 2150 0    60   ~ 0
3V3_RF
Wire Wire Line
	8850 2150 8850 2200
Text Label 6900 4450 0    60   ~ 0
3V3_RF
$Comp
L R R86
U 1 1 569FFD2D
P 3600 3050
AR Path="/562E7B12/569FFD2D" Ref="R86"  Part="1" 
AR Path="/562EEB8A/563156F1/569FFD2D" Ref="R87"  Part="1" 
F 0 "R87" V 3680 3050 50  0000 C CNN
F 1 "0" V 3600 3050 50  0000 C CNN
F 2 "VNA:R_0402b" V 3530 3050 30  0001 C CNN
F 3 "" H 3600 3050 30  0000 C CNN
	1    3600 3050
	0    1    1    0   
$EndComp
Wire Wire Line
	3450 3050 3400 3050
$EndSCHEMATC
