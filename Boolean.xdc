# CLOCK_100 input is from the 100 MHz oscillator on Boolean board
#create_clock -period 10.000 -name MAIN_CLK [get_ports CLOCK_100]
set_property -quiet -dict {PACKAGE_PIN F14 IOSTANDARD LVCMOS33} [get_ports {CLOCK_100}]

# Using BTN[3] as a clock, so need to override using dedicated clock routing
#set_property CLOCK_DEDICATED_ROUTE FALSE [get_nets BTN_IBUF[3]]

# Set Bank 0 voltage
set_property CFGBVS VCCO [current_design]
set_property CONFIG_VOLTAGE 3.3 [current_design]

# On-board Slide Switches
set_property -quiet -dict {PACKAGE_PIN V2 IOSTANDARD LVCMOS33} [get_ports {SW[0]}]
set_property -quiet -dict {PACKAGE_PIN U2 IOSTANDARD LVCMOS33} [get_ports {SW[1]}]
set_property -quiet -dict {PACKAGE_PIN U1 IOSTANDARD LVCMOS33} [get_ports {SW[2]}]
set_property -quiet -dict {PACKAGE_PIN T2 IOSTANDARD LVCMOS33} [get_ports {SW[3]}]
set_property -quiet -dict {PACKAGE_PIN T1 IOSTANDARD LVCMOS33} [get_ports {SW[4]}]
set_property -quiet -dict {PACKAGE_PIN R2 IOSTANDARD LVCMOS33} [get_ports {SW[5]}]
set_property -quiet -dict {PACKAGE_PIN R1 IOSTANDARD LVCMOS33} [get_ports {SW[6]}]
set_property -quiet -dict {PACKAGE_PIN P2 IOSTANDARD LVCMOS33} [get_ports {SW[7]}]
set_property -quiet -dict {PACKAGE_PIN P1 IOSTANDARD LVCMOS33} [get_ports {SW[8]}]
set_property -quiet -dict {PACKAGE_PIN N2 IOSTANDARD LVCMOS33} [get_ports {SW[9]}]
set_property -quiet -dict {PACKAGE_PIN N1 IOSTANDARD LVCMOS33} [get_ports {SW[10]}]
set_property -quiet -dict {PACKAGE_PIN M2 IOSTANDARD LVCMOS33} [get_ports {SW[11]}]
set_property -quiet -dict {PACKAGE_PIN M1 IOSTANDARD LVCMOS33} [get_ports {SW[12]}]
set_property -quiet -dict {PACKAGE_PIN L1 IOSTANDARD LVCMOS33} [get_ports {SW[13]}]
set_property -quiet -dict {PACKAGE_PIN K2 IOSTANDARD LVCMOS33} [get_ports {SW[14]}]
set_property -quiet -dict {PACKAGE_PIN K1 IOSTANDARD LVCMOS33} [get_ports {SW[15]}]

# On-board LEDs
set_property -quiet -dict {PACKAGE_PIN G1 IOSTANDARD LVCMOS33} [get_ports {LD[0]}]
set_property -quiet -dict {PACKAGE_PIN G2 IOSTANDARD LVCMOS33} [get_ports {LD[1]}]
set_property -quiet -dict {PACKAGE_PIN F1 IOSTANDARD LVCMOS33} [get_ports {LD[2]}]
set_property -quiet -dict {PACKAGE_PIN F2 IOSTANDARD LVCMOS33} [get_ports {LD[3]}]
set_property -quiet -dict {PACKAGE_PIN E1 IOSTANDARD LVCMOS33} [get_ports {LD[4]}]
set_property -quiet -dict {PACKAGE_PIN E2 IOSTANDARD LVCMOS33} [get_ports {LD[5]}]
set_property -quiet -dict {PACKAGE_PIN E3 IOSTANDARD LVCMOS33} [get_ports {LD[6]}]
set_property -quiet -dict {PACKAGE_PIN E5 IOSTANDARD LVCMOS33} [get_ports {LD[7]}]
set_property -quiet -dict {PACKAGE_PIN E6 IOSTANDARD LVCMOS33} [get_ports {LD[8]}]
set_property -quiet -dict {PACKAGE_PIN C3 IOSTANDARD LVCMOS33} [get_ports {LD[9]}]
set_property -quiet -dict {PACKAGE_PIN B2 IOSTANDARD LVCMOS33} [get_ports {LD[10]}]
set_property -quiet -dict {PACKAGE_PIN A2 IOSTANDARD LVCMOS33} [get_ports {LD[11]}]
set_property -quiet -dict {PACKAGE_PIN B3 IOSTANDARD LVCMOS33} [get_ports {LD[12]}]
set_property -quiet -dict {PACKAGE_PIN A3 IOSTANDARD LVCMOS33} [get_ports {LD[13]}]
set_property -quiet -dict {PACKAGE_PIN B4 IOSTANDARD LVCMOS33} [get_ports {LD[14]}]
set_property -quiet -dict {PACKAGE_PIN A4 IOSTANDARD LVCMOS33} [get_ports {LD[15]}]

# On-board Buttons
set_property -quiet -dict {PACKAGE_PIN J2 IOSTANDARD LVCMOS33} [get_ports {BTN[0]}]
set_property -quiet -dict {PACKAGE_PIN J5 IOSTANDARD LVCMOS33} [get_ports {BTN[1]}]
set_property -quiet -dict {PACKAGE_PIN H2 IOSTANDARD LVCMOS33} [get_ports {BTN[2]}]
set_property -quiet -dict {PACKAGE_PIN J1 IOSTANDARD LVCMOS33} [get_ports {BTN[3]}]

# On-board color LEDs
set_property -quiet -dict {PACKAGE_PIN V6 IOSTANDARD LVCMOS33} [get_ports {RGB0[0]}];   # RBG0_R
set_property -quiet -dict {PACKAGE_PIN V4 IOSTANDARD LVCMOS33} [get_ports {RGB0[1]}];   # RBG0_G
set_property -quiet -dict {PACKAGE_PIN U6 IOSTANDARD LVCMOS33} [get_ports {RGB0[2]}];   # RBG0_B
set_property -quiet -dict {PACKAGE_PIN U3 IOSTANDARD LVCMOS33} [get_ports {RGB1[0]}];   # RBG1_R
set_property -quiet -dict {PACKAGE_PIN V3 IOSTANDARD LVCMOS33} [get_ports {RGB1[1]}];   # RBG1_G
set_property -quiet -dict {PACKAGE_PIN V5 IOSTANDARD LVCMOS33} [get_ports {RGB1[2]}];   # RBG1_B

# On-board 7-Segment display 1
set_property -quiet -dict {PACKAGE_PIN D5 IOSTANDARD LVCMOS33} [get_ports {D1_AN[0]}]
set_property -quiet -dict {PACKAGE_PIN C4 IOSTANDARD LVCMOS33} [get_ports {D1_AN[1]}]
set_property -quiet -dict {PACKAGE_PIN C7 IOSTANDARD LVCMOS33} [get_ports {D1_AN[2]}]
set_property -quiet -dict {PACKAGE_PIN A8 IOSTANDARD LVCMOS33} [get_ports {D1_AN[3]}]
set_property -quiet -dict {PACKAGE_PIN D7 IOSTANDARD LVCMOS33} [get_ports {D1_SEG[0]}]
set_property -quiet -dict {PACKAGE_PIN C5 IOSTANDARD LVCMOS33} [get_ports {D1_SEG[1]}]
set_property -quiet -dict {PACKAGE_PIN A5 IOSTANDARD LVCMOS33} [get_ports {D1_SEG[2]}]
set_property -quiet -dict {PACKAGE_PIN B7 IOSTANDARD LVCMOS33} [get_ports {D1_SEG[3]}]
set_property -quiet -dict {PACKAGE_PIN A7 IOSTANDARD LVCMOS33} [get_ports {D1_SEG[4]}]
set_property -quiet -dict {PACKAGE_PIN D6 IOSTANDARD LVCMOS33} [get_ports {D1_SEG[5]}]
set_property -quiet -dict {PACKAGE_PIN B5 IOSTANDARD LVCMOS33} [get_ports {D1_SEG[6]}]
set_property -quiet -dict {PACKAGE_PIN A6 IOSTANDARD LVCMOS33} [get_ports {D1_SEG[7]}]

# On-board 7-Segment display 2
set_property -quiet -dict {PACKAGE_PIN H3 IOSTANDARD LVCMOS33} [get_ports {D2_AN[0]}]
set_property -quiet -dict {PACKAGE_PIN J4 IOSTANDARD LVCMOS33} [get_ports {D2_AN[1]}]
set_property -quiet -dict {PACKAGE_PIN F3 IOSTANDARD LVCMOS33} [get_ports {D2_AN[2]}]
set_property -quiet -dict {PACKAGE_PIN E4 IOSTANDARD LVCMOS33} [get_ports {D2_AN[3]}]
set_property -quiet -dict {PACKAGE_PIN F4 IOSTANDARD LVCMOS33} [get_ports {D2_SEG[0]}]
set_property -quiet -dict {PACKAGE_PIN J3 IOSTANDARD LVCMOS33} [get_ports {D2_SEG[1]}]
set_property -quiet -dict {PACKAGE_PIN D2 IOSTANDARD LVCMOS33} [get_ports {D2_SEG[2]}]
set_property -quiet -dict {PACKAGE_PIN C2 IOSTANDARD LVCMOS33} [get_ports {D2_SEG[3]}]
set_property -quiet -dict {PACKAGE_PIN B1 IOSTANDARD LVCMOS33} [get_ports {D2_SEG[4]}]
set_property -quiet -dict {PACKAGE_PIN H4 IOSTANDARD LVCMOS33} [get_ports {D2_SEG[5]}]
set_property -quiet -dict {PACKAGE_PIN D1 IOSTANDARD LVCMOS33} [get_ports {D2_SEG[6]}]
set_property -quiet -dict {PACKAGE_PIN C1 IOSTANDARD LVCMOS33} [get_ports {D2_SEG[7]}]

# UART
set_property -quiet -dict {PACKAGE_PIN V12 IOSTANDARD LVCMOS33} [get_ports {UART_RXD}]
set_property -quiet -dict {PACKAGE_PIN U11 IOSTANDARD LVCMOS33} [get_ports {UART_TXD}]

#HDMI Signals
set_property -quiet -dict { PACKAGE_PIN T14   IOSTANDARD TMDS_33 } [get_ports {HDMI_CLOCK_N}]
set_property -quiet -dict { PACKAGE_PIN R14   IOSTANDARD TMDS_33 } [get_ports {HDMI_CLOCK_P}]

set_property -quiet -dict { PACKAGE_PIN T15   IOSTANDARD TMDS_33  } [get_ports {HDMI_TX_N[0]}]
set_property -quiet -dict { PACKAGE_PIN R17   IOSTANDARD TMDS_33  } [get_ports {HDMI_TX_N[1]}]
set_property -quiet -dict { PACKAGE_PIN P16   IOSTANDARD TMDS_33  } [get_ports {HDMI_TX_N[2]}]

set_property -quiet -dict { PACKAGE_PIN R15   IOSTANDARD TMDS_33  } [get_ports {HDMI_TX_P[0]}]
set_property -quiet -dict { PACKAGE_PIN R16   IOSTANDARD TMDS_33  } [get_ports {HDMI_TX_P[1]}]
set_property -quiet -dict { PACKAGE_PIN N15   IOSTANDARD TMDS_33  } [get_ports {HDMI_TX_P[2]}]

# PWM audio signals
set_property -quiet -dict {PACKAGE_PIN N13 IOSTANDARD LVCMOS33} [get_ports {left_audio_out}]
set_property -quiet -dict {PACKAGE_PIN N14 IOSTANDARD LVCMOS33} [get_ports {right_audio_out}]

# BLE UART signals
set_property -quiet -dict {PACKAGE_PIN G5 IOSTANDARD LVCMOS33} [get_ports {BLE_UART_TX}]
set_property -quiet -dict {PACKAGE_PIN F5 IOSTANDARD LVCMOS33} [get_ports {BLE_UART_RX}]
set_property -quiet -dict {PACKAGE_PIN H6 IOSTANDARD LVCMOS33} [get_ports {BLE_UART_RTS}]
set_property -quiet -dict {PACKAGE_PIN G6 IOSTANDARD LVCMOS33} [get_ports {BLE_UART_CTS}]

# Servomotor signals
set_property -quiet -dict {PACKAGE_PIN M14 IOSTANDARD LVCMOS33} [get_ports {SERVO0}]
set_property -quiet -dict {PACKAGE_PIN M16 IOSTANDARD LVCMOS33} [get_ports {SERVO1}]
set_property -quiet -dict {PACKAGE_PIN L15 IOSTANDARD LVCMOS33} [get_ports {SERVO2}]
set_property -quiet -dict {PACKAGE_PIN L16 IOSTANDARD LVCMOS33} [get_ports {SERVO3}]

# GPIO0
set_property -dict {PACKAGE_PIN C18 IOSTANDARD LVCMOS33} [get_ports {GPIO0[0]}]
set_property -dict {PACKAGE_PIN E18 IOSTANDARD LVCMOS33} [get_ports {GPIO0[1]}]
set_property -dict {PACKAGE_PIN G18 IOSTANDARD LVCMOS33} [get_ports {GPIO0[2]}]
set_property -dict {PACKAGE_PIN D18 IOSTANDARD LVCMOS33} [get_ports {GPIO0[3]}]
set_property -dict {PACKAGE_PIN F18 IOSTANDARD LVCMOS33} [get_ports {GPIO0[4]}]
set_property -dict {PACKAGE_PIN H18 IOSTANDARD LVCMOS33} [get_ports {GPIO0[5]}]

# GPIO1
set_property -dict {PACKAGE_PIN M5 IOSTANDARD LVCMOS33} [get_ports {GPIO1[0]}]
set_property -dict {PACKAGE_PIN L6 IOSTANDARD LVCMOS33} [get_ports {GPIO1[1]}]
set_property -dict {PACKAGE_PIN K6 IOSTANDARD LVCMOS33} [get_ports {GPIO1[2]}]
set_property -dict {PACKAGE_PIN M3 IOSTANDARD LVCMOS33} [get_ports {GPIO1[3]}]
set_property -dict {PACKAGE_PIN K3 IOSTANDARD LVCMOS33} [get_ports {GPIO1[4]}]
set_property -dict {PACKAGE_PIN J6 IOSTANDARD LVCMOS33} [get_ports {GPIO1[5]}]
