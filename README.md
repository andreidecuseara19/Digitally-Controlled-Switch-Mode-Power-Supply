# Digitally-Controlled-Switch-Mode-Power-Supply
This diploma project outlines the design and implementation of a digitally controlled switch-
mode power supply using synchronous buck converter topology. The primary objective was to

develop a compact, efficient, and user-programmable DC voltage source, combining the high-
efficiency characteristics of switch-mode converters and the precision and flexibility of digital

control.
The circuit uses a microcontroller board (Arduino Mega), a digital-to-analog converter
(MCP4725), a KY-040 rotary encoder for the real-time adjustment of the output voltage and a
compact switching converter circuit using MP2307 which was modified. Voltage and current
readings were made using a STEVAL-DIGAFEV1 board, which features a high-precision
monitoring IC (TSC1641), and are displayed on an I2C LCD. Control over the output voltage was
achieved by injecting current into the feedback node of a switching converter circuit to obtain an
adjustable output voltage between 1.5V and 21V.
Experimental results confirmed the stability and functionality of the system, and the measured
output voltage was very close to the theoretical predictions. Measured percentage differences
were typically below 1.5%, and voltage stability was measured under varying loads. The project
was successful in demonstrating the practical utility of digitally controlled switch-mode power
supplies.
