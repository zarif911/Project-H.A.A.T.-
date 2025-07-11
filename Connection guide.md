## Servo-pot:

Connection Guide:

PCA9685 Connections:



SDA → Arduino Mega SDA (pin 20)



SCL → Arduino Mega SCL (pin 21)



VCC → 5V



GND → GND



Servos → Channels 0-3



Potentiometer Connections:



Left pin → 5V



Right pin → GND



Middle pin → Analog pins A0-A3



Power Supply:



Use a separate 5-6V power supply for servos



Connect PCA9685 power input to this external supply



Keep Arduino powered via USB or separate power



Calibration Notes:

For the 360° base servo:



1500μs = stop



1300μs = full speed clockwise



1700μs = full speed counter-clockwise



Adjust BASE\_MIN and BASE\_MAX if needed



For 180° servos:



If servos jitter or don't move fully:



Increase JOINT\_MAX (up to 2500)



Decrease JOINT\_MIN (down to 500)



Test safe limits before permanent mounting



Required Libraries:

Adafruit PWM Servo Driver Library:



Install via Arduino Library Manager or



https://github.com/adafruit/Adafruit-PWM-Servo-Driver-Library



Wire Library (built-in with Arduino IDE)



Troubleshooting Tips:

If servos behave erratically:



Check power supply (servos require significant current)



Add 1000μF capacitor across servo power lines



Ensure all grounds are connected together



If movements are jumpy:



Add small capacitors (0.1μF) across potentiometer leads



Increase the delay at the end of loop()



For serial monitoring (optional):



Add this to setup: Serial.begin(9600);



Add this before delay in loop:



cpp

Serial.print("Pulses: ");

Serial.print(basePulse);

Serial.print(" ");

Serial.print(joint1Pulse);

Serial.print(" ");

Serial.print(joint2Pulse);

Serial.print(" ");

Serial.println(gripperPulse);



## Record-play:

Connection Guide

Additional Components:

Pushbuttons (3x):



Connect between Arduino pin and GND



Internal pull-up resistors are used



LEDs (3x):



Anode (+) to Arduino pin through 220Ω resistor



Cathode (-) to GND



Wiring:

text

Buttons:

&nbsp; Record Button -> Pin 22

&nbsp; Play Button   -> Pin 24

&nbsp; Pause Button  -> Pin 26



LEDs:

&nbsp; Record LED -> Pin 28

&nbsp; Play LED   -> Pin 30

&nbsp; Pause LED  -> Pin 32

Power Considerations:

Use a separate 5-6V power supply for servos



Connect PCA9685 power to external supply



Keep Arduino powered via USB or separate power



Add 1000μF capacitor across servo power lines



Features:

Manual Control Mode:



Use potentiometers to control arm in real-time



Default state when no modes are active



Record Mode:



Press RECORD button to start recording positions



Arm movements are stored at 50ms intervals



Press RECORD again to stop



Red LED indicates recording



Playback Mode:



Press PLAY to execute recorded sequence



Sequence loops continuously



Green LED indicates playback



Automatically stops if recording buffer is full



Pause Function:



Press PAUSE during playback to freeze arm position



Yellow LED indicates pause state



Press PLAY to resume



Calibration Tips:

For 360° base servo:



1500μs = Stop



1300μs = Full speed clockwise



1700μs = Full speed counter-clockwise



Adjust BASE\_MIN and BASE\_MAX if needed



For 180° servos:



If movement range is insufficient:



Increase JOINT\_MAX (up to 2500)



Decrease JOINT\_MIN (down to 500)



Test limits before permanent mounting



Troubleshooting:

Servo jitter:



Add 0.1μF capacitors across potentiometer leads



Ensure stable power supply



Increase STEP\_INTERVAL for smoother playback



Recording issues:



Check serial monitor for debug messages



Increase MAX\_STEPS for longer sequences



Decrease STEP\_INTERVAL for more frequent sampling



Button responsiveness:



Adjust DEBOUNCE\_DELAY in the code



Ensure buttons are properly connected to GND



Memory issues:



Reduce MAX\_STEPS if experiencing crashes



Arduino Mega has 8KB RAM - 200 steps uses ~1.6KB





