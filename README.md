This project will create a program that will aid in identifying obd2 codes produced by devices (buttons) on the vehicle.
This will be done in two stages:
1) Compile:    In this phase, the program will listen to a steady state obd2 bus and develop a dictionary of messages that occur over a period of time.
2) Discovery:  As buttons are pushed, or other manually activated events occur, the program emits all messages that do not appear in the
               dictionary.
