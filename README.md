Faderboard Controller for a Yamaha X3423
========================================

This project takes out the X3423 motorized faderboard from a Yamaha 01v96 mixer and lets you wire it
up via USB. Additionally, it will have a display for each two faders that can display information
such as the assigned control.

Features right now:
	- Fader values are sent and received via MIDI CCs 1-17; sending a CC via MIDI will move the
	  fader to the respective position.
	- Integrating position regulator allows precise movement of the faders (which would otherwise
	  stop a bit before the actual target position due to friction).
	- Faders can work continously or in discrete steps; CCs 71-87 control the number of steps (where
	  0 means "continuous").
	- Calibration data is stored to flash. Perform a calibration by sending CC 100.

Hardware setup
--------------

Use a 1.25 pitch FFC A ribbon cable with 34 pins (*Note: these are quite difficult to get, I bought
a 40-lane-cable and cut it down.*) to connect the PCB with the faderboard on CN001; either use the
proper connector (*not JST*) to connect J12 with CN002, or solder an appropriate connector/cables to
CN002. Connect J16 and J17 to a buck/boost converter accepting 16V and outputting +15V/-15V, e.g.
DD39AJPA (DD1912PA should work too) from your favourite chinese marketplace.

Be sure to simultaneously power up both the main PSU and the USB board for now.
