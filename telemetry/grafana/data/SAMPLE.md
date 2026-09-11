The synthetic examples contain 301 samples over 15 seconds. Field center is (0, 0), with boundaries at +/-72 inches. Heading zero is +Y, clockwise positive. The route follows a smooth S-shaped curve from (-42, -54) to (42, 54), with gentle acceleration and deceleration and a brief simulated intake obstruction. Heading follows the curve tangent; wheel speeds include the difference needed to turn.

Model assumptions: 11W blue-cartridge motors, 3.25-inch wheels, 1:1 external gearing, and a 12-inch track width. These are demonstration assumptions, not a calibrated model of the robot. RPM follows differential-drive motion; current rises during acceleration and intake obstruction; battery voltage sags under load. Temperatures change slowly in 5-degree steps. Battery capacity stays constant during this short run. Target RPM stays zero to represent voltage-mode commands. Numerical precision of synthetic pose/RPM values is not a claim about sensor accuracy.

PROS documents temperature resolution as 5 C and thermal reduction beginning at 55 C; 55 C is not a maximum sensor reading. Current and voltage are integer mA/mV; measured RPM can be fractional. These examples assume a warmed-up robot: both intakes start near 45 C and warm toward 50 C with similar loads, while the six drivetrain motors stay near 35 C. These examples remain below thermal limiting.

Sources: https://pros.cs.purdue.edu/v5/api/cpp/motors.html and https://kb.vex.com/hc/en-us/articles/360044325872-Understanding-V5-Smart-Motor-11W-Performance

Running generate.py without arguments recreates example.csv. Using --csv only configures the dashboard and preserves that recording. Both bundled sample files are explicitly marked SYNTHETIC in every row.
