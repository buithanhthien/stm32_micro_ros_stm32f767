# PS2 speed limits

Manual analog commands use PS2_V_MAX=0.20 m/s and W_MAX=2.00 rad/s.
D-pad commands are +/-0.20 m/s for forward/reverse and +/-0.50 rad/s for
turning. Both body commands are clamped before differential-drive mixing
to handle the raw analog -128/127 endpoint. After mixing, each wheel's target
is limited to 0.20 m/s for manual and ROS commands. With the 0.545 m wheelbase,
a D-pad turn requests 0.13625 m/s per wheel and fits within the limit. An
analog turn in place saturates at about 0.734 rad/s after wheel limiting.
These are command limits, not measured speeds.

The earlier build in build-linux/candidates/f767-ps2-limits/ predates these
settings.
