# PS2 speed limits

Manual analog commands use PS2_V_MAX=0.20 m/s and W_MAX=2.00 rad/s.
D-pad commands are +/-0.20 m/s for forward/reverse and +/-0.50 rad/s for
turning. Both body commands are clamped before differential-drive mixing
to handle the raw analog -128/127 endpoint. ROS command limits remain
unchanged. Per-wheel speeds can exceed 0.20 m/s during combined translation
and turning; 0.20 is the requested body linear-speed limit. These are
command limits, not guaranteed instantaneous measured speeds.

Build succeeded using the existing zackon dependency installation in an
isolated /tmp staging directory, with existing application warnings.
Candidate: build-linux/candidates/f767-ps2-limits/. Not flashed yet.
