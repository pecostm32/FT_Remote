# FT_Remote
All resources for upgrading an old 27MHz fischertechink remote control system to modern digital technology.

Owning two old fischertechnik remote control systems that have intermittent problems with transmissions I
decided to design a new digital system for it, reusing the old casings and joysticks.

Wrote about the journey on EEVBlog: https://www.eevblog.com/forum/projects/modernizing-old-fischertechnik-27mc-remote-control/

In this repository all parts of the project are made available.

As per the lisence, I take no reponsability for what is in this repository. Use it at your own risk.

The source code for the firmware of both boards is based on shared files all included in the repository. It is fully
bare metal and no extras are needed. It may be that you have to tweak the makefile to make it work from command line
directly. I use netbeans 8.2.

-------------------------------------------------------------------------------------------------------------------------
Still to do:

The transmitter needs code for battery status monitoring. At the moment the power LED just blinks all the time, but this
needs to be changed to constant on when the batteries are fresh and fade when the voltage drops and start to blink when
the voltage reaches a certain low point.
