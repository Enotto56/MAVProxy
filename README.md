![GitHub Actions](https://github.com/ardupilot/MAVProxy/actions/workflows/windows_build.yml/badge.svg)

MAVProxy

This is a MAVLink ground station written in python. 

Please see https://ardupilot.org/mavproxy/index.html for more information

This ground station was developed as part of the CanberraUAV OBC team
entry

License
-------

MAVProxy is released under the GNU General Public License v3 or later


Maintainers
-----------

The best way to discuss MAVProxy with the maintainers is to join the
mavproxy channel on ArduPilot discord at https://ardupilot.org/discord

Lead Developers: Andrew Tridgell and Peter Barker

Windows Maintainer: Stephen Dade

MacOS Maintainer: Rhys Mainwaring

CatchLeader Target Smoothing
---------------------------

The `catchleader` module now exposes a `target_filter_alpha` setting that
controls exponential smoothing applied to the commanded intercept target.
Values close to `1.0` track the raw predictor output, while lower values apply
more damping to latitude, longitude, and altitude updates. The filter resets
whenever guidance mode changes or a manual waypoint is supplied so that fresh
commands are not delayed by stale history.

When the optional GUI is running the smoothing gain can be adjusted with the
"Target smoothing" slider, which updates the setting in real time and mirrors
changes performed through the CLI.
