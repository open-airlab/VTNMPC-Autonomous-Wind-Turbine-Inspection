#!/bin/bash
#
# Dependencies:
#   sudo apt install xdotool

#WID=$(xprop -root | grep "_NET_ACTIVE_WINDOW(WINDOW)"| awk '{print $5}')
#xdotool windowfocus $WID
xdotool key ctrl+shift+t
#wmctrl -i -a $WID
sleep 1; xdotool type --delay 1 --clearmodifiers "$@"; xdotool key Return;
