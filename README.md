# Controller for a Mobile Robot with 4 Mecanum Wheels

## Set up bot-lcm-tunnel

`ssh pi@mbot04.m.eecs.umich.edu`

On Pi: 

`ps aux | grep tunnel`

If there are `bot-lcm-tunnel` running in the background, kill it.

On Beaglebone(192.168.7.2):

`bot-lcm-tunnel &`

On Pi: 

`bot-lcm-tunnel 192.168.7.2`


## Send files to Beaglebone from Pi

`scp -r mobilebot-f19 debian@192.168.4.2:~/`

`scp -r mobilebot-f19 debian@192.168.7.2:~/`

`scp mobilebot-f19/mobilebot/mobilebot.c debian@192.168.4.2:~/mobilebot-f19/mobilebot/`

`scp mobilebot-14/mobilebot/mobilebot.c debian@192.168.4.2:~/mobilebot-14/mobilebot/`

`scp mobilebot-23/mobilebot/mobilebot.c debian@192.168.7.2:~/mobilebot-23/mobilebot/`

## Notes

1 utime = 10^-6 s

`find . -exec touch \{\} \;`