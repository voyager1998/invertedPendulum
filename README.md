# Controller for a Mobile Robot with 4 Mecanum Wheels

## Set up bot-lcm-tunnel

On Pi: 
`ssh pi@mbot04.m.eecs.umich.edu`

`ps aux | grep tunnel`

If there are `bot-lcm-tunnel` running in the background, kill it.

On Beaglebone(192.168.7.2):

`bot-lcm-tunnel &`

On Pi: 

`bot-lcm-tunnel 192.168.7.2`


## Send mobilebot-f19 to Beaglebone from Pi

`scp -r mobilebot-f19 debian@192.168.4.2:~/name`

`scp -r mobilebot-f19 debian@192.168.7.2:~/name`

