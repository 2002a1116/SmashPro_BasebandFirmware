| Supported Targets | ESP32 |
| ----------------- | ----- |

# SmashPro Baseband Firmware

Connected to Ch32v103 mcu through custom uart packet.Collecting inputs from Ch32v103 and send them to nintendo switch by bluetooth.

Supported basic functions,ota upload and tcp connection for control.


configures needed by nintendo switch r not ported from wired firmware yet,if anyone can have that done,I'd like to merge it.


## todo:

1.configures function porting or internal sharing.

2.fix rumble delay on bluetooth.I assume MotorStatus byte is the key.

3.make an app for wireless config through wifi,dont have time for that now.

4.flash ch32v103 firmware through uart,this is possible on hardware but need a lot of work,as wch.cn doesnt provide a guide.

