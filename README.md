Wireless baseband firmware for SmashPro controller.

(actually in wireless mode esp32 baseband works more like the main mcu,and ch32 works like a south bridge.How ps2 this is!!)

todo:

1.code imu driver for gyroscope in wireless mode.

2.code ble for wireless configuring.(config through 2.4g with tcp is done already,but this will be very inconvenience for users because it means disconnecting internet.)

3.action film,save,replay and macro programming.these thing will be saved outside device(or saved on esp32 temporary and non-playable),

  and can be played by driver software by commanding controller report certain input. in this way,macro function will not hurt controller's tournament usage.
  
//i dont have time to do anything says above,sry.
