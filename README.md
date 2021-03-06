# open-auto-car
This project aims to build an autonomous RC car based on cheap components (Raspberry-Pi ZeroW +ESP32)

# Hardware :

## ESP32 :
Most pins on the ESP32 are re-mappable. Only the reciever is tied to Serial 2 pins (RX2 = Pin 16)

## Car Parts :
[Pinion](https://www.amazon.fr/gp/product/B00061H6XC/ref=ppx_yo_dt_b_asin_title_o07_s00?ie=UTF8&psc=1)
[Motor](https://www.banggood.com/fr/Surpass-Hobby-Waterproof-3650-4300KV-Brushless-RC-Car-Motor-With-60A-ESC-Set-For-1-or-10-RC-Car-p-1298142.html?rmmds=myorder&cur_warehouse=CN)
[ESC](https://www.amazon.fr/gp/product/B07B3V7C78/ref=ppx_yo_dt_b_asin_title_o08_s00?ie=UTF8&psc=1)
[Chassis](https://www.banggood.com/fr/1-or-10-Aluminium-Alloy-Metal-Carbon-Fiber-Chassis-Kit-Rc-Car-Frame-Parts-p-1593686.html?rmmds=myorder&cur_warehouse=CN)

## Sensors :
Accelerometer : MPU9250 
Proximity : HC-SR05 

## Remote control :
We're using a FlySky i6-AB remote control flashed with a 10 channel upgrade.
* [Remote](https://www.banggood.com/fr/FlySky-FS-i6-2_4G-6CH-AFHDS-RC-Radion-Transmitter-With-FS-iA6B-Receiver-for-RC-FPV-Drone-p-983537.html?rmmds=myorder&ID=42482&cur_warehouse=CN)
* [Custom Firmware](https://github.com/benb0jangles/FlySky-i6-Mod-/tree/master/10ch%20Timer%20Mod%20i6%20Updater/10ch_Timer_MOD_i6_Programmer_V1_4)

# Controls :

| Key | Function | 
| --- | --- | 
| Left stick vertical | Thrust | 
| Right stick horizontal | Steering | 
| 3 positions switch | Gear (see below) | 
| AUX A | Max speed cap | 

Gear :
* Top position : Forward
* Middle position : Neutral (idle)
* Bottom position : Backward

The car seems to be more controllable if you apply exponential curves to the steering/thrust on the controller.

	