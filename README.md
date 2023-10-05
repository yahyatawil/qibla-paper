

Source files of "A Hardware Implementation Of An Open-source Qibla Direction Finder With Tilt Compensation Using 9-DOF IMU And GPS" paper presented in Innovations in Intelligent Systems and Applications Conference (ASYU). 

## Abstract 
The proposed Hardware implementation studied the tilt compensation and compass soft-iron and hard-iron calibration. Without the tilt compensation and compass calibration, finding the Qibla direction will prone to error. The Qibla error without tilt compensation can reach up to 100 degrees for 30 degrees pitch. The proposed design uses out-of-the-shelf 9DOF (9 Degrees of Freedom) IMU (Inertial Measurement Unit), and an optional GPS receiver to get the location. The accelerometer and gyroscope data, with a complementary filter,  are used to calculate the pitch and roll angles which are used in tilt compensation. The magnetometer is used to calculate the azimuth.   

Bellow comparison between the calculated direction with and without tilt compensation in different pitch and roll setups:

![comparison between the calculated direction with
and without tilt compensation in different pitch and roll setups](/imgs/compare-table.png)

## Hardware 
The circuit consists of
- Arduino Nano 33 BLE Sense board
- BMI270 shuttle board
- Adafruit Mini GPS PA1010D
- Monochrome 0.91" 128x32 I2C OLED Display
- Li-Ion battery and tp4056 charger

<figure>
<img src="/imgs/hw.png" width=45% alt="The proposed Qibla finder"/>
</figure>


## Experiaments
The calculated Qibla direction using the proposed system is compared against the reference qibla of two mosques in Gaziantep/Turkey, as shown bellow: 

<figure>
<img src="/imgs/validation.png" width=45% alt="Comparing with mosques qibla"/>
</figure>

Also, an Android Qibla finder app was used to test the calculated Qibla with and without tilt. 

<figure>
<img src="/imgs/validation2.png" width=45% alt="Comparing with Android app"/>
</figure>
  

## Extras
As part of the work was done in this paper, please check:
- Magnetometer calibration remotely using BLE connection: [MotionCal-BLE](https://github.com/yahyatawil/MotionCal-BLE)
- Arduino library for BMI270 with BMM150 as an auxiliary sensor: [BMI270_AUX_BMM150](https://github.com/yahyatawil/BMI270_AUX_BMM150)
- Technical articles about IMU, and Magnetometer calibration (available in English and Arabic):
  - [The first prototype of Qibla finder](https://atadiat.com/en/e-open-source-qibla-compass-with-tilt-compensation/).
  - [Towards understanding imu: basics of accelerometer and gyroscope sensors](https://atadiat.com/en/e-towards-understanding-imu-basics-of-accelerometer-and-gyroscope-sensors/)
  - [Magnetometer soft-iron and hard-iron calibration](https://atadiat.com/en/e-magnetometer-soft-iron-and-hard-iron-calibration-why-how/)

## Citation 
```
@inproceedings{tawil2022deep,
  title={A Hardware Implementation Of An Open-source Qibla Direction Finder With Tilt Compensation Using 9-DOF IMU And GPS},
  author={Tawil, Yahya},
  booktitle={2023 Innovations in Intelligent Systems and Applications Conference (ASYU)},
  pages={1--6},
  year={2023},
  organization={IEEE}
}
```
