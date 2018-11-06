# PolarisAir
Custom flight control system (c/c++)

Copyright (C) 2014 Hery A Mwenegoha.

Built for the Atmega-2560 chip. 

## Setup and Functions
Works with the Polaris Ground Station - UGCS3

4 Functioning Modes - Manual, Stabilised, Auto and Guided

Uses L1 for guidance and TECS for pitch and throttle control

Tested on a 2kg and 4kg fixed wing UAV. Works well, even though needs some additional work

## Collaborations
Contact: Hery.

1. Implement EKF (INS/GNSS/Mag/Baro/Airspeed) for sensor fusion and have the DCM working as secondary fusion scheme.
   EKF are fairly common, plan on extending this to UKFs for a model based approach to improve accuracy of both mean and covariance      
   estimation to fourth order.
2. Optimise IMU setup and support for additional sensors.
   GNSS-RTK setup, medium grade MEMs IMU, LiDAR etc.
3. Optimise TECS and Guidance logic
4. VTOL application and support for hybrid propulsion
5. Extend to other chips (Tentatively).
