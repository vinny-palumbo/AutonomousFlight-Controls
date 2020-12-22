# Project Writeup #

## Implement body rate control ##

I implemented the function `BodyRateControl()`
```
V3F momentCmd;
V3F pqr_err = pqrCmd - pqr;
momentCmd = V3F(Ixx,Iyy,Izz) * kpPQR * pqr_err;

return momentCmd;
```

## Implement roll / pitch control ##

I implemented the function `RollPitchControl()`
```
V3F pqrCmd;
Mat3x3F R = attitude.RotationMatrix_IwrtB();

if (collThrustCmd > 0.0){
  float c = -collThrustCmd / mass;
  float b_x_c = CONSTRAIN(accelCmd.x/c, -maxTiltAngle, maxTiltAngle);
  float b_y_c = CONSTRAIN(accelCmd.y/c, -maxTiltAngle, maxTiltAngle);
  
  float b_x = b_x_c - R(0, 2);
  float b_y = b_y_c - R(1, 2);

  pqrCmd.x = kpBank *((R(1, 0) * b_x) - (R(0, 0) * b_y)) / R(2, 2);
  pqrCmd.y = kpBank *((R(1, 1) * b_x) - (R(0, 1) * b_y)) / R(2, 2);
}else{
  pqrCmd.x = 0.0;
  pqrCmd.y = 0.0;
}

return pqrCmd;
```

## Implement altitude control ##

I implemented the function `AltitudeControl()`
```
Mat3x3F R = attitude.RotationMatrix_IwrtB();
float thrust = 0;

float posZ_err = posZCmd - posZ;
float velZ_err = velZCmd - velZ;
integratedAltitudeError += posZ_err * dt;

float u_bar = (kpPosZ * posZ_err) + (kpVelZ * velZ_err) + accelZCmd + (KiPosZ * integratedAltitudeError);

float accZ = (u_bar - 9.81f) / R(2,2);
if (accZ > 0){
    accZ = 0;
}
thrust = -accZ * mass;

return thrust;
```

## Implement lateral position control ##

I implemented the function `LateralPositionControl()`
```
// make sure we don't have any incoming z-component
accelCmdFF.z = 0;
velCmd.z = 0;
posCmd.z = pos.z;

// we initialize the returned desired acceleration to the feed-forward value.
// Make sure to _add_, not simply replace, the result of your controller
// to this variable
V3F accelCmd = accelCmdFF;

velCmd.x = CONSTRAIN(velCmd.x, -maxSpeedXY, maxSpeedXY);
velCmd.y = CONSTRAIN(velCmd.y, -maxSpeedXY, maxSpeedXY);

V3F pos_err = posCmd - pos;
V3F vel_err = velCmd - vel;

accelCmd = accelCmdFF + (kpPosXY * pos_err) + (kpVelXY * vel_err); 
accelCmd.x = CONSTRAIN(accelCmd.x, -maxAccelXY, maxAccelXY);
accelCmd.y = CONSTRAIN(accelCmd.y, -maxAccelXY, maxAccelXY);
accelCmd.z = 0;

return accelCmd;
```

## Implement yaw control ##

I first implemented the function `YawControl()`
```
float yawRateCmd=0;

yawCmd = fmodf(yawCmd, (2.0f*F_PI));
float yaw_err = yawCmd - yaw;

if(yaw_err > F_PI){
 yaw_err -= (2.0f*F_PI);
}else if(yaw_err <= -F_PI){
 yaw_err += (2.0f*F_PI);
}
yawRateCmd = kpYaw * yaw_err;

return yawRateCmd;
```

## Generate motor commands ##

I implemented the function `GenerateMotorCommands()`
```
float l = L / (sqrt(2.f));

float c_bar = collThrustCmd;
float p_bar = momentCmd.x / l;
float q_bar = momentCmd.y / l;
float r_bar = momentCmd.z / kappa;

cmd.desiredThrustsN[0] = (c_bar + p_bar + q_bar + r_bar) / 4.f; // front left
cmd.desiredThrustsN[1] = (c_bar - p_bar + q_bar - r_bar) / 4.f; // front right
cmd.desiredThrustsN[2] = (c_bar + p_bar - q_bar - r_bar) / 4.f; // rear left
cmd.desiredThrustsN[3] = (c_bar - p_bar - q_bar + r_bar) / 4.f; // rear right

return cmd;
```

## Perform the required tasks ##

I tuned the parameters in `QuadControlParams.txt`. The following params passed all the tests from the different scenarios:
```
# Position control gains
kpPosXY = 40 
kpPosZ = 20  
KiPosZ = 30 

# Velocity control gains
kpVelXY = 15 
kpVelZ = 9  

# Angle control gains
kpBank = 10
kpYaw = 2 

# Angle rate gains
kpPQR = 70, 70, -3
```

All the tests passed:
```
Simulation #1029 (../config/1_Intro.txt)
PASS: ABS(Quad.PosFollowErr) was less than 0.500000 for at least 0.800000 seconds

Simulation #1034 (../config/2_AttitudeControl.txt)
PASS: ABS(Quad.Roll) was less than 0.025000 for at least 0.750000 seconds
PASS: ABS(Quad.Omega.X) was less than 2.500000 for at least 0.750000 seconds

Simulation #1036 (../config/3_PositionControl.txt)
PASS: ABS(Quad1.Pos.X) was less than 0.100000 for at least 1.250000 seconds
PASS: ABS(Quad2.Pos.X) was less than 0.100000 for at least 1.250000 seconds
PASS: ABS(Quad2.Yaw) was less than 0.100000 for at least 1.000000 seconds

Simulation #1038 (../config/4_Nonidealities.txt)
PASS: ABS(Quad1.PosFollowErr) was less than 0.100000 for at least 1.500000 seconds
PASS: ABS(Quad2.PosFollowErr) was less than 0.100000 for at least 1.500000 seconds
PASS: ABS(Quad3.PosFollowErr) was less than 0.100000 for at least 1.500000 seconds

Simulation #1040 (../config/5_TrajectoryFollow.txt)
PASS: ABS(Quad2.PosFollowErr) was less than 0.250000 for at least 3.000000 seconds
```