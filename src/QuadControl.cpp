#include "Common.h"
#include "QuadControl.h"

#include "Utility/SimpleConfig.h"

#include "Utility/StringUtils.h"
#include "Trajectory.h"
#include "BaseController.h"
#include "Math/Mat3x3F.h"

#ifdef __PX4_NUTTX
#include <systemlib/param/param.h>
#endif

#define GRAVITY	(float)9.81

static void limit(V3F &vec, float a, float b)
{
  vec.x = CONSTRAIN(vec.x, a, b);
  vec.y = CONSTRAIN(vec.y, a, b);
  vec.z = CONSTRAIN(vec.z, a, b);
}


void QuadControl::Init()
{
  BaseController::Init();

  // variables needed for integral control
  integratedAltitudeError = 0;

#ifndef __PX4_NUTTX
  // Load params from simulator parameter system
  ParamsHandle config = SimpleConfig::GetInstance();

  // Load parameters (default to 0)
  kpPosXY = config->Get(_config + ".kpPosXY", 0);
  kpPosZ = config->Get(_config + ".kpPosZ", 0);

  KiPosZ = config->Get(_config + ".KiPosZ", 0);

  kpVelXY = config->Get(_config + ".kpVelXY", 0);
  kpVelZ = config->Get(_config + ".kpVelZ", 0);

  kpBank = config->Get(_config + ".kpBank", 0);
  kpYaw = config->Get(_config + ".kpYaw", 0);

  kpPQR = config->Get(_config + ".kpPQR", V3F());

  maxDescentRate = config->Get(_config + ".maxDescentRate", 100);
  maxAscentRate = config->Get(_config + ".maxAscentRate", 100);
  maxSpeedXY = config->Get(_config + ".maxSpeedXY", 100);
  maxAccelXY = config->Get(_config + ".maxHorizAccel", 100);

  maxTiltAngle = config->Get(_config + ".maxTiltAngle", 100);

  minMotorThrust = config->Get(_config + ".minMotorThrust", 0);
  maxMotorThrust = config->Get(_config + ".maxMotorThrust", 100);
#else
  // load params from PX4 parameter system
  //TODO
  param_get(param_find("MC_PITCH_P"), &Kp_bank);
  param_get(param_find("MC_YAW_P"), &Kp_yaw);
#endif
}

VehicleCommand QuadControl::GenerateMotorCommands(float collThrustCmd, V3F momentCmd)
{
  // Convert a desired 3-axis moment and collective thrust command to 
  //   individual motor thrust commands
  // INPUTS: 
  //   collThrustCmd: desired collective thrust [N]
  //   momentCmd: desired rotation moment about each axis [N m]
  // OUTPUT:
  //   set class member variable cmd (class variable for graphing) where
  //   cmd.desiredThrustsN[0..3]: motor commands, in [N]

  // HINTS: 
  // - you can access parts of momentCmd via e.g. momentCmd.x
  // You'll need the arm length parameter L, and the drag/thrust ratio kappa

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
  float l = L / (float)sqrt(2);

  float A = collThrustCmd;
  float B = momentCmd.x / l;
  float C = momentCmd.y / l;
  float D = momentCmd.z / kappa;
  //float maxThrustDiff = 2 * (maxMotorThrust - minMotorThrust)*collThrustCmd/(4*maxMotorThrust);
  //B = CONSTRAIN(B, -maxThrustDiff, maxThrustDiff);
  //C = CONSTRAIN(C, -maxThrustDiff, maxThrustDiff);
  //D = CONSTRAIN(D, -maxThrustDiff, maxThrustDiff);

  cmd.desiredThrustsN[0] = (A + B + C - D) / 4;
  cmd.desiredThrustsN[1] = (A - B + C + D) / 4;
  cmd.desiredThrustsN[2] = (A + B - C + D) / 4;
  cmd.desiredThrustsN[3] = (A - B - C - D) / 4;

  cmd.desiredThrustsN[0] = CONSTRAIN(cmd.desiredThrustsN[0], minMotorThrust, maxMotorThrust);
  cmd.desiredThrustsN[1] = CONSTRAIN(cmd.desiredThrustsN[1], minMotorThrust, maxMotorThrust);
  cmd.desiredThrustsN[2] = CONSTRAIN(cmd.desiredThrustsN[2], minMotorThrust, maxMotorThrust);
  cmd.desiredThrustsN[3] = CONSTRAIN(cmd.desiredThrustsN[3], minMotorThrust, maxMotorThrust);

  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return cmd;
}

V3F QuadControl::BodyRateControl(V3F pqrCmd, V3F pqr)
{
  // Calculate a desired 3-axis moment given a desired and current body rate
  // INPUTS: 
  //   pqrCmd: desired body rates [rad/s]
  //   pqr: current or estimated body rates [rad/s]
  // OUTPUT:
  //   return a V3F containing the desired moments for each of the 3 axes

  // HINTS: 
  //  - you can use V3Fs just like scalars: V3F a(1,1,1), b(2,3,4), c; c=a-b;
  //  - you'll need parameters for moments of inertia Ixx, Iyy, Izz
  //  - you'll also need the gain parameter kpPQR (it's a V3F)

  V3F momentCmd;

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
  //pqrCmd = V3F(0, 0, 0);

  V3F err = pqrCmd - pqr;
  V3F u_bar = kpPQR * err;

  V3F I = V3F(Ixx, Iyy, Izz);
  momentCmd = u_bar * I;

  //printf("err = %f, %f, %f\n", err[0], err[1], err[2]);
  //printf("momentCmd = %f, %f, %f\n", momentCmd[0], momentCmd[1], momentCmd[2]);

  /////////////////////////////// END STUDENT CODE ////////////////////////////
  return momentCmd;
}

// returns a desired roll and pitch rate 
V3F QuadControl::RollPitchControl(V3F accelCmd, Quaternion<float> attitude, float collThrustCmd)
{
  // Calculate a desired pitch and roll angle rates based on a desired global
  //   lateral acceleration, the current attitude of the quad, and desired
  //   collective thrust command
  // INPUTS: 
  //   accelCmd: desired acceleration in global XY coordinates [m/s2]
  //   attitude: current or estimated attitude of the vehicle
  //   collThrustCmd: desired collective thrust of the quad [N]
  // OUTPUT:
  //   return a V3F containing the desired pitch and roll rates. The Z
  //     element of the V3F should be left at its default value (0)

  // HINTS: 
  //  - we already provide rotation matrix R: to get element R[1,2] (python) use R(1,2) (C++)
  //  - you'll need the roll/pitch gain kpBank
  //  - collThrustCmd is a force in Newtons! You'll likely want to convert it to acceleration first

  V3F pqrCmd;
  Mat3x3F R = attitude.RotationMatrix_IwrtB();

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
  float acc = collThrustCmd / mass;
  V3F b_c = accelCmd / acc;
  limit(b_c, -maxTiltAngle, maxTiltAngle);
  assert(collThrustCmd >= 0);
  V3F b = V3F(R(0, 2), R(1, 2), 0);
  V3F b_err = b_c - b;
  V3F b_dot = kpBank * b_err;

  float rot_mat[9] = { R(1,0), -R(0,0), 0,
                       R(1,1), -R(0,1), 0,
                       0,	   0,      0 };

  float b_z = R(2, 2);
  Mat3x3F R1 = Mat3x3F(rot_mat) / b_z;

  pqrCmd = R1 * b_dot;

  //printf("accelCmd = %f, %f, %f\n", accelCmd[0], accelCmd[1], accelCmd[2]);
  //printf("b_c = %f, %f, %f\n", b_c[0], b_c[1], b_c[2]);
  //printf("b_err = %f, %f, %f\n", b_err[0], b_err[1], b_err[2]);
  //printf("b_dot = %f, %f, %f\n", b_dot[0], b_dot[1], b_dot[2]);
  //printf("pqrCmd = %f, %f, %f\n", pqrCmd[0], pqrCmd[1], pqrCmd[2]);

  //pqrCmd = V3F(0, 0, 0);
  pqrCmd.z = 0;

  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return pqrCmd;
}

float QuadControl::AltitudeControl(float posZCmd, float velZCmd, float posZ, float velZ, Quaternion<float> attitude, float accelZCmd, float dt)
{
  // Calculate desired quad thrust based on altitude setpoint, actual altitude,
  //   vertical velocity setpoint, actual vertical velocity, and a vertical 
  //   acceleration feed-forward command
  // INPUTS: 
  //   posZCmd, velZCmd: desired vertical position and velocity in NED [m]
  //   posZ, velZ: current vertical position and velocity in NED [m]
  //   accelZCmd: feed-forward vertical acceleration in NED [m/s2]
  //   dt: the time step of the measurements [seconds]
  // OUTPUT:
  //   return a collective thrust command in [N]

  // HINTS: 
  //  - we already provide rotation matrix R: to get element R[1,2] (python) use R(1,2) (C++)
  //  - you'll need the gain parameters kpPosZ and kpVelZ
  //  - maxAscentRate and maxDescentRate are maximum vertical speeds. Note they're both >=0!
  //  - make sure to return a force, not an acceleration
  //  - remember that for an upright quad in NED, thrust should be HIGHER if the desired Z acceleration is LOWER

  Mat3x3F R = attitude.RotationMatrix_IwrtB();
  float thrust = 0;

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
  float e_pos_z = posZCmd - posZ;
  velZCmd += kpPosZ*e_pos_z;
  velZCmd = CONSTRAIN(velZCmd, -maxAscentRate, maxDescentRate);
  float e_vel_z = velZCmd - velZ;
  integratedAltitudeError += dt * e_pos_z;
  
  accelZCmd += kpVelZ * e_vel_z + KiPosZ * integratedAltitudeError - GRAVITY;
  thrust = -mass * accelZCmd / R(2,2);

  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return thrust;
}

// returns a desired acceleration in global frame
V3F QuadControl::LateralPositionControl(V3F posCmd, V3F velCmd, V3F pos, V3F vel, V3F accelCmdFF)
{
  // Calculate a desired horizontal acceleration based on 
  //  desired lateral position/velocity/acceleration and current pose
  // INPUTS: 
  //   posCmd: desired position, in NED [m]
  //   velCmd: desired velocity, in NED [m/s]
  //   pos: current position, NED [m]
  //   vel: current velocity, NED [m/s]
  //   accelCmdFF: feed-forward acceleration, NED [m/s2]
  // OUTPUT:
  //   return a V3F with desired horizontal accelerations. 
  //     the Z component should be 0
  // HINTS: 
  //  - use the gain parameters kpPosXY and kpVelXY
  //  - make sure you limit the maximum horizontal velocity and acceleration
  //    to maxSpeedXY and maxAccelXY

  // make sure we don't have any incoming z-component
  accelCmdFF.z = 0;
  velCmd.z = 0;
  posCmd.z = pos.z;

  // we initialize the returned desired acceleration to the feed-forward value.
  // Make sure to _add_, not simply replace, the result of your controller
  // to this variable
  V3F accelCmd = accelCmdFF;

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
  V3F err = posCmd - pos;
  V3F err_dot = velCmd - vel;

  accelCmd += kpPosXY * err + kpVelXY * err_dot;

  //printf("posCmd = %f, %f, %f\n", posCmd[0], posCmd[1], posCmd[2]);
  //printf("pos = %f, %f, %f\n", pos[0], pos[1], pos[2]);
  //printf("err = %f, %f, %f\n", err[0], err[1], err[2]);
  //printf("accelCmd a= %f, %f, %f\n", accelCmd[0], accelCmd[1], accelCmd[2]);
  //printf("vel.x = %.3f\n", vel.x);
  //limit(accelCmd, -maxAccelXY, +maxAccelXY);
  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return -accelCmd;
}

//static const double     _PI = 3.1415926535897932384626433832795028841971693993751058209749445923078164062862089986280348;

// returns desired yaw rate
float QuadControl::YawControl(float yawCmd, float yaw)
{
  // Calculate a desired yaw rate to control yaw to yawCmd
  // INPUTS: 
  //   yawCmd: commanded yaw [rad]
  //   yaw: current yaw [rad]
  // OUTPUT:
  //   return a desired yaw rate [rad/s]
  // HINTS: 
  //  - use fmodf(foo,b) to unwrap a radian angle measure float foo to range [0,b]. 
  //  - use the yaw control gain parameter kpYaw

  float yawRateCmd = 0;
  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
  float e_yaw = fmodf(yawCmd - yaw, 2 * F_PI);
 
  if (e_yaw > F_PI) e_yaw -= 2.f*F_PI;
  if (e_yaw < -F_PI) e_yaw += 2.f*F_PI;

  yawRateCmd = kpYaw * e_yaw;

  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return yawRateCmd;

}

VehicleCommand QuadControl::RunControl(float dt, float simTime)
{
  curTrajPoint = GetNextTrajectoryPoint(simTime);

  float collThrustCmd = AltitudeControl(curTrajPoint.position.z, curTrajPoint.velocity.z, estPos.z, estVel.z, estAtt, curTrajPoint.accel.z, dt);

  // reserve some thrust margin for angle control
  float thrustMargin = .1f*(maxMotorThrust - minMotorThrust);
  collThrustCmd = CONSTRAIN(collThrustCmd, (minMotorThrust + thrustMargin)*4.f, (maxMotorThrust - thrustMargin)*4.f);

  V3F desAcc = LateralPositionControl(curTrajPoint.position, curTrajPoint.velocity, estPos, estVel, curTrajPoint.accel);

  V3F desOmega = RollPitchControl(desAcc, estAtt, collThrustCmd);
  desOmega.z = YawControl(curTrajPoint.attitude.Yaw(), estAtt.Yaw());

  //Limit tilt:
  Mat3x3F R = estAtt.RotationMatrix_IwrtB();
  float b_x = R(0, 2);
  float b_y = R(1, 2);
  /*
  if (abs(b_x + dt * desOmega.x) > maxTiltAngle)
  desOmega.x = 0;
  if (abs(b_y + dt * desOmega.y) > maxTiltAngle)
  desOmega.y = 0;
  */
  //printf("0x%x\n", (int)this);
  //printf("b_x = %.3f, b_y = %.3f\n", b_x, b_y);
  //printf("thrust = %.3f, %.3f, %.3f, %.3f\n", cmd.desiredThrustsN[0], cmd.desiredThrustsN[1], cmd.desiredThrustsN[2], cmd.desiredThrustsN[3]);
  V3F desMoment = BodyRateControl(desOmega, estOmega);

  return GenerateMotorCommands(collThrustCmd, desMoment);
}
