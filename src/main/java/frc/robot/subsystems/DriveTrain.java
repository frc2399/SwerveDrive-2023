// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.util.NavX.AHRS;

public class DriveTrain extends SubsystemBase {
  // MAXSwerveModules
  private final MAXSwerveModule m_frontLeft;
  private final MAXSwerveModule m_frontRight;
  private final MAXSwerveModule m_rearLeft;
  private final MAXSwerveModule m_rearRight;

  // Gyro (NAVX)
  public static AHRS ahrs;

  private LinearFilter derivativeCalculator = LinearFilter.backwardFiniteDifference(1, 2, 0.02);
  private double pitchRate;

  public DriveTrain() {
    // Create MAXSwerveModules
    final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
      DriveTrainConstants.FRONT_LEFT_DRIVING_CAN_ID,
      DriveTrainConstants.FRONT_LEFT_TURNING_CAN_ID,
      DriveTrainConstants.FRONT_LEFT_CHASSIS_ANGULAR_OFFSET);

    final MAXSwerveModule m_frontRight = new MAXSwerveModule(
      DriveTrainConstants.FRONT_RIGHT_DRIVING_CAN_ID,
      DriveTrainConstants.FRONT_RIGHT_TURNING_CAN_ID,
      DriveTrainConstants.FRONT_RIGHT_CHASSIS_ANGULAR_OFFSET);

    final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
      DriveTrainConstants.REAR_LEFT_DRIVING_CAN_ID,
      DriveTrainConstants.REAR_LEFT_TURNING_CAN_ID,
      DriveTrainConstants.REAR_LEFT_CHASSIS_ANGULAR_OFFSET);

    final MAXSwerveModule m_rearRight = new MAXSwerveModule(
      DriveTrainConstants.REAR_RIGHT_DRIVING_CAN_ID,
      DriveTrainConstants.REAR_RIGHT_TURNING_CAN_ID,
      DriveTrainConstants.REAR_RIGHT_CHASSIS_ANGULAR_OFFSET);

    // Create gyro
    ahrs = new AHRS(SPI.Port.kMXP, (byte) 66);
    ahrs.reset();
    
    //steerEncoder1.setZeroOffset(convertToSparkMaxAngle(0 + DriveTrainConstants.ENCODER1_OFFSET));
    //steerEncoder2.setZeroOffset(convertToSparkMaxAngle(Math.PI/2 + DriveTrainConstants.ENCODER2_OFFSET));
    //steerEncoder3.setZeroOffset(convertToSparkMaxAngle(Math.PI + DriveTrainConstants.ENCODER3_OFFSET));
    //steerEncoder4.setZeroOffset(convertToSparkMaxAngle(-Math.PI/2 + DriveTrainConstants.ENCODER4_OFFSET));

    // gives the controller a target value that is a position. since this is in the
    // constructor, we set the target to 0 so that the robot does not move
    controller1.setReference(convertToSparkMaxAngle(0), ControlType.kPosition);
    controller2.setReference(convertToSparkMaxAngle(0), ControlType.kPosition);
    controller3.setReference(convertToSparkMaxAngle(0), ControlType.kPosition);
    controller4.setReference(convertToSparkMaxAngle(0), ControlType.kPosition);
  }

  @Override
  public void periodic()
  {
      // This will get the simulated sensor readings that we set
  // in the previous article while in simulation, but will use
  // real values on the robot itself.
//   m_odometry.update(m_gyro.getRotation2d(),
//   m_leftEncoder.getDistance(),
//   m_rightEncoder.getDistance());
// m_field.setRobotPose(m_odometry.getPoseMeters());
    SmartDashboard.putNumber("Gyro angle", ahrs.getAngle());
    pitchRate = derivativeCalculator.calculate(getGyroPitch());

  }

  public void setSpeed(double fwd, double str, double rcw) {
  
    SmartDashboard.putNumber("Encoder position: ", driveEncoder1.getPosition());
    //field oriented
    double angleRad = Math.toRadians(ahrs.getAngle());
        double temp = fwd * Math.cos(angleRad) +
                str * Math.sin(angleRad);
        str = -fwd * Math.sin(angleRad) + str * Math.cos(angleRad);
        fwd = temp;
    
    double rcw_fwd = rcw * Math.sin(DriveTrainConstants.THETA);
    double rcw_str = rcw * Math.cos(DriveTrainConstants.THETA);

    // calculate the forward and strafe values for each wheel
    double fwd1 = fwd - rcw_fwd;
    double str1 = str + rcw_str;

    double fwd2 = fwd + rcw_fwd;
    double str2 = str + rcw_str;

    double fwd3 = fwd + rcw_fwd;
    double str3 = str - rcw_str;

    double fwd4 = fwd - rcw_fwd;
    double str4 = str - rcw_str;

    double currentAngle1 = steerEncoder1.getPosition();
    //SmartDashboard.putNumber("Current angle", Units.radiansToDegrees(currentAngle1));
    double currentAngle2 = steerEncoder2.getPosition();
    double currentAngle3 = steerEncoder3.getPosition();
    double currentAngle4 = steerEncoder4.getPosition();

    // calculate the speed and angle for each wheel
    double speed1 = Math.sqrt(fwd1 * fwd1 + str1 * str1);
    double angle1 = Math.atan2(str1, fwd1);
    //SmartDashboard.putNumber("Target angle", Units.radiansToDegrees(convertToSparkMaxAngle(angle1)));

    double speed2 = Math.sqrt(fwd2 * fwd2 + str2 * str2);
    double angle2 = Math.atan2(str2, fwd2);

    double speed3 = Math.sqrt(fwd3 * fwd3 + str3 * str3);
    double angle3 = Math.atan2(str3, fwd3);

    double speed4 = Math.sqrt(fwd4 * fwd4 + str4 * str4);
    double angle4 = Math.atan2(str4, fwd4);

    double[] flipAngleAndSpeed1 = flipAngle(currentAngle1, angle1, speed1);
    double[] flipAngleAndSpeed2 = flipAngle(currentAngle2, angle2, speed2);
    double[] flipAngleAndSpeed3 = flipAngle(currentAngle3, angle3, speed3);
    double[] flipAngleAndSpeed4 = flipAngle(currentAngle4, angle4, speed4);

    double flippedSpeed1 = flipAngleAndSpeed1[1];
    SmartDashboard.putNumber("Changed speed", flippedSpeed1);
    double flippedAngle1 = flipAngleAndSpeed1[0];
    SmartDashboard.putNumber("Changed angle", flippedAngle1);

    double flippedSpeed2 = flipAngleAndSpeed2[1];
    double flippedAngle2 = flipAngleAndSpeed2[0];

    double flippedSpeed3 = flipAngleAndSpeed3[1];
    double flippedAngle3 = flipAngleAndSpeed3[0];

    double flippedSpeed4 = flipAngleAndSpeed4[1];
    double flippedAngle4 = flipAngleAndSpeed4[0];

    //limiting speeds in case they go over 1 (normalize or desaturate)
    double speedProportion = 1;
    double maxSpeed = Math.max(Math.abs(flippedSpeed1), 
                      Math.max(Math.abs(flippedSpeed2), 
                      Math.max(Math.abs(flippedSpeed3), Math.abs(flippedSpeed4))));
    if (maxSpeed > 1)
    {
      speedProportion = maxSpeed;
    }


    SmartDashboard.putNumber("Max Speed", maxSpeed);
    SmartDashboard.putNumber("Speed 1", flippedSpeed1/speedProportion);
  
    // set all wheel speeds
    if (flippedSpeed1 != 0) {
      drive1.set(flippedSpeed1/speedProportion * 0.5);
      steer1.getPIDController().setReference(convertToSparkMaxAngle(flippedAngle1), ControlType.kPosition);

    } else {
      drive1.set(0);
      steer1.set(0);
    }

    if (flippedSpeed2 != 0) {
      drive2.set(flippedSpeed2/speedProportion * 0.5);
      steer2.getPIDController().setReference(convertToSparkMaxAngle(flippedAngle2), ControlType.kPosition);

    } else {
      drive2.set(0);
      steer2.set(0);
    }

    if (flippedSpeed3 != 0) {
      drive3.set(flippedSpeed3/speedProportion * 0.5);
      steer3.getPIDController().setReference(convertToSparkMaxAngle(flippedAngle3), ControlType.kPosition);

    } else {
      drive3.set(0);
      steer3.set(0);
    }

    if (flippedSpeed4 != 0) {
      drive4.set(flippedSpeed4/speedProportion * 0.5);
      steer4.getPIDController().setReference(convertToSparkMaxAngle(flippedAngle4), ControlType.kPosition);

    } else {
      drive4.set(0);
      steer4.set(0);
    }
    
  }

  public static void setWheelAngles(double angle1, double angle2, double angle3, double angle4) {
    steer1.getPIDController().setReference(convertToSparkMaxAngle(angle1), ControlType.kPosition);
    steer2.getPIDController().setReference(convertToSparkMaxAngle(angle2), ControlType.kPosition);
    steer3.getPIDController().setReference(convertToSparkMaxAngle(angle3), ControlType.kPosition);
    steer4.getPIDController().setReference(convertToSparkMaxAngle(angle4), ControlType.kPosition);
  }
  
// convert zero to 2pi range to negative pi to pi range
  public static double convertFromSparkMaxAngle(double currentAngle){
    if (currentAngle >= 0 && currentAngle <= Math.PI)
    {
      return currentAngle; 
    }
     currentAngle= currentAngle - 2 * Math.PI;

    while(currentAngle < -Math.PI){
      currentAngle += Math.PI*2;
    }
    while(currentAngle > Math.PI){
      currentAngle-=Math.PI*2;
  }
  return currentAngle;
  }

//convert from negative pi to pi range to zero to 2pi range
  public static double convertToSparkMaxAngle(double angle) {

    if (angle >= 0 && angle <= Math.PI)
    {
      return angle; 
    }
    angle= angle+Math.PI * 2;
    while(angle < 0){
      angle += Math.PI*2;
    }
    while(angle>Math.PI *2){
      angle-=Math.PI*2;
    }
    return angle;
  }

  public double getAverageEncoderMeters()
  {
    return (driveEncoder1.getPosition() + driveEncoder2.getPosition() + driveEncoder3.getPosition() + driveEncoder4.getPosition()) / 4;
  }

  public double getGyroPitch() {
    return -ahrs.getPitch();
  }

  public double getGyroPitchRate()
  {
      return pitchRate;
  }

  public void setDriveMotorVoltage(double drivePower1, double drivePower2, double drivePower3, double drivePower4) {
    drive1.setVoltage(drivePower1);
    drive2.setVoltage(drivePower2);
    drive3.setVoltage(drivePower3);
    drive4.setVoltage(drivePower4);
  }


}
