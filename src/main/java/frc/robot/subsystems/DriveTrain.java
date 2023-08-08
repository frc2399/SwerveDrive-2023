// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveTrainConstants;

public class DriveTrain extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public static CANSparkMax steer1;
  public static CANSparkMax drive1;

  public static CANSparkMax steer2;
  public static CANSparkMax drive2;

  public static CANSparkMax steer3;
  public static CANSparkMax drive3;

  public static CANSparkMax steer4;
  public static CANSparkMax drive4;

  public static SparkMaxAbsoluteEncoder encoder1;
  public static SparkMaxAbsoluteEncoder encoder2;
  public static SparkMaxAbsoluteEncoder encoder3;
  public static SparkMaxAbsoluteEncoder encoder4;

  public DriveTrain() {

    // TODO: figure out the correct device IDs or reconfigure the IDs
    steer1 = new CANSparkMax(0, MotorType.kBrushless);
    drive1 = new CANSparkMax(0, MotorType.kBrushless);

    steer2 = new CANSparkMax(0, MotorType.kBrushless);
    drive2 = new CANSparkMax(0, MotorType.kBrushless);

    steer3 = new CANSparkMax(0, MotorType.kBrushless);
    drive3 = new CANSparkMax(0, MotorType.kBrushless);

    steer4 = new CANSparkMax(0, MotorType.kBrushless);
    drive4 = new CANSparkMax(0, MotorType.kBrushless);

    //driving motor inversion
    drive1.setInverted(false);
    drive2.setInverted(false);
    drive3.setInverted(false);
    drive4.setInverted(false);

    //steering motor inversion 
    steer1.setInverted(true);
    steer2.setInverted(true);
    steer3.setInverted(true);
    steer4.setInverted(true);

    // steering encoders
    encoder1 = steer1.getAbsoluteEncoder(Type.kDutyCycle);
    encoder2 = steer2.getAbsoluteEncoder(Type.kDutyCycle);
    encoder3 = steer3.getAbsoluteEncoder(Type.kDutyCycle);
    encoder4 = steer4.getAbsoluteEncoder(Type.kDutyCycle);

    encoder1.setInverted(false);
    encoder2.setInverted(false);
    encoder3.setInverted(false);
    encoder4.setInverted(false);

    
    encoder1.setPositionConversionFactor(2 * Math.PI);
    encoder2.setPositionConversionFactor(2 * Math.PI);
    encoder3.setPositionConversionFactor(2 * Math.PI);
    encoder4.setPositionConversionFactor(2 * Math.PI);

    encoder1.setZeroOffset(convertToSparkMaxAngle(0 + DriveTrainConstants.ENCODER1_OFFSET));
    encoder2.setZeroOffset(convertToSparkMaxAngle(Math.PI/2 + DriveTrainConstants.ENCODER2_OFFSET));
    encoder3.setZeroOffset(convertToSparkMaxAngle(Math.PI + DriveTrainConstants.ENCODER3_OFFSET));
    encoder4.setZeroOffset(convertToSparkMaxAngle(-Math.PI/2 + DriveTrainConstants.ENCODER4_OFFSET));

    // steering pid controllers
    var controller1 = steer1.getPIDController();
    var controller2 = steer2.getPIDController();
    var controller3 = steer3.getPIDController();
    var controller4 = steer4.getPIDController();

    // giving the controller an encoder to read values from
    controller1.setFeedbackDevice(encoder1);
    controller2.setFeedbackDevice(encoder2);
    controller3.setFeedbackDevice(encoder3);
    controller4.setFeedbackDevice(encoder4);

    // if the sensor value is greater than 360, it wraps the number such that the
    // value is between 0 and 360
    controller1.setPositionPIDWrappingEnabled(true);
    controller2.setPositionPIDWrappingEnabled(true);
    controller3.setPositionPIDWrappingEnabled(true);
    controller4.setPositionPIDWrappingEnabled(true);

    // sets P gain for the PID loop
    // TODO: tune the P gain
    controller1.setP(1);
    controller2.setP(1);
    controller3.setP(1);
    controller4.setP(1);

    // gives the controller a target value that is a position. since this is in the
    // constructor, we set the target to 0 so that the robot does not move
    controller1.setReference(convertToSparkMaxAngle(0), ControlType.kPosition);
    controller2.setReference(convertToSparkMaxAngle(0), ControlType.kPosition);
    controller3.setReference(convertToSparkMaxAngle(0), ControlType.kPosition);
    controller4.setReference(convertToSparkMaxAngle(0), ControlType.kPosition);

  }

  public void setSpeed(double fwd, double str, double rcw) {
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

    double currentAngle1 = encoder1.getPosition();
    double currentAngle2 = encoder2.getPosition();
    double currentAngle3 = encoder3.getPosition();
    double currentAngle4 = encoder4.getPosition();

    // calculate the speed and angle for each wheel
    double speed1 = Math.sqrt(fwd1 * fwd1 + str1 * str1);
    double angle1 = Math.atan2(fwd1, str1);

    double speed2 = Math.sqrt(fwd2 * fwd2 + str2 * str2);
    double angle2 = Math.atan2(fwd2, str2);

    double speed3 = Math.sqrt(fwd3 * fwd3 + str3 * str3);
    double angle3 = Math.atan2(fwd3, str3);

    double speed4 = Math.sqrt(fwd4 * fwd4 + str4 * str4);
    double angle4 = Math.atan2(fwd4, str4);

    double[] flipAngleAndSpeed1 = flipAngle(currentAngle1, angle1, speed1);
    double[] flipAngleAndSpeed2 = flipAngle(currentAngle2, angle2, speed2);
    double[] flipAngleAndSpeed3 = flipAngle(currentAngle3, angle3, speed3);
    double[] flipAngleAndSpeed4 = flipAngle(currentAngle4, angle4, speed4);

    double flippedSpeed1 = flipAngleAndSpeed1[1];
    double flippedAngle1 = flipAngleAndSpeed1[0];

    double flippedSpeed2 = flipAngleAndSpeed2[1];
    double flippedAngle2 = flipAngleAndSpeed2[0];

    double flippedSpeed3 = flipAngleAndSpeed3[1];
    double flippedAngle3 = flipAngleAndSpeed3[0];

    double flippedSpeed4 = flipAngleAndSpeed4[1];
    double flippedAngle4 = flipAngleAndSpeed4[0];

    // set all wheel speeds
    // TODO: limit speed in case speed goes over 1
    if (flippedSpeed1 != 0) {
      drive1.set(flippedSpeed1);
      steer1.getPIDController().setReference(convertToSparkMaxAngle(flippedAngle1), ControlType.kPosition);

    } else {
      drive1.set(0);
      steer1.set(0);
    }

    if (flippedSpeed2 != 0) {
      drive2.set(flippedSpeed1);
      steer2.getPIDController().setReference(convertToSparkMaxAngle(flippedAngle2), ControlType.kPosition);

    } else {
      drive2.set(0);
      steer2.set(0);
    }

    if (flippedSpeed3 != 0) {
      drive3.set(flippedSpeed3);
      steer3.getPIDController().setReference(convertToSparkMaxAngle(flippedAngle3), ControlType.kPosition);

    } else {
      drive3.set(0);
      steer3.set(0);
    }

    if (flippedSpeed4 != 0) {
      drive4.set(flippedSpeed4);
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



  public static double[] flipAngle(double currentAngle, double targetAngle, double speed) {
    double[] changedValues = new double[2];
    double desiredAngle = targetAngle;
    double desiredSpeed = speed;
    if (Math.abs(targetAngle - currentAngle) > Units.degreesToRadians(90)
        && Math.abs(targetAngle - currentAngle) < Units.degreesToRadians(270)) {
      desiredAngle += Units.degreesToRadians(180);
      if (desiredAngle > Units.degreesToRadians(180)) {
        desiredAngle -= Units.degreesToRadians(360);
      }
      desiredSpeed *= -1;
    }
    changedValues[0] = desiredAngle;
    changedValues[1] = desiredSpeed;
    return changedValues;
  }
// convert zero to 2pi range to negative pi to pi range
  public static double convertFromSparkMaxAngle(double currentAngle){
    if (currentAngle >= 0 && currentAngle <= Math.PI)
    {
      return currentAngle; 
    }
     currentAngle= currentAngle - 2 * Math.PI;
    while(currentAngle<-Math.PI){
      currentAngle+=Math.PI*2;
    }
    while(currentAngle>Math.PI){
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
    while(angle<0){
      angle+=Math.PI*2;
    }
    while(angle>Math.PI *2){
      angle-=Math.PI*2;
    }
    return angle;
    
  
  }
}
