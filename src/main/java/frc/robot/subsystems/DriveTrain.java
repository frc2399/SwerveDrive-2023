// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveTrainConstants;

public class DriveTrain extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public final CANSparkMax steer1; 
  public final CANSparkMax drive1; 

  public final CANSparkMax steer2; 
  public final CANSparkMax drive2; 

  public final CANSparkMax steer3; 
  public final CANSparkMax drive3; 

  public final CANSparkMax steer4; 
  public final CANSparkMax drive4; 



  public DriveTrain() {
  
    //TODO: figure out the correct device IDs or reconfigure the IDs
    steer1 = new CANSparkMax(0, MotorType.kBrushless);
    drive1 = new CANSparkMax(0, MotorType.kBrushless);

    steer2 = new CANSparkMax(0, MotorType.kBrushless);
    drive2 = new CANSparkMax(0, MotorType.kBrushless);

    steer3 = new CANSparkMax(0, MotorType.kBrushless);
    drive3 = new CANSparkMax(0, MotorType.kBrushless);

    steer4 = new CANSparkMax(0, MotorType.kBrushless);
    drive4 = new CANSparkMax(0, MotorType.kBrushless);

    //steering encoders
    SparkMaxAbsoluteEncoder encoder1 = steer1.getAbsoluteEncoder(Type.kDutyCycle);
    SparkMaxAbsoluteEncoder encoder2 = steer2.getAbsoluteEncoder(Type.kDutyCycle);
    SparkMaxAbsoluteEncoder encoder3 = steer3.getAbsoluteEncoder(Type.kDutyCycle);
    SparkMaxAbsoluteEncoder encoder4 = steer4.getAbsoluteEncoder(Type.kDutyCycle);

    //TODO: determine if any encoder values need to be inverted 
    encoder1.setInverted(false);
    encoder2.setInverted(false);
    encoder3.setInverted(false);
    encoder4.setInverted(false);

    //TODO: determine the position conversion factor 
    encoder1.setPositionConversionFactor(1);
    encoder2.setPositionConversionFactor(1);
    encoder3.setPositionConversionFactor(1);
    encoder4.setPositionConversionFactor(1);

    //steering pid controllers
    var controller1 = steer1.getPIDController();
    var controller2 = steer2.getPIDController();
    var controller3 = steer3.getPIDController();
    var controller4 = steer4.getPIDController();

    //giving the controller an encoder to read values from 
    controller1.setFeedbackDevice(encoder1);
    controller2.setFeedbackDevice(encoder2);
    controller3.setFeedbackDevice(encoder3);
    controller4.setFeedbackDevice(encoder4);

    //if the sensor value is greater than 360, it wraps the number such that the value is between 0 and 360
    controller1.setPositionPIDWrappingEnabled(true);
    controller2.setPositionPIDWrappingEnabled(true);
    controller3.setPositionPIDWrappingEnabled(true);
    controller4.setPositionPIDWrappingEnabled(true);

    //sets P gain for the PID loop
    //TODO: tune the P gain  
    controller1.setP(1);
    controller2.setP(1);
    controller3.setP(1);
    controller4.setP(1);

    //gives the controller a target value that is a position. since this is in the constructor, we set the target to 0 so that the robot does not move
    controller1.setReference(0, ControlType.kPosition);
    controller2.setReference(0, ControlType.kPosition);
    controller3.setReference(0, ControlType.kPosition);
    controller4.setReference(0, ControlType.kPosition);





  }

  public void setSpeed(double fwd, double str, double rcw){
    double rcw_fwd = rcw * Math.sin(DriveTrainConstants.THETA);
    double rcw_str = rcw * Math.cos(DriveTrainConstants.THETA); 

    //calculate the forward and strafe values for each wheel
    double fwd1 = fwd - rcw_fwd;
    double str1 = str + rcw_str;

    double fwd2 = fwd + rcw_fwd;
    double str2 = str + rcw_str;

    double fwd3 = fwd + rcw_fwd;
    double str3 = str - rcw_str;

    double fwd4 = fwd - rcw_fwd;
    double str4 = str - rcw_str; 

    //calculate the speed and angle for each wheel
    double speed1 = Math.sqrt(fwd1 * fwd1 + str1 * str1);
    double angle1 = Math.atan2(fwd1, str1);

    double speed2 = Math.sqrt(fwd2 * fwd2 + str2 * str2);
    double angle2 = Math.atan2(fwd2, str2);

    double speed3 = Math.sqrt(fwd3 * fwd3 + str3 * str3);
    double angle3 = Math.atan2(fwd3, str3);

    double speed4 = Math.sqrt(fwd4 * fwd4 + str4 * str4);
    double angle4 = Math.atan2(fwd4, str4);

    //set all wheel speeds
    //TODO: limit speed in case speed goes over 1
    //TODO: if calculated speed is 0, do not spin the steering motor  
    drive1.set(speed1);
    drive2.set(speed2);
    drive3.set(speed3);
    drive4.set(speed4);

    //steering motors controllers recieve the given target angle depending on the corner 
    steer1.getPIDController().setReference(angle1, ControlType.kPosition);
    steer2.getPIDController().setReference(angle2, ControlType.kPosition);
    steer3.getPIDController().setReference(angle3, ControlType.kPosition);
    steer4.getPIDController().setReference(angle4, ControlType.kPosition);

    //TODO: ask the motors to go to a different angle and invert the speed depending on the closest position 
  }

}
