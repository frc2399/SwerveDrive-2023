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
  /** Creates a new ExampleSubsystem. */
  public static CANSparkMax steer1;
  public static CANSparkMax drive1;

  public static CANSparkMax steer2;
  public static CANSparkMax drive2;

  public static CANSparkMax steer3;
  public static CANSparkMax drive3;

  public static CANSparkMax steer4;
  public static CANSparkMax drive4;

  public static SparkMaxAbsoluteEncoder steerEncoder1;
  public static SparkMaxAbsoluteEncoder steerEncoder2;
  public static SparkMaxAbsoluteEncoder steerEncoder3;
  public static SparkMaxAbsoluteEncoder steerEncoder4;

  public static RelativeEncoder driveEncoder1;
  public static RelativeEncoder driveEncoder2;
  public static RelativeEncoder driveEncoder3;
  public static RelativeEncoder driveEncoder4;

  public static AHRS ahrs;

  private LinearFilter derivativeCalculator = LinearFilter.backwardFiniteDifference(1, 2, 0.02);
  private double pitchRate;

  public DriveTrain() {

    ahrs = new AHRS(SPI.Port.kMXP, (byte) 66);
    ahrs.reset();

    // right front
    steer1 = new CANSparkMax(32, MotorType.kBrushless);
    drive1 = new CANSparkMax(31, MotorType.kBrushless);

    // left front
    steer2 = new CANSparkMax(12, MotorType.kBrushless);
    drive2 = new CANSparkMax(11, MotorType.kBrushless);

    // left back
    steer3 = new CANSparkMax(22, MotorType.kBrushless);
    drive3 = new CANSparkMax(21, MotorType.kBrushless);

    // right back
    steer4 = new CANSparkMax(42, MotorType.kBrushless);
    drive4 = new CANSparkMax(41, MotorType.kBrushless);

    // driving factory resets
    drive1.restoreFactoryDefaults();
    drive2.restoreFactoryDefaults();
    drive3.restoreFactoryDefaults();
    drive4.restoreFactoryDefaults();

    // steering factory resets
    steer1.restoreFactoryDefaults();
    steer2.restoreFactoryDefaults();
    steer3.restoreFactoryDefaults();
    steer4.restoreFactoryDefaults();

    // drive current limits
    drive1.setSmartCurrentLimit(50);
    drive2.setSmartCurrentLimit(50);
    drive3.setSmartCurrentLimit(50);
    drive4.setSmartCurrentLimit(50);

    // steer current limits
    steer1.setSmartCurrentLimit(20);
    steer2.setSmartCurrentLimit(20);
    steer3.setSmartCurrentLimit(20);
    steer4.setSmartCurrentLimit(20);

    // driving motor inversion
    drive1.setInverted(false);
    drive2.setInverted(false);
    drive3.setInverted(false);
    drive4.setInverted(false);

    // steering motor inversion
    steer1.setInverted(true);
    steer2.setInverted(true);
    steer3.setInverted(true);
    steer4.setInverted(true);

    drive1.setIdleMode(IdleMode.kBrake);
    drive2.setIdleMode(IdleMode.kBrake);
    drive3.setIdleMode(IdleMode.kBrake);
    drive4.setIdleMode(IdleMode.kBrake);

    steer1.setIdleMode(IdleMode.kBrake);
    steer2.setIdleMode(IdleMode.kBrake);
    steer3.setIdleMode(IdleMode.kBrake);
    steer4.setIdleMode(IdleMode.kBrake);

    // driving encoders
    driveEncoder1 = drive1.getEncoder();
    driveEncoder2 = drive2.getEncoder();
    driveEncoder3 = drive3.getEncoder();
    driveEncoder4 = drive4.getEncoder();

    driveEncoder1.setPosition(0);
    driveEncoder2.setPosition(0);
    driveEncoder3.setPosition(0);
    driveEncoder4.setPosition(0);

    // native unit is rotations, converting to meters
    // 1 rotation = x wheel revolutions (using gear ratio) = y inches (using
    // circumference) = z meters
    driveEncoder1.setPositionConversionFactor(0.0508);
    driveEncoder2.setPositionConversionFactor(0.0508);
    driveEncoder3.setPositionConversionFactor(0.0508);
    driveEncoder4.setPositionConversionFactor(0.0508);

    // native unit is RPM, converting to m/s
    driveEncoder1.setVelocityConversionFactor(0.0508 / 60);
    driveEncoder2.setVelocityConversionFactor(0.0508 / 60);
    driveEncoder3.setVelocityConversionFactor(0.0508 / 60);
    driveEncoder4.setVelocityConversionFactor(0.0508 / 60);

    // steering encoders
    steerEncoder1 = steer1.getAbsoluteEncoder(Type.kDutyCycle);
    steerEncoder2 = steer2.getAbsoluteEncoder(Type.kDutyCycle);
    steerEncoder3 = steer3.getAbsoluteEncoder(Type.kDutyCycle);
    steerEncoder4 = steer4.getAbsoluteEncoder(Type.kDutyCycle);

    steerEncoder1.setInverted(false);
    steerEncoder2.setInverted(false);
    steerEncoder3.setInverted(false);
    steerEncoder4.setInverted(false);

    steerEncoder1.setPositionConversionFactor(2 * Math.PI);
    steerEncoder2.setPositionConversionFactor(2 * Math.PI);
    steerEncoder3.setPositionConversionFactor(2 * Math.PI);
    steerEncoder4.setPositionConversionFactor(2 * Math.PI);

    steerEncoder1.setZeroOffset(convertToSparkMaxAngle(0 + DriveTrainConstants.ENCODER1_OFFSET));
    steerEncoder2.setZeroOffset(convertToSparkMaxAngle(Math.PI / 2 + DriveTrainConstants.ENCODER2_OFFSET));
    steerEncoder3.setZeroOffset(convertToSparkMaxAngle(Math.PI + DriveTrainConstants.ENCODER3_OFFSET));
    steerEncoder4.setZeroOffset(convertToSparkMaxAngle(-Math.PI / 2 + DriveTrainConstants.ENCODER4_OFFSET));

    // steering pid controllers
    var steerController1 = steer1.getPIDController();
    var steerController2 = steer2.getPIDController();
    var steerController3 = steer3.getPIDController();
    var steerController4 = steer4.getPIDController();

    // steering pid controllers
    var driveController1 = drive1.getPIDController();
    var driveController2 = drive2.getPIDController();
    var driveController3 = drive3.getPIDController();
    var driveController4 = drive4.getPIDController();

    // giving the controller an encoder to read values from
    steerController1.setFeedbackDevice(steerEncoder1);
    steerController2.setFeedbackDevice(steerEncoder2);
    steerController3.setFeedbackDevice(steerEncoder3);
    steerController4.setFeedbackDevice(steerEncoder4);

    driveController1.setFeedbackDevice(driveEncoder1);
    driveController2.setFeedbackDevice(driveEncoder2);
    driveController3.setFeedbackDevice(driveEncoder3);
    driveController4.setFeedbackDevice(driveEncoder4);

    // if the sensor value is greater than 360, it wraps the number such that the
    // value is between 0 and 360
    steerController1.setPositionPIDWrappingEnabled(true);
    steerController2.setPositionPIDWrappingEnabled(true);
    steerController3.setPositionPIDWrappingEnabled(true);
    steerController4.setPositionPIDWrappingEnabled(true);

    // setting max and min inputs for encoder positions
    steerController1.setPositionPIDWrappingMinInput(DriveTrainConstants.kTurningEncoderPositionPIDMinInput);
    steerController1.setPositionPIDWrappingMaxInput(DriveTrainConstants.kTurningEncoderPositionPIDMaxInput);
    steerController2.setPositionPIDWrappingMinInput(DriveTrainConstants.kTurningEncoderPositionPIDMinInput);
    steerController2.setPositionPIDWrappingMaxInput(DriveTrainConstants.kTurningEncoderPositionPIDMaxInput);
    steerController3.setPositionPIDWrappingMinInput(DriveTrainConstants.kTurningEncoderPositionPIDMinInput);
    steerController3.setPositionPIDWrappingMaxInput(DriveTrainConstants.kTurningEncoderPositionPIDMaxInput);
    steerController4.setPositionPIDWrappingMinInput(DriveTrainConstants.kTurningEncoderPositionPIDMinInput);
    steerController4.setPositionPIDWrappingMaxInput(DriveTrainConstants.kTurningEncoderPositionPIDMaxInput);

    // sets P gain for the PID loop
    // TODO: tune the P gain
    steerController1.setP(1);
    steerController2.setP(1);
    steerController3.setP(1);
    steerController4.setP(1);

    driveController1.setFF(1);
    driveController2.setFF(1);
    driveController3.setFF(1);
    driveController4.setFF(1);
    
    // driveController1.setP(1);
    // driveController2.setP(1);
    // driveController3.setP(1);
    // driveController4.setP(1);

    // gives the controller a target value that is a position. since this is in the
    // constructor, we set the target to 0 so that the robot does not move
    steerController1.setReference(convertToSparkMaxAngle(0), ControlType.kPosition);
    steerController2.setReference(convertToSparkMaxAngle(0), ControlType.kPosition);
    steerController3.setReference(convertToSparkMaxAngle(0), ControlType.kPosition);
    steerController4.setReference(convertToSparkMaxAngle(0), ControlType.kPosition);

  }

  @Override
  public void periodic() {
    // This will get the simulated sensor readings that we set
    // in the previous article while in simulation, but will use
    // real values on the robot itself.
    // m_odometry.update(m_gyro.getRotation2d(),
    // m_leftEncoder.getDistance(),
    // m_rightEncoder.getDistance());
    // m_field.setRobotPose(m_odometry.getPoseMeters());
    SmartDashboard.putNumber("Gyro angle", ahrs.getAngle());
    pitchRate = derivativeCalculator.calculate(getGyroPitch());

  }

  public void setSpeed(double fwd, double str, double rcw) {

    SmartDashboard.putNumber("Encoder position: ", driveEncoder1.getPosition());
    // field oriented
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
    // SmartDashboard.putNumber("Current angle",
    // Units.radiansToDegrees(currentAngle1));
    double currentAngle2 = steerEncoder2.getPosition();
    double currentAngle3 = steerEncoder3.getPosition();
    double currentAngle4 = steerEncoder4.getPosition();

    // calculate the speed and angle for each wheel
    double speed1 = Math.sqrt(fwd1 * fwd1 + str1 * str1);
    double angle1 = Math.atan2(str1, fwd1);
    // SmartDashboard.putNumber("Target angle",
    // Units.radiansToDegrees(convertToSparkMaxAngle(angle1)));

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

    // limiting speeds in case they go over 1 (normalize or desaturate)
    double speedProportion = 1;
    double maxSpeed = Math.max(Math.abs(flippedSpeed1),
        Math.max(Math.abs(flippedSpeed2),
            Math.max(Math.abs(flippedSpeed3), Math.abs(flippedSpeed4))));
    if (maxSpeed > 1) {
      speedProportion = maxSpeed;
    }

    SmartDashboard.putNumber("Max Speed", maxSpeed);
    SmartDashboard.putNumber("Speed 1", flippedSpeed1 / speedProportion * 0.5);
    SmartDashboard.putNumber("Speed 2", flippedSpeed2 / speedProportion * 0.5);
    SmartDashboard.putNumber("Speed 3", flippedSpeed3 / speedProportion * 0.5);
    SmartDashboard.putNumber("Speed 4", flippedSpeed4 / speedProportion * 0.5);

    // set all wheel speeds
    if (flippedSpeed1 != 0) {
      drive1.getPIDController().setReference(flippedSpeed1 / speedProportion * 0.5, ControlType.kVelocity);
      steer1.getPIDController().setReference(convertToSparkMaxAngle(flippedAngle1), ControlType.kPosition);

    } else {
      drive1.set(0);
      steer1.set(0);
    }

    if (flippedSpeed2 != 0) {
      drive2.getPIDController().setReference(flippedSpeed2 / speedProportion * 0.5, ControlType.kVelocity);
      steer2.getPIDController().setReference(convertToSparkMaxAngle(flippedAngle2), ControlType.kPosition);

    } else {
      drive2.set(0);
      steer2.set(0);
    }

    if (flippedSpeed3 != 0) {
      drive3.getPIDController().setReference(flippedSpeed3 / speedProportion * 0.5, ControlType.kVelocity);
      steer3.getPIDController().setReference(convertToSparkMaxAngle(flippedAngle3), ControlType.kPosition);

    } else {
      drive3.set(0);
      steer3.set(0);
    }

    if (flippedSpeed4 != 0) {
      drive4.getPIDController().setReference(flippedSpeed4 / speedProportion * 0.5, ControlType.kVelocity);
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
    // double[] values = {targetAngle, speed};
    // return values;
  }

  // convert zero to 2pi range to negative pi to pi range
  public static double convertFromSparkMaxAngle(double currentAngle) {
    if (currentAngle >= 0 && currentAngle <= Math.PI) {
      return currentAngle;
    }
    currentAngle = currentAngle - 2 * Math.PI;

    while (currentAngle < -Math.PI) {
      currentAngle += Math.PI * 2;
    }
    while (currentAngle > Math.PI) {
      currentAngle -= Math.PI * 2;
    }
    return currentAngle;
  }

  // convert from negative pi to pi range to zero to 2pi range
  public static double convertToSparkMaxAngle(double angle) {

    if (angle >= 0 && angle <= Math.PI) {
      return angle;
    }
    angle = angle + Math.PI * 2;
    while (angle < 0) {
      angle += Math.PI * 2;
    }
    while (angle > Math.PI * 2) {
      angle -= Math.PI * 2;
    }
    return angle;
  }

  public double getAverageEncoderMeters() {
    return (driveEncoder1.getPosition() + driveEncoder2.getPosition() + driveEncoder3.getPosition()
        + driveEncoder4.getPosition()) / 4;
  }

  public double getGyroPitch() {
    return -ahrs.getPitch();
  }

  public double getGyroPitchRate() {
    return pitchRate;
  }

  public void setDriveMotorVoltage(double drivePower1, double drivePower2, double drivePower3, double drivePower4) {
    drive1.setVoltage(drivePower1);
    drive2.setVoltage(drivePower2);
    drive3.setVoltage(drivePower3);
    drive4.setVoltage(drivePower4);
  }

}
