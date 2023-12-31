// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

  private IntakeIO intakeIO;
  public static boolean isIntooked = false;

  /** Creates a new Intake. */
  public Intake(IntakeIO io) {
    
    intakeIO = io;
        
  }

  public void setMotor(double intakeSpeed) {
    intakeIO.setMotor(intakeSpeed);
    SmartDashboard.putNumber("intake/motor input (%)", intakeSpeed);
}

  public double getCurrent() {
    return intakeIO.getCurrent();
  }

  //returns speed of the intake
  public double getEncoderSpeed() {
    return intakeIO.getEncoderSpeed();
  }

  public double getEncoderPosition() {
    return intakeIO.getEncoderPosition();
  }

  public void setPosition(double position) {
    intakeIO.setPosition(position);
  }

  public void setCurrentLimit(int current) {
    intakeIO.setCurrentLimit(current);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("intake/current (A)", getCurrent());
    // SmartDashboard.putNumber("intake/motor output (%)", RealIntake.intakeMotorController.get());
    // SmartDashboard.putNumber("intake/MOTOR OUTPUT", RealIntake.intakeMotorController.getAppliedOutput());
    
    intakeIO.periodicUpdate();
  }
}
