// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.JoystickConstants;
import frc.robot.commands.drivetrain.DriveForwardGivenDistance;
import frc.robot.commands.drivetrain.StrafeGivenDistance;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.intake.Intake;

import java.util.Map;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.drivetrain.StallIntakeCmd;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveTrain m_driveTrain = new DriveTrain();

  private Joystick joystick = new Joystick(0);
  public static Intake intake;
  public static Arm arm;

  private Command setGroundUpIntakeSetpoint;

  public static CommandSelector angleHeight = CommandSelector.CUBE_INTAKE;



  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

 
  private void configureBindings() {
    
    //TODO: refine the cubing and deadband
    //test deadband
   m_driveTrain.setDefaultCommand(new RunCommand(() -> m_driveTrain.setSpeed(
      -computeDeadband(joystick.getRawAxis(JoystickConstants.FWD_AXIS), 0.05), 
      computeDeadband(joystick.getRawAxis(JoystickConstants.STR_AXIS), 0.05),  
     computeDeadband(Math.pow(joystick.getRawAxis(JoystickConstants.RCW_AXIS), 3), 0.05)) , m_driveTrain));
     
    
    //button that sets all wheels to 0 degrees (homing position)
    new JoystickButton(joystick, 1).whileTrue(
        new RunCommand(() -> DriveTrain.setWheelAngles(0,0,0,0), m_driveTrain)); 

    //button that sets the wheels into lock position (an X)
    new JoystickButton(joystick, 9).whileTrue( new RunCommand(() -> DriveTrain.setWheelAngles(
          Units.degreesToRadians(45),
          Units.degreesToRadians(-45),
          Units.degreesToRadians(45),
          Units.degreesToRadians(-45)), m_driveTrain)); 
    
    new JoystickButton(joystick, 8).onTrue( new InstantCommand(() -> DriveTrain.ahrs.reset(), m_driveTrain));

    new JoystickButton(joystick, 7).onTrue( new StrafeGivenDistance(-1, m_driveTrain));

    //button for ground setpoint
    new JoystickButton(joystick, 10).onTrue(setGroundUpIntakeSetpoint);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }


  //computing deadband
  public static double computeDeadband(double x, double deadband) {
    if (Math.abs(x) <= deadband) { 
        return 0; 
    }
    else {
        return x;
    }
  }

  private void setUpConeCubeCommands () {
    setGroundUpIntakeSetpoint = new InstantCommand(() -> {
      angleHeight = CommandSelector.CUBE_INTAKE;
  });

  }
  public static Command makeSetPositionCommand(ProfiledPIDSubsystem base, double target) {
    return new SequentialCommandGroup(
        new ConditionalCommand(new InstantCommand(() -> {}), new InstantCommand(() -> base.enable()), () -> base.isEnabled()),
        new InstantCommand(() -> base.setGoal(target), base)
    );
}

  public enum CommandSelector {
    CUBE_INTAKE,
    CUBE_SHOOT,
    ARM_UP
  }

  private Command selectPositionCommand() {
    return new SelectCommand(
        Map.ofEntries(
            Map.entry(CommandSelector.CUBE_INTAKE, makeSetPositionCommand(arm, ArmConstants.CUBE_INTAKE_ANGLE)),
            Map.entry(CommandSelector.CUBE_SHOOT, makeSetPositionCommand(arm, ArmConstants.CUBE_SHOOT_ANGLE)),
            Map.entry(CommandSelector.ARM_UP, makeSetPositionCommand(arm, ArmConstants.ARM_UP_ANGLE))),
        this::select);
    }
}
