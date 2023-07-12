// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

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



  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

 
  private void configureBindings() {
   m_driveTrain.setDefaultCommand(new RunCommand(() -> m_driveTrain.setSpeed(joystick.getRawAxis(0), joystick.getRawAxis(1), joystick.getRawAxis(2)), m_driveTrain));
   //TODO: figure out which axis numbers correspond with which inputs from the joystick (ie which number represents forward, strafe, and rotation)
   //TODO: tune joysticks (add deadband, square value maybe, etc)
   //TODO: add a homing button that sets all wheels to position 0
   //TODO: add a button that turns the robot into an X position
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

}
