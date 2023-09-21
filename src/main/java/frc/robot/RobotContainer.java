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

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
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

  private Command setGroundIntakeSetpoint;
  private Command setArmUpIntakeSetpoint;
  private Command setShootSetpoint;

  public static CommandSelector angleHeight = CommandSelector.CUBE_INTAKE;

  //a chooser for the autons
  final SendableChooser<Command> chooser = new SendableChooser<>();
  final ComplexWidget autonChooser = Shuffleboard.getTab("Driver")
  .add("Choose Auton", chooser).withWidget(BuiltInWidgets.kSplitButtonChooser).withPosition(4, 4).withSize(9, 1);



  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    setUpAutonChooser();
    setUpConeCubeCommands();
    configureBindings();
  }

 
  private void configureBindings() {
    
    //TODO: refine the cubing and deadband
    //test deadband
   m_driveTrain.setDefaultCommand(new RunCommand(() -> m_driveTrain.setSpeed(
      -computeDeadband(joystick.getRawAxis(JoystickConstants.FWD_AXIS), 0.05), 
      computeDeadband(joystick.getRawAxis(JoystickConstants.STR_AXIS), 0.05),  
     computeDeadband(Math.pow(joystick.getRawAxis(JoystickConstants.RCW_AXIS), 3), 0.05)) , m_driveTrain));

     //right d-pad to intake on operator
    intake.setDefaultCommand(
      new StallIntakeCmd(intake,
      //up on joystick dpad to intake
      () -> joystick.getPOV() == 0,
      //down on joystick dpad to outtake
      () -> joystick.getPOV() == 180));
     
    
    //button that sets all wheels to 0 degrees (homing position) (Button 3)
    new JoystickButton(joystick, 3).whileTrue(
        new RunCommand(() -> DriveTrain.setWheelAngles(0,0,0,0), m_driveTrain)); 

    //button that sets the wheels into lock position (an X) (Button 4)
    new JoystickButton(joystick, 4).whileTrue( new RunCommand(() -> DriveTrain.setWheelAngles(
          Units.degreesToRadians(45),
          Units.degreesToRadians(-45),
          Units.degreesToRadians(45),
          Units.degreesToRadians(-45)), m_driveTrain)); 
    
    //button that resets gyro (Button 5)
    new JoystickButton(joystick, 5).onTrue( new InstantCommand(() -> DriveTrain.ahrs.reset(), m_driveTrain));

    //new JoystickButton(joystick, 7).onTrue( new StrafeGivenDistance(-1, m_driveTrain));

    //button for ground setpoint (Button 7)
    new JoystickButton(joystick, 7).onTrue(setGroundIntakeSetpoint);

    //button for arm up/turtle mode (Button 9)
    new JoystickButton(joystick, 9).onTrue(setArmUpIntakeSetpoint);

    //button for shoot (Button 11)
    new JoystickButton(joystick, 11).onTrue(setShootSetpoint);

    //button to send arm to selected position (Button 1)
    new JoystickButton(joystick, 1).onTrue(selectPositionCommand());

    //button to reset arm (Button 6)
    new JoystickButton(joystick, 6).onTrue(resetArmEncoderCommand(arm)); 

    //manually control the arm
    new Trigger(() -> joystick.getPOV() == 90).whileTrue(makeSetSpeedGravityCompensationCommand(arm, 0.2)).onFalse(makeSetSpeedGravityCompensationCommand(arm, 0)); 
    new Trigger(() -> joystick.getPOV() == 270).whileTrue(makeSetSpeedGravityCompensationCommand(arm, -0.2)).onFalse(makeSetPositionCommand(arm, 0)); 

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // The selected command will be run in autonomous
    String autonCommand = chooser.getSelected().toString();
    DataLogManager.log("Auton: " + autonCommand);
    return chooser.getSelected();
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
    setGroundIntakeSetpoint = new InstantCommand(() -> {
      angleHeight = CommandSelector.CUBE_INTAKE;
  });

    setArmUpIntakeSetpoint = new InstantCommand(() -> {
      angleHeight = CommandSelector.ARM_UP; 
    });

    setShootSetpoint = new InstantCommand(() -> {
      angleHeight = CommandSelector.CUBE_SHOOT;
    });

  }


  private void setUpAutonChooser () {
    chooser.setDefaultOption("do nothing", new PrintCommand("i am doing nothing"));
}

  public static Command makeSetPositionCommand(ProfiledPIDSubsystem base, double target) {
    return new SequentialCommandGroup(
        new ConditionalCommand(new InstantCommand(() -> {}), new InstantCommand(() -> base.enable()), () -> base.isEnabled()),
        new InstantCommand(() -> base.setGoal(target), base)
    );
}

  private Command makeSetSpeedGravityCompensationCommand(Arm a, double speed) {
    return new SequentialCommandGroup(
        new InstantCommand(() -> a.disable()),
        new RunCommand(() -> a.setSpeedGravityCompensation(speed), a)
    );
  }

  private static Command resetArmEncoderCommand(Arm a) {
    Debouncer debouncer = new Debouncer(0.2);
    return new SequentialCommandGroup(
        new InstantCommand(() ->  a.disable()),
        new RunCommand(() -> a.setSpeed(0.15)).withTimeout(0.2),
        new RunCommand(() -> a.setSpeed(0.15)).until(() -> debouncer.calculate(Math.abs(a.getEncoderSpeed()) < 0.01)),
        new InstantCommand(() -> a.setPosition(Constants.ArmConstants.INITIAL_OFFSET)),
        makeSetPositionCommand(a, ArmConstants.ARM_UP_ANGLE)
    );
  }

  public enum CommandSelector {
    CUBE_INTAKE,
    CUBE_SHOOT,
    ARM_UP
  }


  private CommandSelector select() {
    return angleHeight;
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
