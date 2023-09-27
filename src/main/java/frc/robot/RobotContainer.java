// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ArmConstants;
import frc.robot.commands.DriveForwardGivenDistance;
import frc.robot.commands.IntakeForGivenTime;
import frc.robot.commands.StallIntakeCmd;
import frc.robot.commands.autonomous.ShootLeaveCommunity;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmIO;
import frc.robot.subsystems.arm.RealArm;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.RealIntake;

import java.util.Map;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
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

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveTrain m_driveTrain = new DriveTrain();

  private XboxController xbox = new XboxController(0);
  public static Intake intake;
  public static Arm arm;

  private Command setGroundIntakeSetpoint;
  private Command setArmUpIntakeSetpoint;
  private Command setShootIntakeSetpoint;

  public static CommandSelector setpoint = CommandSelector.CUBE_INTAKE;

  // a chooser for the autons
  final SendableChooser<Command> chooser = new SendableChooser<>();
  final ComplexWidget autonChooser = Shuffleboard.getTab("Driver")
      .add("Choose Auton", chooser).withWidget(BuiltInWidgets.kSplitButtonChooser).withPosition(4, 4).withSize(9, 1);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    setUpSubsystems();
    setUpAutonChooser();
    setUpCubeCommands();
    configureBindings();
  }

  private void configureBindings() {

    m_driveTrain.setDefaultCommand(new RunCommand(() -> m_driveTrain.setSpeed(
        -computeDeadband(xbox.getRawAxis(XboxController.Axis.kLeftY.value), 0.07),
        computeDeadband(xbox.getRawAxis(XboxController.Axis.kLeftX.value), 0.07),
        computeDeadband(Math.pow(xbox.getRawAxis(XboxController.Axis.kRightX.value), 3), 0.05)),
        m_driveTrain));

    // right trigger to intake and left trigger to outtake on driver
    intake.setDefaultCommand(
        new StallIntakeCmd(intake,
            // right trigger to intake
            () -> (xbox.getRawAxis(XboxController.Axis.kRightTrigger.value) > 0.1),
            // left trigger to outtake
            () -> (xbox.getRawAxis(XboxController.Axis.kLeftTrigger.value) > 0.1)));

    // Driver Button Y (4) - sets all wheels to 0 degrees (homing position)
    new JoystickButton(xbox, Button.kY.value).whileTrue(
        new RunCommand(() -> DriveTrain.setWheelAngles(0, 0, 0, 0), m_driveTrain));

    // Driver Button X (3) - button that sets the wheels into lock position (an X)
    new JoystickButton(xbox, Button.kX.value).whileTrue(new RunCommand(() -> DriveTrain.setWheelAngles(
        Units.degreesToRadians(45),
        Units.degreesToRadians(-45),
        Units.degreesToRadians(45),
        Units.degreesToRadians(-45)), m_driveTrain));

    // B BUTTON (2): resets gyro
    new JoystickButton(xbox, Button.kB.value).onTrue(new InstantCommand(() -> DriveTrain.ahrs.reset(), m_driveTrain));

    // button for ground setpoint (Button 6/right bumper)
    new Trigger(() -> xbox.getRightBumperPressed() &&
        !xbox.getLeftBumperPressed()).onTrue(setGroundIntakeSetpoint);
    // button for shoot (Button 5/left bumper)
    new Trigger(() -> !xbox.getRightBumperPressed() &&
        xbox.getLeftBumperPressed()).onTrue(setShootIntakeSetpoint);
    // button for arm up/turtle mode (click right stick/Button 15)
    new JoystickButton(xbox, Button.kRightStick.value).onTrue(setArmUpIntakeSetpoint);

    // A Button (1) - button to reset arm
    new JoystickButton(xbox, Button.kA.value).onTrue(resetArmEncoderCommand(arm));

    // manually control the arm (dpad left and right)
    new Trigger(() -> xbox.getPOV() == 90).whileTrue(makeSetSpeedGravityCompensationCommand(arm, 0.3))
        .onFalse(makeSetSpeedGravityCompensationCommand(arm, 0));
    new Trigger(() -> xbox.getPOV() == 270).whileTrue(makeSetSpeedGravityCompensationCommand(arm, -0.3))
        .onFalse(makeSetSpeedGravityCompensationCommand(arm, 0));

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

  // computing deadband
  public static double computeDeadband(double x, double deadband) {
    if (Math.abs(x) <= deadband) {
      return 0;
    } else {
      return x;
    }
  }

  private void setUpCubeCommands() {
    setGroundIntakeSetpoint = new SequentialCommandGroup(
        new InstantCommand(() -> {
          setpoint = CommandSelector.CUBE_INTAKE;
        }),
        selectPositionCommand());

    setArmUpIntakeSetpoint = new SequentialCommandGroup(
        new InstantCommand(() -> {
          setpoint = CommandSelector.ARM_UP;
        }),
        selectPositionCommand());

    setShootIntakeSetpoint = new SequentialCommandGroup(
        new InstantCommand(() -> {
          setpoint = CommandSelector.CUBE_SHOOT;
        }),
        selectPositionCommand());
  }

  private void setUpSubsystems() {

    ArmIO armIO;
    IntakeIO intakeIO;

    armIO = new RealArm();
    intakeIO = new RealIntake();

    arm = new Arm(armIO);
    intake = new Intake(intakeIO);

  }

  private void setUpAutonChooser() {
    chooser.setDefaultOption("do nothing", new PrintCommand("i am doing nothing"));
    chooser.addOption("shoot", new IntakeForGivenTime(intake, -0.5, 1));
    chooser.addOption("leave community", new DriveForwardGivenDistance(-4.2, m_driveTrain));
    chooser.addOption("shoot and leave community", new ShootLeaveCommunity(m_driveTrain, intake, arm));
  }

  public static Command makeSetPositionCommand(ProfiledPIDSubsystem base, double target) {
    return new SequentialCommandGroup(
        new ConditionalCommand(new InstantCommand(() -> {
        }), new InstantCommand(() -> base.enable()), () -> base.isEnabled()),
        new InstantCommand(() -> base.setGoal(target), base));
  }

  private Command makeSetSpeedGravityCompensationCommand(Arm a, double speed) {
    return new SequentialCommandGroup(
        new InstantCommand(() -> a.disable()),
        new RunCommand(() -> a.setSpeedGravityCompensation(speed), a));
  }

  public static Command resetArmEncoderCommand(Arm a) {
    Debouncer debouncer = new Debouncer(0.2);
    return new SequentialCommandGroup(
        new InstantCommand(() -> a.disable()),
        new RunCommand(() -> a.setSpeed(0.15)).withTimeout(0.2),
        new RunCommand(() -> a.setSpeed(0.15)).until(() -> debouncer.calculate(Math.abs(a.getEncoderSpeed()) < 0.01)),
        new InstantCommand(() -> a.setPosition(Constants.ArmConstants.INITIAL_OFFSET)),
        makeSetPositionCommand(a, ArmConstants.ARM_UP_ANGLE));
  }

  public enum CommandSelector {
    CUBE_INTAKE,
    CUBE_SHOOT,
    ARM_UP
  }

  private CommandSelector select() {
    return setpoint;
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
