// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final int NEO550_CURRENT_LIMIT = 30;
  public static final int NEO_CURRENT_LIMIT = 60;
  
  public static class DriveTrainConstants {
    // ***Chassis configuration

    // Distance between centers of right and left wheels on robot
    public static final double TRACK_WIDTH = Units.inchesToMeters(23.0);
    // Distance between front and back wheels on robot
    public static final double TRACK_LENGTH = Units.inchesToMeters(23.0);
    
    // Create swerve kinematics
    public static final SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(
        new Translation2d(TRACK_LENGTH / 2, TRACK_WIDTH / 2),
        new Translation2d(TRACK_LENGTH / 2, -TRACK_WIDTH / 2),
        new Translation2d(-TRACK_LENGTH / 2, TRACK_WIDTH / 2),
        new Translation2d(-TRACK_LENGTH / 2, -TRACK_WIDTH / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double FRONT_LEFT_CHASSIS_ANGULAR_OFFSET = -Math.PI / 2;
    public static final double FRONT_RIGHT_CHASSIS_ANGULAR_OFFSET = 0;
    public static final double REAR_LEFT_CHASSIS_ANGULAR_OFFSET = Math.PI;
    public static final double REAR_RIGHT_CHASSIS_ANGULAR_OFFSET = Math.PI / 2;

    // SPARK MAX CAN IDs
    public static final int FRONT_LEFT_DRIVING_CAN_ID = 11;
    public static final int REAR_LEFT_DRIVING_CAN_ID = 13;
    public static final int FRONT_RIGHT_DRIVING_CAN_ID = 15;
    public static final int REAR_RIGHT_DRIVING_CAN_ID = 17;

    public static final int FRONT_LEFT_TURNING_CAN_ID = 10;
    public static final int REAR_LEFT_TURNING_CAN_ID = 12;
    public static final int FRONT_RIGHT_TURNING_CAN_ID = 14;
    public static final int REAR_RIGHT_TURNING_CAN_ID = 16;

    public static final boolean GYRO_REVERSED = false;
  }

  public static class ModuleConstants {
    // ***Constants for swerve modules

    // Pinion gear teeth drive motor
    public static final double DRIVING_MOTOR_PINION_TEETH = 14;

    // Turning encoder inversion boolean
    public static final boolean TURNING_ENCODER_INVERTED = true;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double DRIVING_MOTOR_FREE_SPEED_RPS = 5676 / 60;
    public static final double WHEEL_DIAMETER_METERS = 0.0762;
    public static final double WHEEL_CIRCUMFERENCE_METERS = WHEEL_DIAMETER_METERS * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
    public static final double DRIVING_MOTOR_REDUCTION = (45.0 * 22) / (DRIVING_MOTOR_PINION_TEETH * 15);
    public static final double DRIVE_WHEEL_FREE_SPEED_RPS = (DRIVING_MOTOR_FREE_SPEED_RPS * WHEEL_CIRCUMFERENCE_METERS)
        / DRIVING_MOTOR_REDUCTION;

    // Drive encoder position and velocity conversion factors (calculated)
    public static final double DRIVING_ENCODER_POSITION_CONVERSION_FACTOR = (WHEEL_DIAMETER_METERS * Math.PI)
    / DRIVING_MOTOR_REDUCTION; // meters
    public static final double DRIVING_ENCODER_VELOCITY_CONVERSION_FACTOR = (WHEEL_DIAMETER_METERS * Math.PI)
    / DRIVING_MOTOR_REDUCTION / 60.0; // meters per second

    // Drive encoder position and velocity conversion factors (determined manually)
    //public static final double DRIVING_ENCODER_POSITION_CONVERSION_FACTOR = 0.0508;
    //public static final double DRIVING_ENCODER_VELOCITY_CONVERSION_FACTOR = 0.0508 / 60;

    // Turning encoder position and velocity conversion factors
    public static final double TURNING_ENCODER_POSITION_CONVERSION_FACTOR = 2 * Math.PI;
    public static final double TURNING_ENCODER_VELOCITY_CONVERSION_FACTOR = 2 * Math.PI / 60.0;

    // Min/max turning encoder PID inputs
    public static final double TURNING_ENCODER_POSITION_PID_MIN_INPUT = 0;
    public static final double TURNING_ENCODER_POSITION_PID_MAX_INPUT = 2 * Math.PI; 

    //driving encoder PIDF controller constants
    public static final double DRIVING_P = 0.04;
    public static final double DRIVING_I = 0;
    public static final double DRIVING_D = 0;
    public static final double DRIVING_FF = 1 / DRIVE_WHEEL_FREE_SPEED_RPS;
    public static final double DRIVING_MIN_OUTPUT = -1;
    public static final double DRIVING_MAX_OUTPUT = 1;

    public static final double TURNING_P = 1;
    public static final double TURNING_I = 0;
    public static final double TURNING_D = 0;
    public static final double TURNING_FF = 0;
    public static final double TURNING_MIN_OUTPUT = -1;
    public static final double TURNING_MAX_OUTPUT = 1;

    // Define motor idle modes
    public static final IdleMode DRIVING_MOTOR_IDLE_MODE = IdleMode.kBrake;
    public static final IdleMode TURNING_MOTOR_IDLE_MODE = IdleMode.kBrake;

    // Define motor current limits
    public static final int DRIVING_MOTOR_CURRENT_LIMIT = 50; // amps
    public static final int TURNING_MOTOR_CURRENT_LIMIT = 20; // amps

    // Encoder offsets times 2pi (the position conversion factor)
    public static final double FRONT_RIGHT_ENCODER_OFFSET = 0.932 * 2 * Math.PI;
    public static final double FRONT_LEFT_ENCODER_OFFSET = 0.359 * 2 * Math.PI;
    public static final double REAR_LEFT_ENCODER_OFFSET = 0.517 * 2 * Math.PI;
    public static final double REAR_RIGHT_ENCODER_OFFSET = 0.575 * 2 * Math.PI;
  }

  public static class XboxConstants {
    public static final int FWD_AXIS = 1;
    public static final int STR_AXIS = 0;
    public static final int RCW_AXIS = 4;
  }

  public static final class IntakeConstants {

    public static final int INTAKE_MOTOR_ID = 2;
    public static final double INTAKE_SLEW_RATE = 10;
    public static final double CONE_OUT_SPEED = 1.0;
    public static final double CUBE_IN_SPEED = 0.6;
    public static final double CUBE_OUT_SPEED = -1.0;
    public static final int CONE_IN_CURRENT = 30;
    public static final int CUBE_IN_CURRENT = 25;
    public static final int OUT_CURRENT = 30;
}

  public static final class ArmConstants {
    public static final double RADIANS_PER_REVOLUTION = 0.0837;
    // initial offset is 0.711 + (0.287) - (0.308)
    public static final double INITIAL_OFFSET = 0.660;

    //can be 2 degrees off from goal setpoints and still considered at goal; made higher so arm.atGoal() in placeConeOnNode cmd will execute in auton
    public static final double ANGLE_TOLERANCE_AUTON = Units.degreesToRadians(2);
    public static final int ARM_MOTOR_ID = 1;

    public static final double CUBE_INTAKE_ANGLE = -1.7; //TODO: tune
    public static final double CUBE_SHOOT_ANGLE = -0.75; //TODO: tune
    public static final double ARM_UP_ANGLE = 0;


  }

  public static final class VisionConstants {
    public static final String CAMERA_NAME = "2399";
  }
}
