// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
    //robot half width and half length in inches, determined by the full width/length divided by 2
    public static final double HALF_LENGTH = 23/2.0;
    public static final double HALF_WIDTH = 23/2.0;

    //robot radius in inches 
    public static final double RADIUS = Math.sqrt(HALF_LENGTH * HALF_LENGTH + HALF_WIDTH * HALF_WIDTH); 

    //angle formed by the half length and radius of the robot, in radians 
    public static final double THETA = Math.acos(HALF_LENGTH / RADIUS); 

    public static final double kTurningEncoderPositionPIDMinInput = 0;
    public static final double kTurningEncoderPositionPIDMaxInput = 2 * Math.PI; 



    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
      new Translation2d(HALF_LENGTH, HALF_WIDTH),
      new Translation2d(HALF_LENGTH, -HALF_WIDTH),
      new Translation2d(-HALF_LENGTH, HALF_WIDTH),
      new Translation2d(-HALF_LENGTH, -HALF_WIDTH));


    //encoder offsets times 2pi (the position conversion factor)
    public static final double ENCODER1_OFFSET = 0.932 * 2 * Math.PI;
    public static final double ENCODER2_OFFSET = 0.359 * 2 * Math.PI;
    public static final double ENCODER3_OFFSET = 0.517 * 2 * Math.PI;
    public static final double ENCODER4_OFFSET = 0.575 * 2 * Math.PI;
  

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
