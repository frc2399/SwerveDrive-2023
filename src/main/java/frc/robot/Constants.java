// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class DriveTrainConstants {
    //robot half width and half length in inches, determined by the full width/length divided by 2
    public static final double HALF_LENGTH = 23/2.0;
    public static final double HALF_WIDTH = 23/2.0;

    //robot radius in inches 
    public static final double RADIUS = Math.sqrt(HALF_LENGTH * HALF_LENGTH + HALF_WIDTH * HALF_WIDTH); 

    //angle formed by the half length and radius of the robot, in radians 
    public static final double THETA = Math.acos(HALF_LENGTH / RADIUS); 
  }
}
