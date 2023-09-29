package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DataLogManager;
// import edu.wpi.first.wpilibj.interfaces.*;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

/**
 * Drives the robot forward a given distance (relative to the robot's position upon initialization), and a given speed
 * only used for shuffleboard testing, not used in any commands
 **/

public class DriveForwardGivenDistance extends CommandBase {

    //insantiate global variables
    double currentPosition1;
    double currentPosition2;
    double currentPosition3;
    double currentPosition4;

    double initialPosition1;
    double initialPosition2;
    double initialPosition3;
    double initialPosition4;

    double targetDistanceMeters;
    double newTargetDistance;
    DriveTrain m_driveTrain;
    double distanceTravelled;
    double error;
    private SlewRateLimiter driveLimiter;

    
 
	public DriveForwardGivenDistance(double targetDistanceMeters, DriveTrain subsystem) {
        
        //initialize variables
        this.targetDistanceMeters = targetDistanceMeters;
        m_driveTrain = subsystem;
        addRequirements(m_driveTrain);

    }
    

	// Called just before this Command runs the first time
    @Override
    public void initialize() {
        
        DriveTrain.setWheelAngles(0, 0, 0, 0);

        // Sets the current position to where robot is starting
        initialPosition1 = m_driveTrain.driveEncoder1.getPosition();
        initialPosition2 = m_driveTrain.driveEncoder2.getPosition();
        initialPosition3 = m_driveTrain.driveEncoder3.getPosition();
        initialPosition4 = m_driveTrain.driveEncoder4.getPosition();
        //DataLogManager.log("starting current position " + currentPosition);
        DataLogManager.log("DriveForwardGivenDistance started");

        
        // find distance robot needs to travel to from its current position
        //newTargetDistance = currentPosition + targetDistanceMeters;

        //slew rate limiter to remove aggression :(
        this.driveLimiter = new SlewRateLimiter(0.75);

    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {

        // Get the average position between the left side of the drivetrain and the right side
        currentPosition1 = m_driveTrain.driveEncoder1.getPosition();
        currentPosition2 = m_driveTrain.driveEncoder2.getPosition();
        currentPosition3 = m_driveTrain.driveEncoder3.getPosition();
        currentPosition4 = m_driveTrain.driveEncoder4.getPosition();
        //SmartDashboard.putNumber("currentPosition", currentPosition);

        distanceTravelled = (Math.abs(currentPosition1 - initialPosition1) + Math.abs(currentPosition2 - initialPosition2)
            + Math.abs(currentPosition3 - initialPosition3) + Math.abs(currentPosition4 - initialPosition4)) / 4;

        //signum is to make the error trend towards zero
        error = targetDistanceMeters - (Math.signum(targetDistanceMeters) * distanceTravelled);

        double outputSpeed = (1 * error);
        outputSpeed = MathUtil.clamp(outputSpeed, -0.3, 0.3);
        outputSpeed  = driveLimiter.calculate(outputSpeed);

        m_driveTrain.setSpeed(outputSpeed, 0, 0);
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        // increased error tolerance so the command will finish in auton
        double butteryErrorTolerance = 0.05;
        // SmartDashboard.getNumber("Error Tolerance Distance", 0.5);
        //SmartDashboard.putNumber("distance bt td and cp", Math.abs(newTargetDistance - currentPosition));
        // System.out.println("distance bt td and cp " +  Math.abs(td - cp));

        if (Math.abs(error) <= butteryErrorTolerance)
        {
            return true;
        }
        return false;
        
    }

    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
    
        m_driveTrain.setSpeed(0, 0, 0);

        DataLogManager.log("DriveForwardGivenDistance ended");
    }
}