package frc.robot.commands;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.intake.Intake;

//runs the intake at a given speed for a given amount of time

public class IntakeForGivenTime extends CommandBase {

    private final Intake intakeSubsystem;
    private final double speed;
    Timer timer;
    double tm;

    public IntakeForGivenTime(Intake intakeSubsystem, double speed, double time) {
        this.intakeSubsystem = intakeSubsystem;
        this.speed = speed;
        tm = time;
        timer = new Timer();
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        DataLogManager.log("intake time started!");
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        this.intakeSubsystem.setMotor(speed);
    }

    @Override
    public void end(boolean interrupted) {
        DataLogManager.log("Intake time ended!");
        this.intakeSubsystem.setMotor(0.0);
    }

    @Override
    public boolean isFinished() {
        if (timer.get() >= tm)
        {
            return true;
        }
        return false;
    }
}