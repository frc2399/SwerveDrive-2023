package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.DriveForwardGivenDistance;
import frc.robot.commands.IntakeForGivenTime;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.intake.Intake;

public class ShootLeaveCommunity extends SequentialCommandGroup {

    public ShootLeaveCommunity(DriveTrain driveTrain, Intake intake, Arm arm)
    {
        addCommands(
            RobotContainer.resetArmEncoderCommand(arm),
            new IntakeForGivenTime(intake, -0.5, 1.0),
            new DriveForwardGivenDistance(-4.2, driveTrain)

        );
    }

    
}
