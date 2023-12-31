package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.util.MotorUtil;

public class RealIntake implements IntakeIO {

    public static CANSparkMax intakeMotorController;
    public static RelativeEncoder intakeEncoder;
    private double slewRate = 0.2;

    public RealIntake()
    {
        intakeMotorController = MotorUtil.createSparkMAX(IntakeConstants.INTAKE_MOTOR_ID, MotorType.kBrushless, 
            Constants.NEO550_CURRENT_LIMIT, false, true, slewRate);

        // initialize motor encoder
        intakeEncoder = intakeMotorController.getEncoder();
    }

    @Override
    public void setMotor(double intakeSpeed) {
        intakeMotorController.set(intakeSpeed);
    }

    public double getCurrent()
    {
        return intakeMotorController.getOutputCurrent();
    }

    @Override
    public double getEncoderSpeed() {
        return intakeEncoder.getVelocity();
    }

    @Override
    public double getEncoderPosition() {
        return intakeEncoder.getPosition();
    }

    @Override
    public void setPosition(double position) {
        intakeEncoder.setPosition(position);
    }

    @Override
    public void setCurrentLimit(int current) {
        intakeMotorController.setSmartCurrentLimit(current);        
    }

    @Override
    public void periodicUpdate() {
        SmartDashboard.putNumber("intake/current (A)", getCurrent());
        SmartDashboard.putNumber("intake/temp (C)", intakeMotorController.getMotorTemperature());        
    }

}
