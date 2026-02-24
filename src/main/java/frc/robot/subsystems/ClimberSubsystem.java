package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {

    private final CANSparkMax climberMotor =
        new CANSparkMax(
            ClimberConstants.CLIMBER_ROLLER_ID,
            MotorType.kBrushless
        );

    public ClimberSubsystem() {

        climberMotor.restoreFactoryDefaults();

        climberMotor.setIdleMode(IdleMode.kBrake);

        climberMotor.setOpenLoopRampRate(
            ClimberConstants.RAMP_RATE
        );
    }

    public void climbUp() {
        climberMotor.set(
            ClimberConstants.CLIMB_UP_SPEED
        );
    }

    public void climbDown() {
        climberMotor.set(
            ClimberConstants.CLIMB_DOWN_SPEED
        );
    }

    public void stop() {
        climberMotor.stopMotor();
    }
}