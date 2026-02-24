package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {

    private final SparkMax climberMotor =
        new SparkMax(
            ClimberConstants.CLIMBER_ROLLER_ID,
            MotorType.kBrushless
        );

    public ClimberSubsystem() {

        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake);

        config.openLoopRampRate(ClimberConstants.RAMP_RATE);
        config.smartCurrentLimit(40);

        climberMotor.configure(
            config,
            SparkMax.ResetMode.kResetSafeParameters,
            SparkMax.PersistMode.kPersistParameters
            );

    }

    public Command climbUp(DoubleSupplier xSpeed) {
        return this.run(
            () -> climberMotor.set(xSpeed.getAsDouble()
        ));
    }

    public Command climbDown(DoubleSupplier xSpeed) {
        return this.run(
            () -> climberMotor.set(xSpeed.getAsDouble()
        ));
    }

    public void stop() {
        climberMotor.stopMotor();
    }
}