package frc.robot.subsystems;

import static frc.robot.Constants.FuelConstants.FEEDER_ROLLER_ID;
import static frc.robot.Constants.FuelConstants.INTAKE_FEEDER_VOLTAGE;
import static frc.robot.Constants.FuelConstants.INTAKE_MAIN_VOLTAGE;
import static frc.robot.Constants.FuelConstants.LAUNCH_FEEDER_VOLTAGE;
import static frc.robot.Constants.FuelConstants.LAUNCH_MAIN_VOLTAGE;
import static frc.robot.Constants.FuelConstants.MAIN_ROLLER_ID;
import static frc.robot.Constants.FuelConstants.OUTTAKE_FEEDER_VOLTAGE;
import static frc.robot.Constants.FuelConstants.OUTTAKE_MAIN_VOLTAGE;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.ShooterConstants.*;


public class CANFuelSubsystem extends SubsystemBase {

  private final SparkMax mainRoller;
  private final VictorSP feederRoller;

  private final SimpleMotorFeedforward m_shooterFeedforward =
      new SimpleMotorFeedforward(
          kSVolts, kVVoltSecondsPerRotation);

  private final PIDController m_shooterFeedback =
   new PIDController(kP, 0.0, 0.0);

  private final Encoder m_shooterEncoder =
    new Encoder(
      ENCODER_CHANNEL_A,
      ENCODER_CHANNEL_B
    );


  public CANFuelSubsystem() {

    mainRoller = 
      new SparkMax(MAIN_ROLLER_ID, MotorType.kBrushed);
    feederRoller = 
      new VictorSP(FEEDER_ROLLER_ID);

  m_shooterFeedback.setTolerance(kShooterToleranceRPS);
  m_shooterEncoder.setDistancePerPulse(kEncoderDistancePerPulse);

}

  public Command shootCommand(double setpointRotationsPerSecond) {
    return Commands.parallel(
      run(() -> {
        mainRoller.set(
          m_shooterFeedforward.calculate(setpointRotationsPerSecond)
            + m_shooterFeedback.calculate(
                m_shooterEncoder.getRate(), setpointRotationsPerSecond));
    }),

    Commands.waitUntil(m_shooterFeedback::atSetpoint)
    .andThen(() -> feederRoller.set(1))
    );
  }

  public void intake() {
    mainRoller.setVoltage(INTAKE_MAIN_VOLTAGE);
    feederRoller.setVoltage(INTAKE_FEEDER_VOLTAGE);
  }

  public void launch() {
    mainRoller.setVoltage(LAUNCH_MAIN_VOLTAGE);
    feederRoller.setVoltage(LAUNCH_FEEDER_VOLTAGE);
  }

  public void outtake() {
    mainRoller.setVoltage(OUTTAKE_MAIN_VOLTAGE);
    feederRoller.setVoltage(OUTTAKE_FEEDER_VOLTAGE);
  }

  public void stop() {
    mainRoller.stopMotor();
    feederRoller.stopMotor();
  }

  public void shoot() {
    mainRoller.setVoltage(LAUNCH_MAIN_VOLTAGE);
  }

  public void setFeederRoller () {
  feederRoller.set(LAUNCH_FEEDER_VOLTAGE);
  }
   
  @Override
  public void periodic (){
    SmartDashboard.putString("Fuel State", "INTAKE / LAUNCH / OUTTAKE");
  }

}
