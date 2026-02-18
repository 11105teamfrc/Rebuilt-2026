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
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import static frc.robot.Constants.ShooterConstants.*;

public class CANFuelSubsystem extends SubsystemBase {

  private final SparkMax mainRoller;
  private final VictorSP feederRoller;

  private final SimpleMotorFeedforward m_shooterFeedforward = new SimpleMotorFeedforward(
      kSVolts, kVVoltSecondsPerRotation);

  private final PIDController m_shooterFeedback = new PIDController(kP, 0.0, 0.0);

  private final Encoder m_shooterEncoder = new Encoder(
      ENCODER_CHANNEL_A,
      ENCODER_CHANNEL_B);

  // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
  private final MutVoltage m_appliedVoltage = Volts.mutable(0);
  // Mutable holder for unit-safe linear distance values, persisted to avoid
  // reallocation.
  private final MutAngle m_angle = Radians.mutable(0);
  // Mutable holder for unit-safe linear velocity values, persisted to avoid
  // reallocation.
  private final MutAngularVelocity m_velocity = RadiansPerSecond.mutable(0);

  private final SysIdRoutine m_sysIdRoutine;

  public CANFuelSubsystem() {

    mainRoller = new SparkMax(MAIN_ROLLER_ID, MotorType.kBrushed);
    feederRoller = new VictorSP(FEEDER_ROLLER_ID);

    feederRoller.setInverted(true);

    m_shooterFeedback.setTolerance(kShooterToleranceRPS);
    m_shooterEncoder.setDistancePerPulse(kEncoderDistancePerPulse);

    m_shooterEncoder.reset();

    // TEST PID + SysId

    m_sysIdRoutine = new SysIdRoutine(
        // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
        new SysIdRoutine.Config(),
        new SysIdRoutine.Mechanism(
            // Tell SysId how to plumb the driving voltage to the motor(s).
            mainRoller::setVoltage,
            // Tell SysId how to record a frame of data for each motor on the mechanism
            // being
            // characterized.
            log -> {
              // Record a frame for the shooter motor.
              log.motor("shooter-wheel")
                  .voltage(
                      m_appliedVoltage.mut_replace(
                          mainRoller.get() * RobotController.getBatteryVoltage(), Volts))
                  .angularPosition(m_angle.mut_replace(m_shooterEncoder.getDistance(), Rotations))
                  .angularVelocity(
                      m_velocity.mut_replace(m_shooterEncoder.getRate(), RotationsPerSecond));
            },
            // Tell SysId to make generated commands require this subsystem, suffix test
            // state in
            // WPILog with this subsystem's name ("shooter")
            this));
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
            .andThen(() -> feederRoller.set(1)));
  }

  // SysId Config

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
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

  public void setFeederRoller() {
    feederRoller.set(LAUNCH_FEEDER_VOLTAGE);
  }

  public void periodic() {
    SmartDashboard.putBoolean("Is enconder connected", m_shooterEncoder.getStopped());
    SmartDashboard.putNumber("Fuel State (m)", m_shooterEncoder.getDistance());
  }

}
