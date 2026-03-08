package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.FuelConstants.FEEDER_ROLLER_ID;
import static frc.robot.Constants.FuelConstants.INTAKE_FEEDER_VOLTAGE;
import static frc.robot.Constants.FuelConstants.INTAKE_MAIN_VOLTAGE;
import static frc.robot.Constants.FuelConstants.LAUNCH_FEEDER_VOLTAGE;
import static frc.robot.Constants.FuelConstants.LAUNCH_MAIN_VOLTAGE;
import static frc.robot.Constants.FuelConstants.MAIN_ROLLER_ID;
import static frc.robot.Constants.FuelConstants.OUTTAKE_FEEDER_VOLTAGE;
import static frc.robot.Constants.FuelConstants.OUTTAKE_MAIN_VOLTAGE;

import java.util.function.DoubleSupplier;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CANFuelSubsystem extends SubsystemBase {

  private final SparkMax mainRoller;
  private final VictorSP feederRoller;

  /* 
  private final SimpleMotorFeedforward m_shooterFeedforward = new SimpleMotorFeedforward(
      kSVolts, kVVoltSecondsPerRotation);

  private final PIDController m_shooterFeedback = new PIDController(kP, 0.0, 0.0);

  private final Encoder m_shooterEncoder = new Encoder(
      ENCODER_CHANNEL_A,
      ENCODER_CHANNEL_B,true
  /* */



  // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
  private final MutVoltage m_appliedVoltage = Volts.mutable(0);
  // Mutable holder for unit-safe linear distance values, persisted to avoid
  // reallocation.
  private final MutAngle m_angle = Radians.mutable(0);
  // Mutable holder for unit-safe linear velocity values, persisted to avoid
  // reallocation.
  private final MutAngularVelocity m_velocity = RadiansPerSecond.mutable(0);

  //private final SysIdRoutine m_sysIdRoutine;

  public CANFuelSubsystem() {

    var config = new SparkMaxConfig();
    config.inverted(true);

    mainRoller = new SparkMax(MAIN_ROLLER_ID, MotorType.kBrushed);
    mainRoller.configure(config, ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
    feederRoller = new VictorSP(FEEDER_ROLLER_ID);
    
    feederRoller.setInverted(true);
  }

   /* // TEST SysId

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
                          mainRoller.getAppliedOutput() * RobotController.getBatteryVoltage(), Volts))
                  .angularPosition(m_angle.mut_replace(m_shooterEncoder.getDistance(), Rotations))
                  .angularVelocity(
                      m_velocity.mut_replace(m_shooterEncoder.getRate(), RotationsPerSecond));
            },
            // Tell SysId to make generated commands require this subsystem, suffix test
            // state in
            // WPILog with this subsystem's name ("shooter")
            this));
  }

  // Test Command Shoot with PID
  public Command shootCommand(double setPointRotationsPerSecond) {
    return Commands.parallel(
        run(() -> {
          mainRoller.setVoltage(
              m_shooterFeedforward.calculate(setPointRotationsPerSecond)
                  + m_shooterFeedback.calculate(
                      m_shooterEncoder.getRate(), setPointRotationsPerSecond));
        }),

        Commands.waitUntil(m_shooterFeedback::atSetpoint)
            .andThen(() -> {
              feederRoller.setVoltage(7);

              // m_shooterEncoder.reset();
              }));
  }

  // SysId Config


  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
  }*/

  // Métodos

  public void intake() {
    mainRoller.setVoltage(INTAKE_MAIN_VOLTAGE);
    feederRoller.setVoltage(INTAKE_FEEDER_VOLTAGE);
  }

  public void buffer() {
    feederRoller.setVoltage(LAUNCH_FEEDER_VOLTAGE);
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

   public void periodic() {

  }

  public Command launchCommand(DoubleSupplier xSpeed) {
    return this.run(() -> launch());
  }

  public void feederRollback(){
    feederRoller.set(-1);

  }

  public void feederSlowRollback(){
    feederRoller.set(-0.7);
  }

  public Command shoot(DoubleSupplier xSpeed) {
    SmartDashboard.putNumber("Velocity", xSpeed.getAsDouble());
    return this.run(
        () -> mainRoller.set(xSpeed.getAsDouble())); 

  
  }
}


