package frc.robot.subsystems;

import static frc.robot.Constants.DriveConstants.DISTANCE_PER_PULSE;
import static frc.robot.Constants.DriveConstants.DRIVE_MOTOR_CURRENT_LIMIT;
import static frc.robot.Constants.DriveConstants.LEFT_ENCODER_CHANNEL_A;
import static frc.robot.Constants.DriveConstants.LEFT_ENCODER_CHANNEL_B;
import static frc.robot.Constants.DriveConstants.LEFT_FOLLOWER_ID;
import static frc.robot.Constants.DriveConstants.LEFT_LEADER_ID;
import static frc.robot.Constants.DriveConstants.RIGHT_ENCODER_CHANNEL_A;
import static frc.robot.Constants.DriveConstants.RIGHT_ENCODER_CHANNEL_B;
import static frc.robot.Constants.DriveConstants.RIGHT_FOLLOWER_ID;
import static frc.robot.Constants.DriveConstants.RIGHT_LEADER_ID;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CANDriveSubsystem extends SubsystemBase {

  private final SparkMax leftLeader;
  private final SparkMax leftFollower;
  private final SparkMax rightLeader;
  private final SparkMax rightFollower;

  private final DifferentialDrive drive;

  private final Encoder leftEncoder;
  private final Encoder rightEncoder;

  public CANDriveSubsystem() {
    
    // Create/ Set brushed motors for drive

    leftLeader = new SparkMax(
      LEFT_LEADER_ID, MotorType.kBrushed
    );

    leftFollower = new SparkMax(
      LEFT_FOLLOWER_ID, MotorType.kBrushed
    );

    rightLeader = new SparkMax(
      RIGHT_LEADER_ID, MotorType.kBrushed
    );

    rightFollower = new SparkMax(
      RIGHT_FOLLOWER_ID, MotorType.kBrushed
    );

    // Set up differential drive class
    drive = new DifferentialDrive(leftLeader, rightLeader);
    drive.setDeadband(0.02);
 
    // Set timeout
    leftLeader.setCANTimeout(250);
    rightLeader.setCANTimeout(250);
    leftFollower.setCANTimeout(250);
    rightFollower.setCANTimeout(250);

    // Config Break (Motor)
    SparkMaxConfig config = new SparkMaxConfig();
    config.voltageCompensation(12);
    config.smartCurrentLimit(DRIVE_MOTOR_CURRENT_LIMIT);

    // Seguidores (Followers)
    config.follow(leftLeader);
    leftFollower.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    config.follow(rightLeader);
    rightFollower.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Leaders (Motores)
    config.disableFollowerMode();
    rightLeader.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Inversão de motores DriveTrain
    leftLeader.setInverted(false);
    rightLeader.setInverted(true);

    // ENCODER

    leftEncoder = new Encoder(
      LEFT_ENCODER_CHANNEL_A,
      LEFT_ENCODER_CHANNEL_B
    ); 
    
    rightEncoder = new Encoder(
      RIGHT_ENCODER_CHANNEL_A,
      RIGHT_ENCODER_CHANNEL_B
    );

    leftEncoder.setDistancePerPulse(DISTANCE_PER_PULSE);
    rightEncoder.setDistancePerPulse(DISTANCE_PER_PULSE);

    // Inverter Encoders (Fisico)
    leftEncoder.setReverseDirection(false);
    rightEncoder.setReverseDirection(true);


    leftEncoder.reset();
    rightEncoder.reset();

}

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Left Encoder (m)", leftEncoder.getDistance());
    SmartDashboard.putNumber("Right Encoder (m)", rightEncoder.getDistance());
  }

  public void resetEncoders() {
     leftEncoder.reset();
     rightEncoder.reset();
  }

  public double getAverageDistanceMeters() {
    return (leftEncoder.getDistance() + rightEncoder.getDistance()) / 2.0;
  }

  public void arcadeDrive(double xSpeed, double zRotation) {
    drive.arcadeDrive(xSpeed, zRotation);
}

  public Command driveArcade(DoubleSupplier xSpeed, DoubleSupplier zRotation) {
    return this.run(
        () -> drive.arcadeDrive(xSpeed.getAsDouble(), -zRotation.getAsDouble()));
  }
}