// Copyright (c) FIRST and other https://software-metadata.revrobotics.com WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller.Axis;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DrivetrainSubystem extends SubsystemBase {

  private final SparkMax frontLeft = new SparkMax(1, MotorType.kBrushed);
  private final SparkMax frontRight = new SparkMax(2, MotorType.kBrushed);

  private final SparkMax backLeft = new SparkMax(3, MotorType.kBrushed);
  private final SparkMax backRight = new SparkMax(4, MotorType.kBrushed);

  private final DifferentialDrive drive;

  double speed;

  
  public DrivetrainSubystem() {

  SparkBaseConfig frontLeftConfig = new SparkMaxConfig()
      .idleMode(Constants.DrivetrainConstants.IDLE_MODE)
      .smartCurrentLimit(Constants.DrivetrainConstants.CURRENT_LIMIT);

  SparkBaseConfig frontRightConfig = new SparkMaxConfig()
      .idleMode(Constants.DrivetrainConstants.IDLE_MODE)
      .smartCurrentLimit(Constants.DrivetrainConstants.CURRENT_LIMIT)
      .inverted(Constants.DrivetrainConstants.RIGHT_INVERTED);

  SparkBaseConfig backLeftConfig = new SparkMaxConfig()
      .idleMode(Constants.DrivetrainConstants.IDLE_MODE)
      .smartCurrentLimit(Constants.DrivetrainConstants.CURRENT_LIMIT)
      .follow(frontLeft);

  SparkBaseConfig backRightConfig = new SparkMaxConfig()
      .idleMode(Constants.DrivetrainConstants.IDLE_MODE)
      .smartCurrentLimit(Constants.DrivetrainConstants.CURRENT_LIMIT)
      .follow(frontRight);

  frontLeft.configure(frontLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  frontRight.configure(frontRightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  backLeft.configure(backLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  backRight.configure(backRightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Só os masters no DifferentialDrive

    drive = new DifferentialDrive(frontLeft, frontRight);

  }

  public void tankDrive(double left, double right) {
    drive.tankDrive(left, right);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}