// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubystem extends SubsystemBase {

        private SparkMax shooterMotor;
    
      public ShooterSubystem() {
        shooterMotor = new SparkMax(
          ShooterConstants.MOTOR_ID,
          MotorType.kBrushed // Motor CIM
    );

  }

  public void shoot() {
    shooterMotor.set(ShooterConstants.SHOOT_POWER);
  }

  public void stop() {
    shooterMotor.stopMotor();
  }
}
