// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

public final class Constants {

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
  }

  public static class DrivetrainConstants {
    public static final int FRONT_LEFT = 1;
    public static final int FRONT_RIGHT = 2;
    public static final int BACK_LEFT = 3;
    public static final int BACK_RIGHT = 4;

    // Motor Settings
    public static final IdleMode IDLE_MODE = IdleMode.kBrake;
    public static final int CURRENT_LIMIT = 40;

    public static final boolean RIGHT_INVERTED = true;

  }

  public static class DriverControls {

    public static final int LEFT_Y = 1;
    public static final int RIGHT_Y = 4;

  }

  public static class OperatorControls {
    public static final int LEFT_Y = 1;
    public static final int RIGHT_Y = 4;
  }

  public static class ShooterConstants {

    public static final int MOTOR_ID = 5;
    public static final double SHOOT_POWER = 0.5;

  }

  public static class OperatorButtons {
    public static final int SHOOT = 3;
  }

  public static final String Drivetrain = null; 

}
