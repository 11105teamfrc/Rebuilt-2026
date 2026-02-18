package frc.robot;

public final class Constants {
  
  public static final class DriveConstants {

    // Motor controller IDs for drivetrain motors

    public static final int LEFT_LEADER_ID = 3;
    public static final int LEFT_FOLLOWER_ID = 5;
    public static final int RIGHT_LEADER_ID = 6;
    public static final int RIGHT_FOLLOWER_ID = 1;

    public static final int DRIVE_MOTOR_CURRENT_LIMIT = 40;

    // Encoders (CANAL)
    public static final int LEFT_ENCODER_CHANNEL_A = 0;
    public static final int LEFT_ENCODER_CHANNEL_B = 1;

    public static final int RIGHT_ENCODER_CHANNEL_A = 2;
    public static final int RIGHT_ENCODER_CHANNEL_B = 3;

    // Encoders LIMIT PER PULSE

    public static final double DISTANCE_PER_PULSE =
    (2 * Math.PI * 0.0762) / 600.0;
    
  }

  public static final class FuelConstants {

    // CAN IDs  
    public static final int MAIN_ROLLER_ID = 4; // intake + launch
    public static final int FEEDER_ROLLER_ID = 0; // feeder + outtake - VictorSP

    // Current Limits (A)
    public static final double CURRENT_LIMIT = 40.0;

    // Voltages (VELOCIDADE)
    public static final double INTAKE_MAIN_VOLTAGE  = -12.0;
    public static final double INTAKE_FEEDER_VOLTAGE = -10.0;

    public static final double LAUNCH_MAIN_VOLTAGE = -9.0;
    public static final double LAUNCH_FEEDER_VOLTAGE = 10.6;

    public static final double OUTTAKE_MAIN_VOLTAGE = 6.0;
    public static final double OUTTAKE_FEEDER_VOLTAGE = 1.0;
    
   }

  public static final class OperatorConstants {
   
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;

    public static final double DRIVE_SCALING = .7;
    public static final double ROTATION_SCALING = .8;
  }

  public static final class ShooterConstants {

    public static final double kSVolts = 0.05;
    public static final double kVVoltSecondsPerRotation = 12.0 / 88.5;

    public static final double kP = 0.1;

    public static final double kShooterToleranceRPS = 50;
    public static final double kEncoderDistancePerPulse = 1.0 / 600.0;

    public static final int ENCODER_CHANNEL_A = 4; // int = inteiro
    public static final int ENCODER_CHANNEL_B = 5; // double = numero com ponto

    public static int[] kEncoderPorts = {2, 3};
    public static final boolean kEncoderReversed = false;

    // teste
    public static double kForward = 5.0;
    public static double kReverse = -5.0;

  }
}
