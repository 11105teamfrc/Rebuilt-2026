package frc.robot;



public final class Constants {

  public static final class DriveConstants {

    // Motor controller IDs for drivetrain motors

    public static final int LEFT_LEADER_ID = 1;
    public static final int LEFT_FOLLOWER_ID = 2;
    public static final int RIGHT_LEADER_ID = 3;
    public static final int RIGHT_FOLLOWER_ID = 4;

    public static final int DRIVE_MOTOR_CURRENT_LIMIT = 60;
  }

  public static final class FuelConstants {

    // CAN IDs  

    public static final int MAIN_ROLLER_ID = 1; // intake + launch
    public static final int FEEDER_ROLLER_ID = 5; // feeder + outtake 

    // Current Limits (A)

    public static final double INTAKE_MAIN_VOLTAGE = 60;
    public static final double INTAKE_FEEDER_VOLTAGE = 60;

    // Voltages (V)

    public static final double INTAKE_MAIN_VOLTAGE  = -12;
    public static final double INTAKE_FEEDER_VOLTAGE = 10;

    public static final double LAUNCH_MAIN_VOLTAGE = 9;
    public static final double LAUNCH_FEEDER_VOLTAGE = 10.6;

    public static final double OUTTAKE_MAIN_VOLTAGE = -6;
    public static final double OUTTAKE_FEEDER_VOLTAGE = 1;

   }

  public static final class OperatorConstants {
   
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;

    public static final double DRIVE_SCALING = .7;
    public static final double ROTATION_SCALING = .8;
  }
}
