  package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.CANFuelCommand;

public class CANFuelSubsystem extends SubsystemBase {

  private final SparkMax mainRoller;
  private final SparkMax feederRoller;

  public CANFuelSubsystem() {

    mainRoller = new SparkMax(Constants.FuelConstants.MAIN_ROLLER_ID, MotorType.kBrushed);
    feederRoller = new SparkMax(Constants.FuelConstants.FEEDER_ROLLER_ID, MotorType.kBrushed);

    /*SparkMaxConfig maxConfig = new SparkMaxConfig();  
    maxConfig.smartCurrentLimit(Constants.FuelConstants.MAX_CURRENT_LIMIT);
    maxRoller.configure(maxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters
    );*/
  }

  // INTAKE da GamePiece

  /*private void setVoltageWithCurrentLimit(SparkMax motor, double voltage, double maxCurrent){
    if (motor.getOutputCurrent() > maxCurrent) {
        motor.setVoltage(0.0); // ou reduz tensão
    } else {
        motor.setVoltage(voltage);
    }
  }*/

  private int mainRollerOvercurrentCount = 0;
  private int feederRollerOvercurrentCount = 0;
  private static final int MAX_OVERCURRENT_CYCLES = 10; // ~200ms
  
  private boolean mainRollerJammed = false;
  private boolean feederRollerJammed = false;

  public void intake() {
    mainRoller.setVoltage(Constants.FuelConstants.INTAKE_MAIN_MAX_AMPER);
    feederRoller.setVoltage(Constants.FuelConstants.INTAKE_FEEDER_MAX_AMPER);
  }

  public void launch() {
    mainRoller.setVoltage(Constants.FuelConstants.LAUNCH_MAIN_VOLTAGE);
    feederRoller.setVoltage(Constants.FuelConstants.LAUNCH_FEEDER_VOLTAGE);
  }


  public void outtake() {
    mainRoller.setVoltage(Constants.FuelConstants.OUTTAKE_MAIN_VOLTAGE);
    feederRoller.setVoltage(Constants.FuelConstants.OUTTAKE_FEEDER_VOLTAGE);
  }

  public void stop() {
    mainRoller.stopMotor();
    feederRoller.stopMotor();
  }
   
  @Override
  public void periodic(){}
}
