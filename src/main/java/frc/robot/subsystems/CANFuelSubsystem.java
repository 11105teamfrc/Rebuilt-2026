package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class CANFuelSubsystem extends SubsystemBase {

  private final SparkMax mainRoller;
  private final SparkMax feederRoller;

  public CANFuelSubsystem() {

    mainRoller = 
      new SparkMax(Constants.FuelConstants.MAIN_ROLLER_ID, MotorType.kBrushed);
    feederRoller = 
      new SparkMax(Constants.FuelConstants.FEEDER_ROLLER_ID, MotorType.kBrushed);

  }

  public void intake() {
    mainRoller.setVoltage(Constants.FuelConstants.INTAKE_MAIN_VOLTAGE);
    feederRoller.setVoltage(Constants.FuelConstants.INTAKE_FEEDER_VOLTAGE);
  }

  public void launch() {
    mainRoller.setVoltage(Constants.FuelConstants.LAUNCH_MAIN_VOLTAGE);
    feederRoller.setVoltage(Constants.FuelConstants.LAUNCH_FEEDER_VOLTAGE);
  }

  public void outtake() {
    mainRoller.setVoltage(Constants.FuelConstants.OUTTAKE_MAIN_VOLTAGE);
    feederRoller.setVoltage(Constants.FuelConstants.OUTTAKE_FEEDER_VOLTAGE);
  }

  public void startLaunchOnly() {
    mainRoller.setVoltage(Constants.FuelConstants.LAUNCH_MAIN_VOLTAGE);
    feederRoller.stopMotor();
  }

  public void startFeederOnly() {
    feederRoller.setVoltage(Constants.FuelConstants.LAUNCH_FEEDER_VOLTAGE);
  }

  public void stop() {
    mainRoller.stopMotor();
    feederRoller.stopMotor();
  }
   
  @Override
  public void periodic(){
    SmartDashboard.putString("Fuel State", "INTAKE / LAUNCH / OUTTAKE");
  }

}
