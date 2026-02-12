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

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CANFuelSubsystem extends SubsystemBase {

  private final SparkMax mainRoller;
  private final SparkMax feederRoller;

  public CANFuelSubsystem() {
    mainRoller = 
      new SparkMax(MAIN_ROLLER_ID, MotorType.kBrushed);
    feederRoller = 
      new SparkMax(FEEDER_ROLLER_ID, MotorType.kBrushed);

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

  public void gabriela() {
    mainRoller.setVoltage(LAUNCH_MAIN_VOLTAGE);
  }

  public void setFeederRoller () {
  feederRoller.set(LAUNCH_FEEDER_VOLTAGE);
  }


   
  @Override
  public void periodic (){
    SmartDashboard.putString("Fuel State", "INTAKE / LAUNCH / OUTTAKE");
  }

}
