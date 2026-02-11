package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.FuelConstants.*;

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
   
  @Override
  public void periodic(){
    SmartDashboard.putString("Fuel State", "INTAKE / LAUNCH / OUTTAKE");
  }

}
