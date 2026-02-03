  package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.CANFuelCommand;

public class CANFuelSubystem extends SubsystemBase {

  private final SparkMax mainRoller;

  private final SparkMax feederRoller;

    public CANFuelSubystem() {
    
    mainRoller = new SparkMax(Constants.FuelConstants.MAIN_ROLLER_ID, MotorType.kBrushed);
    feederRoller = new SparkMax(Constants.FuelConstants.FEEDER_ROLLER_ID, MotorType.kBrushed);

    SparkMaxConfig maxConfig = new SparkMaxConfig();
    mainConfig.smartCurrentLimit(Constants.FuelConstants.MAIN_CURRENT_LIMIT);
    mainRoller.configure(
      mainConfig,
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters
    );

    SparkMaxConfig mainConfig = new SparkMaxConfig();
    mainConfig.smartCurrentLimit(Constants.FuelConstants.MAIN_CURRENT_LIMIT);
    mainRoller.configure(
      mainConfig,
      ResetMode.kResetSafeParameters,
      ResetMode.kResetSafeParameters
    );

    SparkMaxConfig feederConfig = new SparkMaxConfig();
    feederConfig.smartCurrentLimit(Constants.FuelConstants.FEEDER_CURRENT_LIMIT);
    feederRoller.configure(
      feederConfig,
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters
    );
    
 }
}

  // INTAKE da GamePiece

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

   public void stop() {
    mainRoller.stopMotor();
    feederRoller.stopMotor();
   }
   
   @Override
   public void periodic() {}

