// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static frc.robot.Constants.OperatorConstants;
import static frc.robot.Constants.FuelConstants;
import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.subsystems.CANFuelSubsystem;

public class RobotContainer {
  
  // The robot's subsystems

  private final CANDriveSubsystem driveSubsystem = new CANDriveSubsystem();
  private final CANFuelSubsystem ballSubsystem = new CANFuelSubsystem();

  // The driver's controller
  private final CommandXboxController driverController = new CommandXboxController(Constants.OperatorConstants.DRIVER_CONTROLLER_PORT);

  // The operator's controller
  private final CommandXboxController operatorController = new CommandXboxController(Constants.OperatorConstants.OPERATOR_CONTROLLER_PORT);

  // The autonomous chooser
  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  public RobotContainer() {
    configureBindings();

    // Set the options to show up in the Dashboard for selecting auto modes. If you
    // add additional auto modes you can add additional lines here with
    // autoChooser.addOption
   // autoChooser.setDefaultOption("Autonomous", Autos.exampleAuto(driveSubsystem, ballSubsystem));
  }


  private void configureBindings() {

    // Operator Controller 
    
    // LEFTBUMPER RUN INTAKE

    operatorController.leftTrigger()
      .whileTrue(
        ballSubsystem.runEnd(
          ballSubsystem::intake,
          ballSubsystem::stop));

    // RIGHTBUMPER RUN LAUNCH (SHOOTER)
    operatorController.rightTrigger()
      .whileTrue(
        ballSubsystem.runEnd(
          ballSubsystem::launch,
          ballSubsystem::stop));

    // A RUN OUTTAKE (EJETAR POR BAIXO)      
    operatorController.a()
      .whileTrue(
        ballSubsystem.runEnd(
          ballSubsystem::outtake,
          ballSubsystem::stop));
    
      // Driver Controller
  
    driveSubsystem.setDefaultCommand(
        driveSubsystem.driveArcade(
            () -> driverController.getLeftY() * Constants.OperatorConstants.DRIVE_SCALING,
            () -> -driverController.getRightX() * Constants.OperatorConstants.ROTATION_SCALING));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    
    return autoChooser.getSelected();

  }
}
