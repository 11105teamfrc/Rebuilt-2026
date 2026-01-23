package frc.robot;

import frc.robot.Constants.OperatorButtons;
import frc.robot.commands.DrivetrainCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.subsystems.DrivetrainSubystem;
import frc.robot.subsystems.ShooterSubystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;



public class RobotContainer {

  // Declarar Subsystem 

  private final DrivetrainSubystem drivetrain = new DrivetrainSubystem();
  private final ShooterSubystem shooter = new ShooterSubystem();

  
  // Drive Controller
 
  private final Joystick driverController = 
    new Joystick(Constants.OperatorConstants.kDriverControllerPort);

  private final Joystick operatorController =
    new Joystick(Constants.OperatorConstants.kOperatorControllerPort);

  public RobotContainer() {
    configureDefaultCommands();
    configureBindings();
  }

  // Private Button Configs

  private void configureDefaultCommands() {

    drivetrain.setDefaultCommand(
      new DrivetrainCommand(drivetrain, driverController)
    );
  }


  private void configureBindings() {
    
    new JoystickButton(operatorController, OperatorButtons.SHOOT)
      .whileTrue(new ShooterCommand(shooter));

  }
 
  public Command getAutonomousCommand() {
    return null;
  }
}
