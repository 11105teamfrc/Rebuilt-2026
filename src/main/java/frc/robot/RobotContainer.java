
package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.Auto2;
import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.subsystems.CANFuelSubsystem;
import frc.robot.subsystems.ClimberSubsystem;

public class RobotContainer {

  // The robot's subsystems
  private final CANDriveSubsystem driveSubsystem = new CANDriveSubsystem();
  private final CANFuelSubsystem ballSubsystem = new CANFuelSubsystem();
  private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();

  // The driver's controller
  private final CommandXboxController driverController = new CommandXboxController(
      Constants.OperatorConstants.DRIVER_CONTROLLER_PORT);

  // The operator's controller
  private final CommandXboxController operatorController = new CommandXboxController(
      Constants.OperatorConstants.OPERATOR_CONTROLLER_PORT);

  // The autonomous chooser
  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  public RobotContainer() {

    configureBindings();

    ballSubsystem.periodic();

    autoChooser.setDefaultOption(
        "Auto - Andar",
        new Auto2(driveSubsystem, ballSubsystem));
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  private void configureBindings() {

    // Driver Controller
    driveSubsystem.setDefaultCommand(
        driveSubsystem.driveArcade(
            () -> driverController.getLeftY() * Constants.OperatorConstants.DRIVE_SCALING,
            () -> -driverController.getRightX() * Constants.OperatorConstants.ROTATION_SCALING));

    // Zerar encoder em LB
    // driverController.leftBumper()
    // .onTrue(ballSubsystem.run(ballSubsystem::resetEncoder));

    // Operator Controller

    // While the left bumper on operator controller is held, intake Fuel
    /*
     * operatorController.leftBumper()
     * .whileTrue(ballSubsystem.runEnd(() -> ballSubsystem.intake(),
     * () -> ballSubsystem.stop()));
     * /*
     */
    // While the right bumper on the operator controller is held, spin up for 1
    // second, then launch fuel. When the button is released, stop.
    /*
     * operatorController.rightBumper()
     * 
     * .whileTrue(ballSubsystem.shoot().withTimeout(1.5)
     * .andThen(ballSubsystem.launchCommand())
     * .finallyDo(() -> ballSubsystem.stop()));
     * /*
     */

    // LeftBumper gira Intake
    operatorController.leftBumper()
        .whileTrue(ballSubsystem.run(ballSubsystem::intake))
        .whileFalse(ballSubsystem.run(ballSubsystem::stop));

    // RightBumper gira apenas o Shooter
    operatorController.rightBumper()
        .whileTrue(ballSubsystem.shoot(() -> 1))
        .whileFalse(ballSubsystem.run(ballSubsystem::stop));

    // X gira apenas o feeder
    operatorController.x()
        .whileTrue(ballSubsystem.run(ballSubsystem::buffer))
        .whileFalse(ballSubsystem.run(ballSubsystem::feederSlowRollback)
        .withTimeout(2)
        .andThen(ballSubsystem::stop));

    // A RUN OUTTAKE (EJETAR POR BAIXO)
    operatorController.a()
        .whileTrue(ballSubsystem.run(ballSubsystem::outtake))
        .whileFalse(ballSubsystem.run(ballSubsystem::stop));

    operatorController.rightTrigger()
        .whileTrue(ballSubsystem.run(ballSubsystem::feederRollback))
        .whileFalse(ballSubsystem.run(ballSubsystem::stop));

    /* Climb UPq
    climberSubsystem.setDefaultCommand(
        climberSubsystem.climbUp(
            () -> operatorController.getLeftY())); */

    // TEST for PID bindings controller //

    // TEST SYSID

    /*
     * driverController.a()
     * .whileTrue(ballSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
     * 
     * driverController.b()
     * .whileTrue(ballSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
     * 
     * driverController.x()
     * .whileTrue(ballSubsystem.sysIdDynamic(SysIdRoutine.Direction.kForward));
     * 
     * driverController.y()
     * .whileTrue(ballSubsystem.sysIdDynamic(SysIdRoutine.Direction.kReverse));
     * /*
     */

  }

  public Command getAutonomousCommand() {

    return autoChooser.getSelected();

  }

}
