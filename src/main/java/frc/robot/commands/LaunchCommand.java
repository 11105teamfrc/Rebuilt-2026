package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CANFuelSubsystem;

public class LaunchCommand extends Command {
 
  private final CANFuelSubsystem fuelSubsystem;

  public LaunchCommand(CANFuelSubsystem subsystem) {
    this.fuelSubsystem = subsystem;
    addRequirements(fuelSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    fuelSubsystem.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
