// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CANFuelSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class LaunchCommand extends Command {
 
  private final CANFuelSubsystem fuelSubsystem;
  private final Timer timer = new Timer(); 
  
  private static final double SHOOTER_SPINUP_TIME = 0.4;

  public LaunchCommand(CANFuelSubsystem subsystem) {
    this.fuelSubsystem = subsystem;
    addRequirements(fuelSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();

    fuelSubsystem.startLaunchOnly();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (timer.hasElapsed(SHOOTER_SPINUP_TIME)) {
      fuelSubsystem.startFeederOnly();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
    fuelSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
