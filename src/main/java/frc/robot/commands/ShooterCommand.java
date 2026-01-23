package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubystem;

public class ShooterCommand extends Command {

  private final ShooterSubystem shooter;
  
  public ShooterCommand(ShooterSubystem shooter) {
      this.shooter = shooter;
      addRequirements(shooter);
  }

  
  @Override
  public void initialize() {

    shooter.shoot();

  }

 
  @Override
  public void execute() {}


  @Override
  public void end(boolean interrupted) {

    shooter.stop();

  }

  
  @Override
  public boolean isFinished() {
    return false;
  }
}
