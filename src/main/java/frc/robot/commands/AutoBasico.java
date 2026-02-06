package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.CANDriveSubsystem;

public class AutoBasico extends SequentialCommandGroup {
    
    public AutoBasico(CANDriveSubsystem drive){
        addCommands(
            new InstantCommand(() ->
            drive.driveArcade(() -> 0.4, () -> 0).schedule()
            ),

            // Tempo andando
            new WaitCommand(2.0),

            // Para
            new InstantCommand(() ->
            drive.driveArcade(() -> 0, () -> 0).schedule()
            )
        );
    }
}
