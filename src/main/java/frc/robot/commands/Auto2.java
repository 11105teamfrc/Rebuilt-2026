package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.subsystems.CANFuelSubsystem;

public class Auto2 extends SequentialCommandGroup {

    public Auto2(CANDriveSubsystem drive, CANFuelSubsystem fuel) {

        Command runDriveTrainCommand = Commands.startEnd(
            () -> drive.driveArcade(() -> 0.3, () -> 0.0),
            () -> drive.driveArcade(() -> 0.0, () -> 0.0),
            drive
        );

         Command launchCommand = Commands.startEnd(
            fuel::launch,   
            fuel::stop,    
            fuel
        );


        addCommands(
            runDriveTrainCommand.withTimeout(2.0),
            new WaitCommand(0.3),
            launchCommand.withTimeout(1.0)
        );
    }
}