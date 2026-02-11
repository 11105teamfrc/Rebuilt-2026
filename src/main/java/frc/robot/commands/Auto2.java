package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.subsystems.CANFuelSubsystem;

public class Auto2 extends SequentialCommandGroup {

    private static final double kAutoDriveSpeed = 0.6;
    private static final double kAutoDriveDistanceMeters = 1.5;

    public Auto2(CANDriveSubsystem drive, CANFuelSubsystem fuel) {

        Command runDriveTrainCommand = new FunctionalCommand(
        drive::resetEncoders,

        () -> drive.arcadeDrive(kAutoDriveSpeed, 0),
        
        (interrupt) -> drive.arcadeDrive (0, 0),

        () -> drive.getAverageDistanceMeters() >= kAutoDriveDistanceMeters,

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