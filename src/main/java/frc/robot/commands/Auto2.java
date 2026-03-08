package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.subsystems.CANFuelSubsystem;

public class Auto2 extends SequentialCommandGroup {

    private static final double kAutoDriveSpeed = 1.0;
    private static final double kAutoDriveDistanceMeters = 1.5;

    public Auto2(CANDriveSubsystem drive, CANFuelSubsystem fuel) {

        /*Command runDriveTrainCommand = new FunctionalCommand(
                drive::resetEncoders,

                () -> drive.driveArcade(() -> kAutoDriveSpeed, () -> 1.0),
                (interrupt) -> drive.driveArcade(() -> 0, () -> 0),
                () -> drive.getAverageDistanceMeters() >= kAutoDriveDistanceMeters,
                drive
        );*/

        Command launchCommand = Commands.startEnd(
                fuel::launch,
                fuel::stop,
                fuel);

 // main gira as duas de baixo e shotter
        // feeder gira a de dentro    
        Command initShooterCommand = Commands.startEnd(
            fuel::shoot,
            fuel::stop,
            fuel);

        /* 
        Command driveArcade = Commands.startEnd(
            drive.arcadeDrive(0.8,0),
            drive.arcadeDrive(0, 0),
            drive
        );

            */
        
        addCommands(

               // drive.driveArcade(() -> 0.8, () -> 0).withTimeout(0.3),
                //drive.arcadeDrive(0, 0).withTimeout(),
                new WaitCommand(0.3),
                initShooterCommand.withTimeout(1.0),
                launchCommand
        );
    }
}