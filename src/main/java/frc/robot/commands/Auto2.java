// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.subsystems.CANFuelSubsystem;

public class Auto2 extends SequentialCommandGroup {
    public Auto2(CANDriveSubsystem drive, CANFuelSubsystem fuel) {
        Command runDriveTrainCommand = Commands.startEnd(() -> drive.driveArcade( 
                () -> 0.3, 
                () -> 0.0
            ),
        
        () -> drive.driveArcade(
                () -> 0.0, 
                () -> 0.0
            ), drive);
        
        addCommands( 
        runDriveTrainCommand,

        new WaitCommand(0.3),

        

        );
    }
}