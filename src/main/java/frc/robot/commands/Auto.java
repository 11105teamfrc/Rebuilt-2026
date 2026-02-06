// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.subsystems.CANFuelSubsystem;

public class Auto extends SequentialCommandGroup {
    
    public Auto(CANDriveSubsystem drive, CANFuelSubsystem fuel) {

        addCommands(
        
            new RunCommand(
                () -> drive.driveArcade(
                    () -> 4.0,
                    () -> 0.0
                ),
                drive
            ).withTimeout(2.0), 
            
            new InstantCommand(
                () -> drive.driveArcade(
                    () -> 0.0,
                    () -> 0.0
                ),
                drive
            ),

            new WaitCommand(0.3),

            new InstantCommand(
                fuel::launch,
                fuel
            )
        );
    }
}