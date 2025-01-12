// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.util.CoralVisualizer;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutonomousDistance extends SequentialCommandGroup {
  /**
   * Creates a new Autonomous Drive based on distance. This will drive out for a specified distance,
   * turn around and drive back.
   *
   * @param drivetrain The drivetrain subsystem on which this command will run
   */
  public AutonomousDistance(DrivetrainSubsystem drivetrain) {
    addCommands(
        new InstantCommand(() -> CoralVisualizer.pickupCoral()),
        new TeleportToStart(drivetrain),
        new DriveDistance(1, 1.37, drivetrain),
        new TurnDegrees(.5, -60, drivetrain),  
        new DriveDistance(1, 1.2, drivetrain),
        CoralVisualizer.shoot()); 
  }
}
