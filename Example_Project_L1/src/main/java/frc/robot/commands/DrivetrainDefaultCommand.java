// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DrivetrainDefaultCommand extends Command {
  private DrivetrainSubsystem m_drivetrain;
  private CommandXboxController m_driverController;

  /** Creates a new DrivetrainDefaultCommand. */
  public DrivetrainDefaultCommand(DrivetrainSubsystem drivetrain, CommandXboxController driverController) {
    m_driverController = driverController;
    m_drivetrain = drivetrain;
    addRequirements(m_drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double x = m_driverController.getRawAxis(1) * -1;
    double r = m_driverController.getRawAxis(RobotContainer.m_controller_chooser.getSelected()) * -1;
    x = applyDeadband(x);
    r = applyDeadband(r);
    
    m_drivetrain.drive(x, r);
  }

  private double applyDeadband(double x) {
    if (Math.abs(x) > Constants.CONTROLLER_DEADBAND_VALUE) {
      return x;
    } else {
      return 0;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}