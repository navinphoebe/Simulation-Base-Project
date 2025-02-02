// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DriveDistance extends Command {
  private final DrivetrainSubsystem m_drive;
  private final double m_distance;
  private final double m_speed;
  private double startX;
  private double startY;

  /**
   * Creates a new DriveDistance. This command will drive your your robot for a desired distance at
   * a desired speed.
   *
   * @param speed The speed at which the robot will drive
   * @param inches The number of meters the robot will drive
   * @param drive The drivetrain subsystem on which this command will run
   */
  public DriveDistance(double speed, double meters, DrivetrainSubsystem drive) {
    m_distance = meters;
    m_speed = speed;
    m_drive = drive;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startX = m_drive.getPose().getX();
    startY =  m_drive.getPose().getY();
    m_drive.drive(0, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drive.drive(m_speed, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.drive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Compare distance travelled from start to desired distance
    double endX = m_drive.getPose().getX();
    double endY =  m_drive.getPose().getY();
    double distance = Math.sqrt(Math.pow(endX - startX, 2) + Math.pow(endY - startY, 2));
    return Math.abs(distance) >= m_distance;
  }
}
