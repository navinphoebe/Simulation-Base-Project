// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;

public class TurnDegrees extends Command {
  private final DrivetrainSubsystem m_drive;
  private double m_degrees;
  private final double m_speed;
  private double startDegrees;
  private double sign = 1;

  /**
   * Creates a new TurnDegrees. This command will turn your robot for a desired rotation (in
   * degrees) and rotational speed.
   *
   * @param speed The speed which the robot will drive. Negative is in reverse.
   * @param degrees Degrees to turn. Leverages encoders to compare distance.
   * @param drive The drive subsystem on which this command will run
   */
  public TurnDegrees(double speed, double degrees, DrivetrainSubsystem drive) {
    m_degrees = degrees * sign;
    m_speed = speed * sign;
    m_drive = drive;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Set motors to stop, read encoder values for starting point
    m_degrees %= 360;
    m_degrees += 360;
    m_degrees %= 360;
    startDegrees = m_drive.getPose().getRotation().getDegrees();
    startDegrees %= 360;
    startDegrees += 360;
    startDegrees %= 360;
    m_drive.drive(0, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drive.drive(0, m_speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.drive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double difference = startDegrees - m_drive.getPose().getRotation().getDegrees();
    difference %= 360;
    difference += 360 * sign;
    difference %= 360;
    double distance = m_degrees - difference;
    return Math.abs(distance) < Constants.AUTO_TURNING_DEADBAND;
  }
}
