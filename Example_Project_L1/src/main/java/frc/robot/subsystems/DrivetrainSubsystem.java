// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.GyroSim;
import frc.robot.MotorSim;
import frc.robot.util.CoralVisualizer;

public class DrivetrainSubsystem extends SubsystemBase {
  /** Creates a new DrivetrainSubsystem. */
  public final GyroSim m_gyro = new GyroSim();
  public final MotorSim m_leftMotor = new MotorSim();
  public final MotorSim m_rightMotor = new MotorSim();
  public DifferentialDriveWheelSpeeds m_speeds = new DifferentialDriveWheelSpeeds(0, 0);
  public Pose2d m_pose;
  public double m_rotationRadians;
  public static final double WHEEL_CIRCUMFRANCE = Units.inchesToMeters(4);
  public static final double DRIVETRAIN_TRACKWIDTH_METERS = Units.inchesToMeters(20.811607);

  private final DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(
      DRIVETRAIN_TRACKWIDTH_METERS);

  public DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(new Rotation2d(m_rotationRadians), 0, 0,
      new Pose2d());

  public DrivetrainSubsystem() {
  }

  @AutoLogOutput // Periodically logs the Pose of the simulated robot
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return m_kinematics.toChassisSpeeds(m_speeds);
  }

  public void drive(double xSpeed, double zRotation) {
    xSpeed = Math.copySign(xSpeed * xSpeed, xSpeed);
    zRotation = Math.copySign(zRotation * zRotation, zRotation);

    double leftSpeed = xSpeed - zRotation;
    double rightSpeed = xSpeed + zRotation;

    // Find the maximum possible value of (throttle + turn) along the vector
    // that the joystick is pointing, then desaturate the wheel speeds
    double greaterInput = Math.max(Math.abs(xSpeed), Math.abs(zRotation));
    if (greaterInput == 0.0) {
      m_speeds = new DifferentialDriveWheelSpeeds();
    } else {
      m_speeds = new DifferentialDriveWheelSpeeds(leftSpeed, rightSpeed);
    }
  }

  @AutoLogOutput // Periodically logs the rotation of the simulated robot
  public Rotation2d getGyroscopeRotation() {
    return new Rotation2d(m_gyro.getAngle());
  }

  @Override
  public void periodic() {
    double leftPosition = m_leftMotor.getValue(m_speeds.leftMetersPerSecond);
    double rightPosition = m_rightMotor.getValue(m_speeds.rightMetersPerSecond);
    ChassisSpeeds m_ChassisSpeeds = m_kinematics.toChassisSpeeds(m_speeds);
    m_rotationRadians = m_gyro.getGyroValueAdded(m_ChassisSpeeds.omegaRadiansPerSecond);
    // update gyro and distance
    m_pose = m_odometry.update(new Rotation2d(m_rotationRadians), leftPosition, rightPosition);

    CoralVisualizer.showHeldNotes();
  }

  public void setDisabled() {
    drive(0, 0);
  }

  public void setPose(Pose2d pose) {
    m_odometry.resetPose(pose);
  }
}