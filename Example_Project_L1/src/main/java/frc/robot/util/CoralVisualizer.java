// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import frc.robot.FieldConstants;

import java.util.ArrayList;
import java.util.List;
import java.util.Set;
import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class CoralVisualizer {
  private static final double shotSpeed = 1; // Meters per sec
  private static Supplier<Pose2d> robotPoseSupplier;
  public static final List<Translation2d> autoNotes = new ArrayList<>();
  public static boolean hasNote = false;
  public static Translation3d[] coralReefPoses = { FieldConstants.CoralReef.position1,
      FieldConstants.CoralReef.position1, FieldConstants.CoralReef.position2, FieldConstants.CoralReef.position3,
      FieldConstants.CoralReef.position4, FieldConstants.CoralReef.position5, FieldConstants.CoralReef.position6};

  public static void setPoseSuppliers(Supplier<Pose2d> pose1) {
    robotPoseSupplier = pose1;
  }

  /** Shows the currently held note if there is one */
  public static void showHeldNotes() {
    if (hasNote) {
      Logger.recordOutput("CoralVisualizer/HeldCoral", new Pose3d[] { getIndexerPose3d() });
    } else {
      Logger.recordOutput("CoralVisualizer/HeldCoral", new Pose3d[] {});
    }
  }

  public static Translation3d pickCoralPosition() {
    Pose2d pose = AllianceFlipUtil.apply(robotPoseSupplier.get());
    double minimum = 100;
    int select = 0;
    for (int i = 0; i < coralReefPoses.length; i++) {
      Translation2d translation = coralReefPoses[i].toTranslation2d();
      Pose2d pose2 = new Pose2d(translation.getX(), translation.getY(), new Rotation2d());
      Transform2d pose3 = pose.minus(pose2);
      double distance = Math.sqrt(Math.pow(pose3.getX(), 2) + Math.pow(pose3.getY(), 2));
      if (distance < minimum) {
        minimum = distance;
        select = i;
      }
    }
    return coralReefPoses[select];
  }

  public static Command shoot() {
    return new ScheduleCommand( // Branch off and exit immediately
        Commands.defer(
            () -> {

              if (!hasCoral()) {
                return Commands.run(() -> {
                }).until(() -> true);
              }

              hasNote = false;
              final Pose3d startPose = getIndexerPose3d();
              final Pose3d endPose = new Pose3d(
                  AllianceFlipUtil.apply(pickCoralPosition()),
                  startPose.getRotation());

              final double duration = startPose.getTranslation().getDistance(endPose.getTranslation()) / shotSpeed;
              final Timer timer = new Timer();
              timer.start();
              return Commands.run(
                  () -> Logger.recordOutput(
                      "CoralVisualizer/ShotCoral",
                      new Pose3d[] {
                          startPose.interpolate(endPose, timer.get() / duration)
                      }))
                  .until(() -> timer.hasElapsed(duration))
                  .finallyDo(
                      () -> Logger.recordOutput("CoralVisualizer/ShotCoral", new Pose3d[] {}));
            },
            Set.of())
            .ignoringDisable(true));
  }

  private static Pose3d getIndexerPose3d() {
    Transform3d indexerTransform = new Transform3d(.25, 0, .65, new Rotation3d(0, 0, Math.PI / 2));
    return new Pose3d(robotPoseSupplier.get()).transformBy(indexerTransform);
  }

  @AutoLogOutput
  public static boolean hasCoral() {
    return hasNote;
  }

  public static void pickupCoral() {
    hasNote = true;
  }
}