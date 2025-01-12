// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;

/**
 * Contains various field dimensions and useful reference points. Dimensions are
 * in meters, and sets
 * of corners start in the lower left moving clockwise. <b>All units in
 * Meters</b> <br>
 * <br>
 *
 * <p>
 * All translations and poses are stored with the origin at the rightmost point
 * on the BLUE
 * ALLIANCE wall.<br>
 * <br>
 * Length refers to the <i>x</i> direction (as described by wpilib) <br>
 * Width refers to the <i>y</i> direction (as described by wpilib)
 */
public class FieldConstants {
        public static final double fieldLength = 17.55;
        public static final double fieldWidth = 8.05;

        public static final Translation2d ampCenter = new Translation2d(Units.inchesToMeters(72.455), fieldWidth);

        /** Each corner of the speaker * */
        public static final class CoralReef {

                // corners (blue alliance origin)
                public static final Translation3d position1 = new Translation3d(
                                4.124,
                                4.630,
                                .5);

                public static final Translation3d position2 = new Translation3d(
                                4.859,
                                4.590,
                                .5);

                public static final Translation3d position3 = new Translation3d(
                                5.152,
                                4.046,
                                .5);

                public static final Translation3d position4 = new Translation3d(
                                4.859,
                                3.439,
                                .5);

                public static final Translation3d position5 = new Translation3d(
                                4.168,
                                3.418,
                                .5);

                public static final Translation3d position6 = new Translation3d(
                                3.791,
                                3.983,
                                .5);

        }

        public static final double aprilTagWidth = Units.inchesToMeters(6.50);

}