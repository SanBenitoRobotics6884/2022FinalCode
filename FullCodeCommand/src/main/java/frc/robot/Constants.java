// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public final class Drive {
        public final class TurnRatePID {
            public static final double kP = 0.008;
            public static final double kI = 0;
            public static final double kD = 0.00005;
            public static final double kF = 0.01;
        }

        public static final int kLeftFrontMotor = 1;
        public static final int kRightFrontMotor = 3;
        public static final int kLeftBackMotor = 2;
        public static final int kRightBackMotor = 4;

        public static final double kDriveGearing = 10.71;
        public static final double kWheelCircumference = 6 * Math.PI;
        public static final double kSlowspd = 0.4;
        public static final double kFastSpd = 0.8;
        public static final double kdrivedeadband = 0.1;
        public static final double kMaxTurn = 360;

    }

    public final class Cargo {
        public static final double kLwrStorageDelay = 3;
        public static final double intkVltge = 6;
        public static final double uprMtrSpd = 0.75;
        public static final double lwrMtrSpd = 0.6;

        public static final int kIntakeID = 6;
        public static final int kLowerStorageID = 5;
        public static final int kUpperStorageID = 7;
    }
}
