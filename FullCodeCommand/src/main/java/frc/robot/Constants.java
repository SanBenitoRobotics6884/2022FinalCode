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
        public static final double kSlowSpd = 0.4;
        public static final double kFastSpd = 0.8;
        public static final double kdrivedeadband = 0.1;
        public static final double kMaxTurn = 360;

    }

    public final class Cargo {
        public static final double kLwrStorageDelay = 3;
        public static final double kIntkVltge = 6;
        public static final double kLaunchSpd = 0.75;
        public static final double kLwrMtrSpd = 0.6;

        public static final int kIntakeID = 6;
        public static final int kLowerStorageID = 5;
        public static final int kUpperStorageID = 7;
    }

    public final class Lift {
        public final class LiftPID {
            public static final double kPlift = 0.00015;
            public static final double kIlift = 0;
            public static final double kDlift = 0;
            public static final double kFUnloadedLift = 0;
            public static final double kFLoadedLift = 0.0004;
            public static final double kMinOutputLift = -1;
            public static final double kMaxOutputLift = 1;
            public static final double kMaxVelLift = 2000; // rpm
            public static final double kMaxAccLift = 1500;
            public static final double kAllowedErrLift = 0;
        }

        public static final boolean isPid = false; //Run in PID mode instead of direct control (open loop)
        public static final boolean isDynamicKF = false; //EXPERIMENTAL. USE WITH CAUTION
        public static final double kLowSetpoint = 0; //Units: motor rotations
        public static final double kHighSetpoint = 20; //Units: motor rotations
        public static final double kMaxVoltageLeft = 4;
        public static final double kMaxVoltageRight = 4.5;
        public static final double kRatchetDeploy = 0;
        public static final double kRatchetRetract = -0.5;
        public static final double kRatchetDelay = 3;
        public static final double kMaxHeight = -57; //encoder counts

        public static final int kSmartMotionSlot = 0;
        public static final int kLeftLiftID = 8;
        public static final int kRightLiftID = 9;
        public static final int kLeftServoPort = 0;
        public static final int kRightServoPort = 1;
        public static final int kLeftLimitPort = 8;
        public static final int kRightLimitPort = 9;
    }
}