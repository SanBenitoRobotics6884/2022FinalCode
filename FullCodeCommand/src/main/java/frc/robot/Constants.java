// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class Drive {
        public final class TurnRatePID {
            public static final double kP = 0.002;
            public static final double kI = 0;
            public static final double kD = 0.0;
        }

        public final class AbsoluteAnglePID {
            public static final double kP = 0.02;
            public static final double kI = 0;
            public static final double kD = 0.001;
            public static final double kF = 0.01;
            public static final double kMaxVelRot = 420;
            public static final double kMaxAccRot = 1440;
            public static final double kAllowedError = 4;
        }

        public final class PositionPID {
            public static final double kP = 0.8;
            public static final double kI = 0;
            public static final double kD = 0.0;
            public static final double kF = 0.025;
            public static final double kMaxVel = 1;
            public static final double kMaxAcc = 1;
            public static final double kAllowedError = 0.02;
        }

        public static final double kFTurn = 0;

        public static final Translation2d m_frontLeftLocation = new Translation2d(0.254, 0.254);
        public static final Translation2d m_frontRightLocation = new Translation2d(0.254, -0.254);
        public static final Translation2d m_backLeftLocation = new Translation2d(-0.254, 0.254);
        public static final Translation2d m_backRightLocation = new Translation2d(-0.254, -0.254);

        public static final int kLeftFrontMotor = 1;
        public static final int kRightFrontMotor = 3;
        public static final int kLeftBackMotor = 2;
        public static final int kRightBackMotor = 4;

        public static final double kConversionFactor = 0.028;
        public static final double kRateLimit = 2.0;
        public static final double kSlowSpd = 0.3;
        public static final double kFastSpd = 0.6;
        public static final double kTurboSpd = 1.0;
        public static final double kDefaultTurn = 0.5;
        public static final double kPreciseTurn = 0.3;
        public static final double kdrivedeadband = 0.1;
        public static final double kMaxTurn = 540;
        public static final double kAdditionalFastStrafeMultiplier = 2;

    }

    public static final class Cargo {
        public static final double kIntkVltge = 9;
        public static final double kLaunchSpd = 0.9;
        public static final double kLwrMtrSpd = 0.5;

        public static final int kIntakeID = 6;
        public static final int kLowerStorageID = 5;
        public static final int kUpperStorageID = 7;
    }

    public static final class Lift {
        public static final double kMaxVoltageLeft = 9.5;
        public static final double kMaxVoltageRight = 12;
        public static final double kMaxHeight = 340; //encoder counts

        public static final int kSmartMotionSlot = 0;
        public static final int kLeftLiftID = 8;
        public static final int kRightLiftID = 9;
        public static final int kLeftLimitPort = 8;
        public static final int kRightLimitPort = 9;
    }

    public static final class Feedback {
        public static final int kDrivePowerChannels[] = {4,7,16,19};
        public static final double kRumblePulseWidth = 0.3; // Duration of rumble pulse
        public static final double kRumblePulseRate = 3;
        public static final double kRumbleStrength = 1;
        public static final double kAccelerationRumbleThreshold = 30;
        public static final double kCurrentRatioRumbleThreshold = 10;
    }

    public static final class Auto {
        public static final double kSimpleDistY = 0; // Positive = Left
        public static final double kSimpleDistX = 2.0; // Positive = Forw
        public static final double kSimpleAngle = 0;
        
        public static final double kDumpDistY = 0; // Positive = Left
        public static final double kDumpDistX = 2.2; // Positive = Forw
        public static final double kDumpAngle = 0;

        public static final double kComplexDistY = 0; // Positive = Left
        public static final double kComplexDistX = 2.0; // Positive = Forw
        public static final double kComplexAngle = 0;
    }

}
