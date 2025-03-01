package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.generated.TunerConstants;

public class Constants {
    public static class SwerveConstants {
        public static final double kMaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
        public static final double kMaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

        public static final double kTranslationalDeadband = kMaxSpeed * 0.1;
        public static final double kRotationalDeadband = kMaxAngularRate * 0.1;
    }

    public static class AutoConstants {
        // TRANSLATION PID
        public static final double kTranslationP = 10;
        public static final double kTranslationI = 0;
        public static final double kTranslationD = 0;
        // ROTATION PID
        public static final double kRotationP = 7;
        public static final double kRotationI = 0;
        public static final double kRotationD = 0;
    }

    public static class IntakeConstants {

        public static final int kMotorID = 40;

        public static final double kMAX_LIMIT = 1;
        public static final double kMIN_LIMIT = -1;
    }

    
    public static class TiltWristConstants {
        // MOTOR CAN BUS IDS TODO: CONFIRM
        public static final int kMotorID = 41; // CONFIRM
        // RAW PID CONSTANTS TODO: TUNE
        public static final double kP = 0.01;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kFF = 0;
        // MAXMOTION CONSTANTS TODO: TUNE
        public static final double kMAXMotionP = 0;
        public static final double kMAXMotionI = 0;
        public static final double kMAXMotionD = 0;
        public static final double kMAXMotionFF = 0;
        public static final double kMAXMotionMaxAcceleration = 0;
        public static final double kMAXMotionMaxVelocity = 0;
        public static final double kMAXMotionAllowedError = 0;
        // TOLERANCE FOR PID ERROR
        public static final double kTolerance = 1.0; // TODO: TUNE
        // LIMIT VALUES
        public static final double kMinimumRotationLimit = -150; // TODO: SET
        public static final double kMaximumRotationLimit = 150; // TODO: SET
        public static final double kMinimumOutputLimit = -.8;
        public static final double kMaximumOutputLimit = .8;
        // INVERSIONS TODO: CONFIRM
        public static final boolean kInverted = true;
        public static final boolean kAbsoluteEncoderInverted = false;
        // CONVERSION FACTOR AND OFFSETS
        public static final double kPositionConversionFactor = 360.0;
        public static final double kOffset = 0.2311125; // TODO: SET
        public static final boolean kZeroCentered = true;
        // CURRENT LIMITS TODO: TUNE        
        public static final int kStallLimit = 10;
        public static final int kFreeLimit = 10;
        // IDLE MODE
        public static final IdleMode kIdleMode = IdleMode.kBrake;
    }
    
    public static class RotateWristConstants {
        // MOTOR CAN BUS IDS TODO: CONFIRM
        public static final int kMotorID = 42;
        // RAW PID CONSTANTS TODO: TUNE
        public static final double kP = 0.04;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kFF = 0;
        // MAXMOTION CONSTANTS TODO: TUNE
        public static final double kMAXMotionP = 0;
        public static final double kMAXMotionI = 0;
        public static final double kMAXMotionD = 0;
        public static final double kMAXMotionFF = 0;
        public static final double kMAXMotionMaxAcceleration = 0;
        public static final double kMAXMotionMaxVelocity = 0;
        public static final double kMAXMotionAllowedError = 1.0;
        // TOLERANCE FOR PID ERROR
        public static final double kTolerance = 1.0; // TODO: TUNE
        // LIMIT VALUES
        public static final double kMinimumRotationLimit = -90; // TODO: SET
        public static final double kMaximumRotationLimit = 90; // TODO: SET
        public static final double kMinimumOutputLimit = -.8;
        public static final double kMaximumOutputLimit = .8;
        // INVERSIONS
        public static final boolean kInverted = false;
        public static final boolean kAbsoluteEncoderInverted = false;
        // CONVERSION FACTOR AND OFFSETS
        public static final double kPositionConversionFactor = 360.0;
        public static final double kOffset = 0.7550457; // TODO: SET
        public static final boolean kZeroCentered = true;
        // CURRENT LIMITS TODO: TUNE
        public static final int kStallLimit = 3;
        public static final int kFreeLimit = 3;
        // IDLE MODE
        public static final IdleMode kIdleMode = IdleMode.kBrake;
    }
}