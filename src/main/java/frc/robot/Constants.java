package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.util.Units;
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
    
    public static class ElevatorConstants {
        // MOTOR CAN BUS IDS
        public static final int kLeaderID = 46;
        public static final int kFollowerID = 45;
        // RAW PID CONSTANTS TODO: TUNE
        public static final double kP = .1;
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
        public static final double kMinimumRotationLimit = -5; // TODO: SET
        public static final double kMaximumRotationLimit = 100; // TODO: SET
        public static final double kMinimumOutputLimit = -.8;
        public static final double kMaximumOutputLimit = .8;
        // INVERSIONS
        public static final boolean kInverted = false;
        public static final boolean kFollowerInverted = true;
        // CURRENT LIMITS TODO: TUNE
        public static final int kStallLimit = 80;
        public static final int kFreeLimit = 80;
        // IDLE MODE
        public static final IdleMode kIdleMode = IdleMode.kBrake;
    }
    
    public static class PivotConstants {
        // MOTOR CAN BUS IDS
        public static final int kLeaderID = 44;
        public static final int kFollowerID = 43;
        // RAW PID CONSTANTS TODO: TUNE
        public static final double kP = .012;
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
        public static final double kMinimumRotationLimit = -105; // TODO: SET
        public static final double kMaximumRotationLimit = 5; // TODO: SET
        public static final double kMinimumOutputLimit = -.8;
        public static final double kMaximumOutputLimit = .8;
        // INVERSIONS
        public static final boolean kInverted = false;
        public static final boolean kFollowerInverted = true;
        public static final boolean kAbsoluteEncoderInverted = true;
        // OFFSET
        public static final double kOffset = 0.7140662; // TODO: SET
        // CONVERSION FACTOR
        public static final double kPositionConversionFactor = 360.0;
        // CURRENT LIMITS TODO: TUNE
        public static final int kStallLimit = 80;
        public static final int kFreeLimit = 80;
        // IDLE MODE
        public static final IdleMode kIdleMode = IdleMode.kBrake;
    }

    public static class SimConstants {
        public static final int kNumElevatorMotors = 2;
        public static final int kElevatorMotorPort = 1;
        public static final int kElevatorEncoderAChannel = 3;
        public static final int kElevatorEncoderBChannel = 4;

        public static final double kElevatorDrumRadius = Units.inchesToMeters(1.75 / 2.0);
        public static final double kElevatorCarriageMass = Units.lbsToKilograms(15);
        public static final double kElevatorGearing = 20.0;
        public static final double kElevatorEncoderDistPerPulse = 2.0 * Math.PI * kElevatorDrumRadius / 4096.0;

        public static final double kMinimumElevatorLength = Units.inchesToMeters(15);
        public static final double kMaximumElevatorLength = Units.inchesToMeters(60);

        public static final double kElevatorP = 25;
        public static final double kElevatorI = 0;
        public static final double kElevatorD = 0;

        public static final int kNumPivotMotors = 2;
        public static final int kPivotMotorPort = 2;
        public static final int kPivotEncoderAChannel = 5;
        public static final int kPivotEncoderBChannel = 6;

        public static final double kPivotArmMass = kElevatorCarriageMass + Units.lbsToKilograms(10);
        public static final double kPivotGearing = 20.0 * 52.0 / 12.0;
        public static final double kPivotEncoderDistPerPulse = 2.0 * Math.PI / 4096;

        public static final double kMinimumPivotAngle = -30;
        public static final double kMaximumPivotAngle = 95.0;

        public static final double kPivotP = 4;
        public static final double kPivotI = 0;
        public static final double kPivotD = 0;

        public static final double kLoopTime = 0.020;
    }
}
