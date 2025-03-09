package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.RegionOfInterest;
import au.grapplerobotics.interfaces.LaserCanInterface.TimingBudget;
import frc.robot.generated.TunerConstants;

public class Constants {
    public static class SwerveConstants {
        public static final double kSpeedCoefficient = .75;
        public static final double kSlowCoefficient = .5;
        public static final double kMaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
        public static final double kMaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

        public static final double kTranslationalDeadband = kMaxSpeed * 0.1;
        public static final double kRotationalDeadband = kMaxAngularRate * 0.1;
    }
  
    public static class LEDConstants {
        public static final int kPort = 0;
        public static final int kLEDLength = 36;
    }

    public static class ReefSensorConstants {
        public static final int kLeftID = 9;
        public static final int kLeftDistanceFromCenter = 0;
        public static final int kLeftDistanceFromSide = 0;
        public static final RegionOfInterest leftRegion = new RegionOfInterest(2, 8, 4, 4);

        public static final int kCenterID = 8;
        public static final int kCenterDistanceFromCenter = 95;
        public static final int kCenterDistanceFromSide = 0;
        public static final RegionOfInterest centerRegion = new RegionOfInterest(8, 8, 16, 16);

        public static final int kRightID = 7;
        public static final int kRightDistanceFromCenter = 118;
        public static final int kRightDistanceFromSide = 280; 
        public static final RegionOfInterest rightRegion = new RegionOfInterest(2, 8, 4, 4);
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

        public static final boolean kInverted = true;
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
        public static final double kMaximumRotationLimit = 50; // TODO: SET
        public static final double kMinimumOutputLimit = -.85;
        public static final double kMaximumOutputLimit = .85;
        // INVERSIONS
        public static final boolean kInverted = false;
        public static final boolean kFollowerInverted = true;
        // CURRENT LIMITS TODO: TUNE
        public static final int kStallLimit = 40;
        public static final int kFreeLimit = 40;
        // IDLE MODE
        public static final IdleMode kIdleMode = IdleMode.kBrake;
        // UNSAFE POSITION THRESHOLD
        public static final double kUnsafePosition = 7;
    }
    
    public static class PivotConstants {
        // MOTOR CAN BUS IDS
        public static final int kLeaderID = 44;
        public static final int kFollowerID = 43;
        // RAW PID CONSTANTS TODO: TUNE
        public static final double kP = .06;
        public static final double kI = 0;
        public static final double kD = 0.001;
        public static final double kFF = 0;
        // MAXMOTION CONSTANTS TODO: TUNE
        public static final double kMAXMotionP = 0.02;
        public static final double kMAXMotionI = 0;
        public static final double kMAXMotionD = 0;
        public static final double kMAXMotionFF = 0;
        public static final double kMAXMotionMaxAcceleration = 50;
        public static final double kMAXMotionMaxVelocity = 100;
        public static final double kMAXMotionAllowedError = 1;
        // TOLERANCE FOR PID ERROR
        public static final double kTolerance = 1.5; // TODO: TUNE
        // LIMIT VALUES
        public static final double kMinimumRotationLimit = -105; // TODO: SET
        public static final double kMaximumRotationLimit = 5; // TODO: SET
        public static final double kMinimumOutputLimit = -.6;
        public static final double kMaximumOutputLimit = .75;
        // INVERSIONS
        public static final boolean kInverted = false;
        public static final boolean kFollowerInverted = true;
        public static final boolean kAbsoluteEncoderInverted = true;
        // OFFSET
        public static final double kOffset = 0.8174936; // TODO: SET
        // CONVERSION FACTOR
        public static final double kPositionConversionFactor = 360.0;
        // CURRENT LIMITS TODO: TUNE
        public static final int kStallLimit = 70;
        public static final int kFreeLimit = 35;
        // IDLE MODE
        public static final IdleMode kIdleMode = IdleMode.kBrake;
        // DEFINING UNSAFE POSITION
        public static final double kUnsafePosition = -95; //TODO: REPLACE THE FUNNY NUMBER WITH AN ACTUAL POSITION
    }
  
    public static class TiltWristConstants {
        // MOTOR CAN BUS IDS TODO: CONFIRM
        public static final int kMotorID = 41; // CONFIRM
        // RAW PID CONSTANTS TODO: TUNE
        public static final double kP = 0.025;
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
        public static final double kTolerance = 2.0; // TODO: TUNE
        // LIMIT VALUES
        public static final double kMinimumRotationLimit = -150; // TODO: SET
        public static final double kMaximumRotationLimit = 150; // TODO: SET
        public static final double kMinimumOutputLimit = -.5;
        public static final double kMaximumOutputLimit = .5;
        // INVERSIONS TODO: CONFIRM
        public static final boolean kInverted = true;
        public static final boolean kAbsoluteEncoderInverted = false;
        // CONVERSION FACTOR AND OFFSETS
        public static final double kPositionConversionFactor = 360.0;
        public static final double kOffset = 0.2262425; // TODO: SET
        public static final boolean kZeroCentered = true;
        // CURRENT LIMITS TODO: TUNE        
        public static final int kStallLimit = 20;
        public static final int kFreeLimit = 10;
        // IDLE MODE
        public static final IdleMode kIdleMode = IdleMode.kBrake;
    }
    
    public static class RotateWristConstants {
        // MOTOR CAN BUS IDS TODO: CONFIRM
        public static final int kMotorID = 42;
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
        public static final double kMAXMotionAllowedError = 1.0;
        // TOLERANCE FOR PID ERROR
        public static final double kTolerance = 2.0; // TODO: TUNE
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
        public static final double kOffset = 0.5804630; // TODO: SET
        public static final boolean kZeroCentered = true;
        // CURRENT LIMITS TODO: TUNE
        public static final int kStallLimit = 20;
        public static final int kFreeLimit = 10;
        // IDLE MODE
        public static final IdleMode kIdleMode = IdleMode.kBrake;
    }
}
