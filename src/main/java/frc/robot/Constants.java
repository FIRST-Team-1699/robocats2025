package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.generated.TunerConstants;

public class Constants {
    public static class SwerveConstants {
        public static final double kSpeedCoefficient = 0.85;
        public static final double kSlowCoefficient = 0.35;
        public static final double kMaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
        public static final double kMaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

        public static final double kTranslationalDeadband = kMaxSpeed * 0.1;
        public static final double kRotationalDeadband = kMaxAngularRate * 0.1;
    }
  
    public static class LEDConstants {
        public static final int kPort = 0;
        public static final int kLEDLength = 36;
    }
  
    public static class AutoConstants {
        public static final double kTranslationP = 10;
        public static final double kTranslationI = 0;
        public static final double kTranslationD = 0;
        
        public static final double kRotationP = 7;
        public static final double kRotationI = 0;
        public static final double kRotationD = 0;
    }

    public static class IntakeConstants {
        // SPARK MAX CAN BUS ID
        public static final int kMotorID = 40;
        // MOTOR INVERSION
        public static final boolean kInverted = true;
    }
    
    public static class ElevatorConstants {
        // SPARK MAX CAN BUS IDS
        public static final int kLeaderID = 46;
        public static final int kFollowerID = 45;
        // RAW PID CONSTANTS
        public static final double kP = .1;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kFF = 0;
        // TOLERANCE FOR PID ERROR
        public static final double kTolerance = 1.0;
        // LIMIT VALUES
        public static final double kMinimumRotationLimit = -5;
        public static final double kMaximumRotationLimit = 50;
        public static final double kMinimumOutputLimit = -.85;
        public static final double kMaximumOutputLimit = .85;
        // INVERSIONS
        public static final boolean kInverted = false;
        public static final boolean kFollowerInverted = true;
        // CURRENT LIMITS
        public static final int kStallLimit = 40;
        public static final int kFreeLimit = 40;
        // DEFAULT IDLE MODE
        public static final IdleMode kIdleMode = IdleMode.kBrake;
        // UNSAFE POSITION THRESHOLD
        public static final double kUnsafePosition = 7;
    }
    
    public static class PivotConstants {
        // SPARK MAX CAN BUS IDS
        public static final int kLeaderID = 44;
        public static final int kFollowerID = 43;
        // RAW PID CONSTANTS
        public static final double kP = .055;
        public static final double kI = 0;
        public static final double kD = 0.001;
        public static final double kFF = 0;
        // TOLERANCE FOR PID ERROR
        public static final double kTolerance = 1.5;
        public static final double kClimbTolerance = 3.1;
        // LIMIT VALUES
        public static final double kMinimumRotationLimit = -105;
        public static final double kMaximumRotationLimit = 5;
        public static final double kMinimumOutputLimit = -.7;
        public static final double kMaximumOutputLimit = .75;
        public static final double kMinimumClimbOutputLimit = -.35;
        public static final double kMaximumClimbOutputLimit = .35;
        // INVERSIONS
        public static final boolean kInverted = false;
        public static final boolean kFollowerInverted = true;
        public static final boolean kAbsoluteEncoderInverted = true;
        // OFFSET
        public static final double kOffset = 0.8174936;
        // CONVERSION FACTOR
        public static final double kPositionConversionFactor = 360.0;
        // CURRENT LIMITS
        public static final int kStallLimit = 100;
        public static final int kFreeLimit = 35;
        // IDLE MODE
        public static final IdleMode kIdleMode = IdleMode.kBrake;
        // DEFINING UNSAFE POSITION
        public static final double kUnsafePosition = -95;
    }
  
    public static class TiltWristConstants {
        // SPARK MAX CAN BUS IDS
        public static final int kMotorID = 41;
        // RAW PID CONSTANTS
        public static final double kP = 0.012;
        public static final double kI = 0;
        public static final double kD = 0.01;
        public static final double kFF = 0;
        // TOLERANCE FOR PID ERROR
        public static final double kTolerance = 2.0;
        // LIMIT VALUES
        public static final double kMinimumRotationLimit = -150;
        public static final double kMaximumRotationLimit = 150;
        public static final double kMinimumOutputLimit = -.5;
        public static final double kMaximumOutputLimit = .5;
        // INVERSIONS
        public static final boolean kInverted = true;
        public static final boolean kAbsoluteEncoderInverted = false;
        // CONVERSION FACTOR AND OFFSETS
        public static final double kPositionConversionFactor = 360.0;
        public static final double kOffset = 0.6048750;
        public static final boolean kZeroCentered = true;
        // CURRENT LIMITS    
        public static final int kStallLimit = 20;
        public static final int kFreeLimit = 10;
        // IDLE MODE
        public static final IdleMode kIdleMode = IdleMode.kBrake;
    }
    
    public static class RotateWristConstants {
        // MOTOR CAN BUS IDS
        public static final int kMotorID = 42;
        // RAW PID CONSTANTS
        public static final double kP = 0.008;
        public static final double kI = 0;
        public static final double kD = 0.001;
        public static final double kFF = 0;
        // TOLERANCE FOR PID ERROR
        public static final double kTolerance = 2.0;
        // LIMIT VALUES
        public static final double kMinimumRotationLimit = -90;
        public static final double kMaximumRotationLimit = 90;
        public static final double kMinimumOutputLimit = -.8;
        public static final double kMaximumOutputLimit = .8;
        // INVERSIONS
        public static final boolean kInverted = false;
        public static final boolean kAbsoluteEncoderInverted = false;
        // CONVERSION FACTOR AND OFFSETS
        public static final double kPositionConversionFactor = 360.0;
        public static final double kOffset = 0.5804630;
        public static final boolean kZeroCentered = true;
        // CURRENT LIMITS
        public static final int kStallLimit = 20;
        public static final int kFreeLimit = 10;
        // IDLE MODE
        public static final IdleMode kIdleMode = IdleMode.kBrake;
    }

    public static class AlignToReefConstants {
        // ALIGNMENT CONSTANTS
        public static final double targetTZ = -.5;
        public static final double leftTargetTX = -.17;
        public static final double rightTargetTX = .17;
        public static final double tolerance = .04;
        // TARGET TRANSLATIONS
        public static final Translation2d leftOffsetTranslation = new Translation2d(targetTZ, leftTargetTX);
        public static final Translation2d rightOffsetTranslation = new Translation2d(targetTZ, rightTargetTX);
        // SECONDS ALLOWED TO ALIGN IN AUTO TODO: TUNE
        public static final double autoAlignTimeLimit = 3;
        // SPEED TO HORIZONTALLY MOVE IF NO TAG IS VISIBLE
        public static final double horizontalReAlignSpeed = .2;
    }
}
