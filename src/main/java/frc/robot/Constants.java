package frc.robot;

import static edu.wpi.first.units.Units.*;

import frc.robot.generated.TunerConstants;

public class Constants {
    public static class SwerveConstants {
        public static final double kMaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
        public static final double kMaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

        public static final double kTranslationalDeadband = kMaxSpeed * 0.1;
        public static final double kRotationalDeadband = kMaxAngularRate * 0.1;
    }
    public static class PivotConstants {
        public static final int kLeaderId = 44;
        public static final int kFollowerId = 43;
        
        public static final double kPOSITIONAL_CONVERSION = 1/360;
        public static final double kStoredToZeroDegrees = -1; //TODO: determine diffrence between 0 and stored

        public static final double kP = 0;
        public static final double kI = 0;
        public static final double kD = 0;

        public static final double kTOLERANCE = 1;

        public static final double kMAX_LIMIT = 235;
        public static final double kMIN_LIMIT = -45;
    }
}
