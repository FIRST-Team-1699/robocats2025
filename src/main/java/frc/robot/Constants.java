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

    public static class IntakeConstants {

        public static final int kMotorID = 40;

        public static final double kMAX_LIMIT = -1;
        public static final double kMIN_LIMIT = 1;
    }

    public static class RotateWristConstants {
        public static final double kMotorID = 42;

        public static final double kP = 0;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kFF = 0;
        public static final double kTolerance = 1;
        public static final double kConversionFactor = (100/360)*.24;

        // public static final double kMAX_LIMIT = -1;
        // public static final double kMIN_LIMIT = -1;
    }

    public static class TiltWristConstants {
        public static final double kP = 0;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kFF = 0;
        public static final double kTolerance = 1;
        public static final double kConversionFactor = 100/360;

        // public static final double kMAX_LIMIT = -1;
        // public static final double kMIN_LIMIT = -1;
    }
}
