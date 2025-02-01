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
        public static final double kP = -1;
        public static final double kI = -1;
        public static final double kD = -1;

        public static final double kTOLERANCE = 1;

        public static final double kMAX_LIMIT = -1;
        public static final double kMIN_LIMIT = -1;
    }
}
