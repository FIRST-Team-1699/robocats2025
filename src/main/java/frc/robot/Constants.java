package frc.robot;

import static edu.wpi.first.units.Units.*;

// import frc.robot.generated.TunerConstants;

public class Constants {
    public static class SwerveConstants {
        // public static final double kMaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
        public static final double kMaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

        // public static final double kTranslationalDeadband = kMaxSpeed * 0.1;
        public static final double kRotationalDeadband = kMaxAngularRate * 0.1;

    }
    public static class ElevatorConstants {
        public static final int kLeadID = 46;
        public static final int kFollowerID = 45;
        // PID CONSTANTS
        public static final double kP = 0;
        public static final double kI = 0;
        public static final double kD = 0;
        // TOLERANCE FOR PID ERROR
        public static final double kTOLERENCE = 1.0;
        // LIMIT VALUES
        public static final double kMIN_LIMIT = 0;
        public static final double kMAX_LIMIT = 50000;
    }
}
