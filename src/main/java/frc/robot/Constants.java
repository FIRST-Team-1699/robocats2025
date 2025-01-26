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
        public static final double kIntakeSpeed = .5;
        public static final double kOuttakeSpeed = -.49;

        public static final int kMotorID = -1;


    }

    public static class RotateConstants {
        public static final double kP = -1;
        public static final double kI = -1;
        public static final double kD = -1;
        public static final double kFF = -1;
        public static final double kTolerance = -1;
    }

    public static class TiltConstants {
        public static final double kP = -1;
        public static final double kI = -1;
        public static final double kD = -1;
        public static final double kFF = -1;
        public static final double kTolerance = -1;
    }

}
