package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
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

    public static class VisionConstants {
        // TODO: EXPERIEMENT W/ ON ACTUAL ROBOT
        public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(-1,-1,-1);
        public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(-1,-1,-1);
        public static final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
    }
    
}
