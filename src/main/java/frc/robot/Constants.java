package frc.robot;

import static edu.wpi.first.units.Units.*;
import frc.robot.generated.TunerConstants;

public class Constants {
    public static class SwerveConstants {
        //SWERVE CONSTANTS
        public static final double kMaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
        public static final double kMaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

        public static final double kTranslationalDeadband = kMaxSpeed * 0.1;
        public static final double kRotationalDeadband = kMaxAngularRate * 0.1;
        
        //ELEVATOR CONSTANTS
        public static final double kELEVATOR_P = -1;
        public static final double kELEVATOR_I = -1;
        public static final double kELEVATOR_D = -1;

        public static final double kTOLEANCE = 1.0;
    }
}
