package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
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
        // TODO: EXPERIEMENT W/ ON ACTUAL ROBOT OR REPLACE IF THERE IS A BETTER SYSTEM
        public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(0,0,0);
        public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0,0,0);

        // TODO: THE FOLLOWING METRICS ARE IN INCHES. IF INCHES DOES NOT COMPLY WITH THE TRANSFORM3D DEFAULT UNIT, THEN CONSIDER CONVERTING TO DESIRED UNITS
        // TODO: MAKING ASSUMPTION THAT DIRECTIONS IN CAD ARE SAME AS TRANSFORM 3D. IF THEY AREN'T: 
            //1: I WILL CRY (WHY WPILib, WHY NO DIRECT STATEMENT IN DOCS: https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/math/geometry/Transform3d.html)
            //2: VALUES WILL BE SWITCHED AROUND
        // TODO: Verify with CHASE THAT CAMERAONE IS SLIGHTLY TILTED DOWN (From Ground). The concern is that I have maybe inverted the angle, I put +4.647 in my notes, but it could be inverted 
        public static final Transform3d kRobotToCameraOne = new Transform3d(6.881, 14.965,8.85, new Rotation3d(0,94.674,164.27832));
        public static final Transform3d kRobotToCameraTwo = new Transform3d(-5.875,-10.035994,7.598710, new Rotation3d(0,65,0));
        
        //TODO: ADD ACORDING FIELD (Welded is a conditional to consider based on attended competetions)
        public static final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded); 
    }
    
}
