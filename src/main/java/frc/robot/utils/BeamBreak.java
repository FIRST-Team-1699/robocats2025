package frc.robot.utils;

import java.util.function.BooleanSupplier;

import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;

import frc.robot.Constants.BeamBreakConstatnts;

public class BeamBreak {
    private static TimeOfFlight sensor = new TimeOfFlight(47);
    static {
        sensor.setRangingMode(RangingMode.Short, 3);
    }
    public static BooleanSupplier hasCoral() {
        return () -> sensor.getRange()<BeamBreakConstatnts.kHasCoralInRange;
    }
    public static boolean hasCoralBoolean() {
        return sensor.getRange()<BeamBreakConstatnts.kHasCoralInRange;
    }
    public static double getDistance() {
        return sensor.getRange();
    }
}
