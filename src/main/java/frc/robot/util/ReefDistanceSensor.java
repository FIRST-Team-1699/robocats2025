package frc.robot.util;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.Measurement;
import au.grapplerobotics.interfaces.LaserCanInterface.RangingMode;
import au.grapplerobotics.interfaces.LaserCanInterface.RegionOfInterest;
import au.grapplerobotics.interfaces.LaserCanInterface.TimingBudget;

public class ReefDistanceSensor {
    private LaserCan sensor;
    private int mmFromCenter, mmFromSide;

    public ReefDistanceSensor(int canID, int mmFromCenter, int mmFromSide) {
        this.sensor = new LaserCan(canID);
        this.mmFromCenter = mmFromCenter;
        this.mmFromSide = mmFromSide;

        try {
            sensor.setRangingMode(RangingMode.SHORT);
            sensor.setTimingBudget(TimingBudget.TIMING_BUDGET_33MS);
            sensor.setRegionOfInterest(new RegionOfInterest(8, 8, 6, 6));
        } catch (ConfigurationFailedException e) {
            e.printStackTrace();
        }
    }

    public int getCenterError() {
        Measurement measurement = sensor.getMeasurement();
        if(measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
            return measurement.distance_mm - mmFromCenter;
        }
        return 0;
    }

    public int getSideError() {
        Measurement measurement = sensor.getMeasurement();
        if(measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
            return measurement.distance_mm - mmFromSide;
        }
        return 0;
    }
}
