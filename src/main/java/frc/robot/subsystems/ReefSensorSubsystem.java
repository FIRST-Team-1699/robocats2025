// package frc.robot.subsystems;

// import java.util.function.BooleanSupplier;

// import au.grapplerobotics.LaserCan;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
// import frc.robot.Constants.ReefSensorConstants;
// import au.grapplerobotics.interfaces.LaserCanInterface.RangingMode;


// // GOAL OF SENSORS:
// // 1: AUTONOMOUS MOVEMENTS TO SPECIFIED AREA (LEFT OR RIGHT RELATIVE TO ROBOT)
// // 2: EASILY CALLABLE FOR TELEOP/ CMDS FOR AUTO

// public class ReefSensorSubsystem extends SubsystemBase{
//     LaserCan laserLeft, laserRight;
    
//     public ReefSensorSubsystem() {
//         laserLeft = new LaserCan(ReefSensorConstants.kLeftLaserID);
//         laserRight = new LaserCan(ReefSensorConstants.kRightLaserID);

//         configureLasers();
//     }

//     private void configureLasers() {
//         try {
//             laserLeft.setRangingMode(RangingMode.LONG);
//             laserLeft.setRegionOfInterest(ReefSensorConstants.kLeftRegionOfIntrest);
//             laserLeft.setTimingBudget(ReefSensorConstants.K_TIMING_BUDGET);

//             laserRight.setRangingMode(RangingMode.LONG);
//             laserRight.setRegionOfInterest(ReefSensorConstants.kRightRegionOfIntrest);
//             laserRight.setTimingBudget(ReefSensorConstants.K_TIMING_BUDGET);

//         } catch (Exception e) {
//             System.out.println("configuring Lasers failed: "+ e);
//         }
//     }


//     /**Checks for if lasers are within a assigned tolerance of eachother in detecting reef.*/
//     public BooleanSupplier isAtAlignedPosition() {
//         return (()  -> Math.abs(laserLeft.getMeasurement().distance_mm- laserRight.getMeasurement().distance_mm)
//             < ReefSensorConstants.kAlignmentTolerance);
//     } 

//     // TODO: DECIDE IF WE WANT ETHER DIRECTION TO WORK ON THE SAME CALL, OR IF WE WANT IT SPLIT INTO 2 METHODS
//     public BooleanSupplier isAtScorePositon() {
//         return (() -> {
//             if(Math.abs(laserLeft.getMeasurement().distance_mm - laserRight.getMeasurement().distance_mm) > ReefSensorConstants.kReefDetectionTolerance) {
//                 return true;
//             }
//             return false;
//         });
//     }

//     // TODO: VERIFY RETURNING DOUBLE WILL BE SCHEDULED
//     /**Returns  double for fixing rotation alignment.*/
//     public double fixRotationError() {
//         return (laserLeft.getMeasurement().distance_mm > laserRight.getMeasurement().distance_mm) ? -.1 : .1;
//     }

//     /**Returns wait command. Waits while the robot moves to a scoring position. */
//     public Command waitUntilAtScorePosition() {
//         return new WaitUntilCommand(() -> {
//             return isAtScorePositon().getAsBoolean();
//         });
//     }

//     /**Returns wait command. Waits while robot is correcting its rotation. */
//     public Command waitUntilAligned() {
//         return new WaitUntilCommand(() -> {
//             return isAtAlignedPosition().getAsBoolean();
//         });
//     } 

//     @Override
//     public void periodic() {
//         SmartDashboard.putBoolean("Is aligned: ", isAtAlignedPosition().getAsBoolean());
//         SmartDashboard.putBoolean("Reached position: ", isAtScorePositon().getAsBoolean());

//     }
// }
