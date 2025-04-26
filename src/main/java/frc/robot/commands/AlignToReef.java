package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.AlignToReefConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.utils.LimelightHelpers;

public class AlignToReef extends Command {
    public static NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");

    // private Translation2d lastStartPosition;

    private CommandSwerveDrivetrain swerve;
    private boolean left;

    private final PIDController translationController = new PIDController(3.5, 0, 0);
    private final PIDController rotationalController = new PIDController(.1, 0, 0.01);

    private boolean thetaInTolerance;
    private boolean forwardInTolerance;
    private boolean horizontalInTolerance;
    
    private final Translation2d targetOffsetTranslation;

    private Timer deadlineTimer = new Timer();

    // FOR DETERMINING WHEN TO TRY TO RE-ALIGN
    private boolean reachedDeadline = false;

    public AlignToReef(CommandSwerveDrivetrain swerve, boolean left) {
        this.swerve = swerve;
        this.left = left;
        addRequirements(swerve);
        thetaInTolerance = false;
        forwardInTolerance = false;
        horizontalInTolerance = false;
        targetOffsetTranslation = left ? AlignToReefConstants.leftOffsetTranslation 
            : AlignToReefConstants.rightOffsetTranslation;
    }

    @Override
    public void initialize() {
        translationController.setSetpoint(0);

        // STARTS TIMER. USED TO PREVENT BEING STUCK AT END POSITION AT AN INTERVAL DEFINED IN COSTANTS (secTimerLimit). 
        // If IN TELE-OP, USED FOR DISPLAYING AUTO ALIGN TIME.
        deadlineTimer.start();
    }

    @Override
    public void execute() {
        if(DriverStation.isAutonomous() && (timerEnd() || !LimelightHelpers.getTV("limelight"))) {
            reachedDeadline = true;
            deadlineTimer.restart();
        }

        if(reachedDeadline) {
            if(left){
                swerve.setControl(
                    new SwerveRequest.RobotCentric()
                        .withVelocityX(-AlignToReefConstants.forwardReAlignSpeed)
                        .withVelocityY(AlignToReefConstants.horizontalReAlignSpeed)
                );
            } else {
                swerve.setControl(
                    new SwerveRequest.RobotCentric()
                        .withVelocityX(-AlignToReefConstants.forwardReAlignSpeed)
                        .withVelocityY(-AlignToReefConstants.horizontalReAlignSpeed)
                );                
            }

            // ALLOWS THE COMMAND TO BE CONTINUED IF LIMELIGHT IS VISIBLE AND ROBOT HAS BEEN MOVED (VIA MIN TIME)
            // TODO: IF THE LIMELIGHT IS NOT CONNECTED, CODE LIKE THIS SHOULD BE RAN BY DEFAULT!!!
            if((LimelightHelpers.getTV("limelight") && deadlineTimer.get() >= AlignToReefConstants.secReAlignMin) 
                || deadlineTimer.get() <= AlignToReefConstants.secReAlignMax) {
                swerve.setControl(new SwerveRequest.RobotCentric());
                deadlineTimer.restart();
                reachedDeadline = false;
            }
        }

        if(LimelightHelpers.getTV("limelight") 
            && (!reachedDeadline || !DriverStation.isAutonomous())) {

            double[] cameraPoseInTagSpace = LimelightHelpers.getBotPose_TargetSpace("limelight");
            Translation2d cameraTranslation = new Translation2d(cameraPoseInTagSpace[2], cameraPoseInTagSpace[0]);
            Translation2d errorTranslation = targetOffsetTranslation.minus(cameraTranslation);

            double translationErrorMagnitude = errorTranslation.getNorm();

            double translationOutputMagnitude = translationController.calculate(translationErrorMagnitude);

            Translation2d translationOutput = new Translation2d(
                translationOutputMagnitude, 
                Rotation2d.fromDegrees(errorTranslation.getAngle().getDegrees() + cameraPoseInTagSpace[4])
            );

            System.out.println(translationOutput);

            double forwardOutput = -translationOutput.getX();
            double horizontalOutput = translationOutput.getY();

            double rotationalOutput = MathUtil.clamp(-rotationalController.calculate(cameraPoseInTagSpace[4], 0), -2.5, 2.5);

            if(inTolerance(cameraPoseInTagSpace[2], AlignToReefConstants.targetTZ, AlignToReefConstants.tolerance)) {
                forwardOutput = 0;
                forwardInTolerance = true;
            } else {
                forwardInTolerance = false;
            }

            if(inTolerance(cameraPoseInTagSpace[0], 
                left ? AlignToReefConstants.leftTargetTX : AlignToReefConstants.rightTargetTX, 
                AlignToReefConstants.tolerance)) {

                horizontalOutput = 0;
                horizontalInTolerance = true;
            } else {
                horizontalInTolerance = false;
            }

            if(inTolerance(cameraPoseInTagSpace[4], 0, 1)) {
                System.out.println(cameraPoseInTagSpace[4]);
                rotationalOutput = 0;
                thetaInTolerance = true;
            } else {
                thetaInTolerance = false;
            }

            swerve.setControl(
                new SwerveRequest.RobotCentric()
                    .withVelocityX(forwardOutput)
                    .withVelocityY(horizontalOutput)
                    .withRotationalRate(rotationalOutput)
            );

            // System.out.println(cameraPoseInTagSpace[4]);
            // System.out.println("FORWARD OUTPUT: " + forwardOutput);
            // System.out.println("HORIZONTAL OUTPUT: " + horizontalOutput);
            // System.out.println("THETA OUTPUT: " + thetaOutput);
            // System.out.println("FORWARD ERROR: " + (cameraPoseInTagSpace[2] - targetTZ));
            // System.out.println("HORIZONTAL ERROR: " + (cameraPoseInTagSpace[0] - leftTargetTX));
            // System.out.println(cameraPoseInTagSpace[0]);
            // System.out.println(left ? leftTargetTX : rightTargetTX);
        } else {
            swerve.setControl(new SwerveRequest.RobotCentric());
        }
    }

    @Override
    public boolean isFinished() {
        return thetaInTolerance && horizontalInTolerance && forwardInTolerance;
    }
    
    @Override
    public void end(boolean interrupted) {
        RobotContainer.isAligned = true;
        swerve.setControl(new SwerveRequest.RobotCentric());

        // REPORTS ALIGNMENT DATA TO DASHBOARD
        reachedDeadline = timerEnd();
        deadlineTimer.stop();

        // FOR TUNING
        SmartDashboard.putNumber("Time to Align: ", deadlineTimer.get());
        deadlineTimer.reset(); 
    }

    private static boolean inTolerance(double valueOne, double valueTwo, double tolerance) {
        return Math.abs(Math.abs(valueOne) - Math.abs(valueTwo)) < tolerance;
    }

    /**Returns if Timer reached deadline*/
    private boolean timerEnd() {
        return deadlineTimer.get() >= AlignToReefConstants.secTimerLimit;
    }
}
