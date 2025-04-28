package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.AlignToReefConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.utils.LimelightHelpers;

public class AlignToReef extends Command {
    // private Translation2d lastStartPosition;

    private CommandSwerveDrivetrain swerve;

    private boolean left;

    private double lastHorizontalOutput;

    private final PIDController translationController = new PIDController(3.5, 0, 0);
    private final PIDController rotationalController = new PIDController(.1, 0, 0.01);

    private boolean thetaInTolerance;
    private boolean forwardInTolerance;
    private boolean horizontalInTolerance;
    
    private final Translation2d targetOffsetTranslation;

    private Timer deadlineTimer = new Timer();

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
        // DEADLINE LOGIC :o

        // NOT VISIBLE LIMELIGHT LOGIC
        if(!LimelightHelpers.getTV("limelight")) {
            if(left) {
                swerve.setControl(
                    new SwerveRequest.RobotCentric()
                        .withVelocityY(AlignToReefConstants.horizontalReAlignSpeed)
                );
            } else {
                swerve.setControl(
                    new SwerveRequest.RobotCentric()
                        .withVelocityY(-AlignToReefConstants.horizontalReAlignSpeed)
                );
            }
        }

        if(LimelightHelpers.getTV("limelight")) {

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

            double forwardOutput = MathUtil.clamp(-translationOutput.getX(), -1.5, 1.5);
            double horizontalOutput = MathUtil.clamp(translationOutput.getY(), -1.5, 1.5);

            double rotationalOutput = MathUtil.clamp(-rotationalController.calculate(cameraPoseInTagSpace[4], 0), -1.5, 1.5);

            if(inTolerance(cameraPoseInTagSpace[2], AlignToReefConstants.targetTZ, AlignToReefConstants.tolerance)) {
                System.out.println("forward in tolerance");
                forwardOutput = 0;
                forwardInTolerance = true;
            } else {
                System.out.println("forward not in tolerance");
                forwardInTolerance = false;
            }

            System.out.println(cameraPoseInTagSpace[2]);
            System.out.println(AlignToReefConstants.targetTZ);

            if(inTolerance(cameraPoseInTagSpace[0], 
                left ? AlignToReefConstants.leftTargetTX : AlignToReefConstants.rightTargetTX, 
                AlignToReefConstants.tolerance)) {

                horizontalOutput = 0;
                horizontalInTolerance = true;
            } else {
                horizontalInTolerance = false;
            }

            lastHorizontalOutput = horizontalOutput;

            if(Math.abs(cameraPoseInTagSpace[4]) < 1.0) {
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
        } else {
            if(lastHorizontalOutput < 0) {
                swerve.setControl(
                    new SwerveRequest.RobotCentric()
                        .withVelocityY(-AlignToReefConstants.horizontalReAlignSpeed)
                );
            } else {
                swerve.setControl(
                    new SwerveRequest.RobotCentric()
                        .withVelocityY(AlignToReefConstants.horizontalReAlignSpeed)
                );
            }
        }
    }

    @Override
    public boolean isFinished() {
        return (thetaInTolerance && horizontalInTolerance && forwardInTolerance && LimelightHelpers.getTV("limelight")) || deadlineTimer.get() >= AlignToReefConstants.autoAlignTimeLimit;
    }
    
    @Override
    public void end(boolean interrupted) {
        RobotContainer.isAligned = true;
        swerve.setControl(new SwerveRequest.RobotCentric());

        // REPORTS ALIGNMENT DATA TO DASHBOARD
        deadlineTimer.stop();

        // FOR TUNING
        deadlineTimer.reset(); 
    }

    private static boolean inTolerance(double valueOne, double valueTwo, double tolerance) {
        return Math.abs(Math.abs(valueOne) - Math.abs(valueTwo)) < tolerance && valueOne * valueTwo > 0;
    }
}
