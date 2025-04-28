package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.utils.LimelightHelpers;

public class AlignToReefWithID extends Command {
    public static final double targetTZ = -.5;
    public static final double leftTargetTX = -.17;
    public static final double rightTargetTX = .17;
    public static final double tolerance = .05;

    public static final Translation2d leftOffsetTranslation = new Translation2d(targetTZ, leftTargetTX);
    public static final Translation2d rightOffsetTranslation = new Translation2d(targetTZ, rightTargetTX);

    private PIDController translationController = new PIDController(3.5, 0, 0);
    private final PIDController rotationalController = new PIDController(.1, 0, 0.01);

    private Translation2d targetOffsetTranslation;

    private CommandSwerveDrivetrain swerve;
    private boolean left;
    private int id;

    private boolean thetaInTolerance;
    private boolean forwardInTolerance;
    private boolean horizontalInTolerance;

    public AlignToReefWithID(CommandSwerveDrivetrain swerve, boolean left, int id) {
        this.swerve = swerve;
        this.left = left;
        this.id = id;
        addRequirements(swerve);
        thetaInTolerance = false;
        forwardInTolerance = false;
        horizontalInTolerance = false;
        targetOffsetTranslation = left ? leftOffsetTranslation : rightOffsetTranslation;
    }

    @Override
    public void initialize() {
        translationController.setSetpoint(0);
    }

    @Override
    public void execute() {
        if(LimelightHelpers.getTV("limelight") && LimelightHelpers.getFiducialID("limelight") == id) {
            double[] cameraPoseInTagSpace = LimelightHelpers.getBotPose_TargetSpace("limelight");
            Translation2d cameraTranslation = new Translation2d(cameraPoseInTagSpace[2], cameraPoseInTagSpace[0]);
            Translation2d errorTranslation = targetOffsetTranslation.minus(cameraTranslation);

            double translationErrorMagnitude = errorTranslation.getNorm();

            double translationOutputMagnitude = translationController.calculate(translationErrorMagnitude);

            Translation2d translationOutput = new Translation2d(translationOutputMagnitude, Rotation2d.fromDegrees(errorTranslation.getAngle().getDegrees() + cameraPoseInTagSpace[4]));
            System.out.println(translationOutput);

            double forwardOutput = -translationOutput.getX();
            double horizontalOutput = translationOutput.getY();

            // double forwardOutput = MathUtil.clamp(forwardController.calculate(cameraPoseInTagSpace[2], targetTZ), -1.5, 1.5);
            // double horizontalOutput = MathUtil.clamp(-horizontalController.calculate(cameraPoseInTagSpace[0], left ? leftTargetTX : rightTargetTX), -1.5, 1.5);
            double rotationalOutput = MathUtil.clamp(-rotationalController.calculate(cameraPoseInTagSpace[4], 0), -2.5, 2.5);

            if(inTolerance(cameraPoseInTagSpace[2], targetTZ, tolerance)) {
                forwardOutput = 0;
                forwardInTolerance = true;
            } else {
                forwardInTolerance = false;
            }

            if(inTolerance(cameraPoseInTagSpace[0], left ? leftTargetTX : rightTargetTX, tolerance)) {
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

            swerve.setControl(new SwerveRequest.RobotCentric().withVelocityX(forwardOutput).withVelocityY(horizontalOutput).withRotationalRate(rotationalOutput));
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
    }

    private static boolean inTolerance(double valueOne, double valueTwo, double tolerance) {
        return Math.abs(Math.abs(valueOne) - Math.abs(valueTwo)) < tolerance;
    }
}
