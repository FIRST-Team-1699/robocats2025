package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AlignToBargeConstants;
import frc.robot.Constants.AlignToReefConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.utils.LimelightHelpers;

public class AlignToBarge extends Command{
    private boolean limelightVisible = !!LimelightHelpers.getTV("limelight");
    private CommandSwerveDrivetrain swerve;

    private final PIDController translationController = new PIDController(3.5, 0, 0);
    private final PIDController rotationalController = new PIDController(.1, 0, 0.01);

    private boolean thetaInTolerance = false;
    private boolean forwardInTolerance = false;
    private boolean horizontalInTolerance = false;

    private Timer deadlineTimer = new Timer();

    public AlignToBarge(CommandSwerveDrivetrain swerve) {
        this.swerve = swerve;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        translationController.setSetpoint(0);
        deadlineTimer.start();
    }

    @Override
    public void execute() {
        double[] cameraPoseInTagSpace = LimelightHelpers.getBotPose_TargetSpace("limelight");

        Translation2d cameraTranslation = new Translation2d(cameraPoseInTagSpace[2], cameraPoseInTagSpace[0]);
        Translation2d errorTranslation = AlignToBargeConstants.kOffsetTranslation.minus(cameraTranslation);

        double translationErrorMagnitude = errorTranslation.getNorm();
        double translationOutputMagnitude = translationController.calculate(translationErrorMagnitude);

        Translation2d translationOutput = new Translation2d(
            translationOutputMagnitude,
            Rotation2d.fromDegrees(errorTranslation.getAngle().getDegrees() + cameraPoseInTagSpace[4])
        );

        double forwardOutput = MathUtil.clamp(translationOutput.getX(), -1.5, 1.5);
        double horizontalOutput = MathUtil.clamp(translationOutput.getY(), -1.5, 1.5);

        double rotationalOutput = MathUtil.clamp(-rotationalController.calculate(cameraPoseInTagSpace[4],0),-1.5,1.5);
        
        if(inTolerance(cameraPoseInTagSpace[2], AlignToBargeConstants.kTargetTZ)) {
            forwardOutput = 0;
            forwardInTolerance = true;
        } else {
            forwardInTolerance = false;
        }
        if(inTolerance(cameraPoseInTagSpace[0], AlignToBargeConstants.kTargetTX)) {
            horizontalOutput = 0;
            horizontalInTolerance = true;
        } else {
            horizontalInTolerance = false;
        }
        if(Math.abs(cameraPoseInTagSpace[4]) < AlignToBargeConstants.kRotateTolerance) {
            rotationalOutput = 0;
            horizontalInTolerance = true;
        } else {
            horizontalInTolerance = false;
        }

        swerve.setControl(
            new SwerveRequest.RobotCentric()
                .withVelocityX(forwardOutput)
                .withVelocityY(horizontalOutput)
                .withRotationalRate(rotationalOutput)
        );
    }

    @Override
    public void end(boolean isInterupted) {
        if(!limelightVisible) {
            return;
        }
        swerve.setControl(new SwerveRequest.RobotCentric());
        deadlineTimer.stop();
        deadlineTimer.reset();
    }

    @Override
    public boolean isFinished() {
        return !limelightVisible || (thetaInTolerance && horizontalInTolerance && forwardInTolerance && LimelightHelpers.getTV("limelight")) || deadlineTimer.get() >= AlignToReefConstants.secTimerLimit;
    }

    private static boolean inTolerance(double valueOne, double valueTwo) {
        return Math.abs(Math.abs(valueOne) - Math.abs(valueTwo)) < AlignToBargeConstants.kMovementTolerance && valueOne * valueTwo > 0;
    }
}
