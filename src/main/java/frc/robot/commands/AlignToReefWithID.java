package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.utils.LimelightHelpers;

public class AlignToReefWithID extends Command {
    public static final Pose2d offsetToLeftPole = new Pose2d();
    public static final Pose2d offsetToRightPose = new Pose2d();
    private CommandSwerveDrivetrain swerve;
    private int targetID;
    private boolean left;

    private final TrapezoidProfile.Constraints translationConstraints = new TrapezoidProfile.Constraints(2, 1);
    private final TrapezoidProfile.Constraints thetaConstraints = new TrapezoidProfile.Constraints(2, 1);
    private final ProfiledPIDController xController = new ProfiledPIDController(.1, 0, 0, translationConstraints);
    private final ProfiledPIDController yController = new ProfiledPIDController(.1, 0, 0, translationConstraints);
    private final ProfiledPIDController thetaController = new ProfiledPIDController(.1, 0, 0, thetaConstraints);

    public AlignToReefWithID(CommandSwerveDrivetrain swerve, int targetID, boolean left) {
        this.swerve = swerve;
        this.targetID = targetID;
        this.left = left;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        if(LimelightHelpers.getFiducialID("limelight") == targetID) {
            Pose3d cameraPoseInTagSpace = LimelightHelpers.getCameraPose3d_TargetSpace("limelight");
            double xOutput = xController.calculate(cameraPoseInTagSpace.getX(), left ? offsetToLeftPole.getX() : offsetToRightPose.getX());
            double yOutput = yController.calculate(cameraPoseInTagSpace.getY(), left ? offsetToLeftPole.getY() : offsetToRightPose.getY());
            double thetaOutput = thetaController.calculate(cameraPoseInTagSpace.getRotation().getZ(), 0);
            swerve.setControl(new SwerveRequest.RobotCentric().withVelocityX(xOutput).withVelocityY(yOutput).withRotationalRate(thetaOutput));
        } else {
            swerve.setControl(new SwerveRequest.RobotCentric());
        }
    }

    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return super.isFinished();
    }
    
    @Override
    public void end(boolean interrupted) {
        // TODO Auto-generated method stub
        super.end(interrupted);
    }
}
