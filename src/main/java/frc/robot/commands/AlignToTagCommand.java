package frc.robot.commands;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AlignToTagCommand extends Command {
    private CommandSwerveDrivetrain swerve;
    private PhotonCamera camera;
    private double targetYaw, targetPitch, targetSkew;
    private int tagId;
    private final PIDController yawController = new PIDController(0.1, 0, 0);
    private final PIDController pitchController = new PIDController(0.1, 0, 0);
    private final PIDController skewController = new PIDController(0.1, 0, 0);
    private final double tolerance = 0.5;


    public AlignToTagCommand(CommandSwerveDrivetrain swerve, PhotonCamera camera, double targetYaw, double targetPitch, double targetSkew, int tagId) {
        this.swerve = swerve;
        this.camera = camera;
        this.targetYaw = targetYaw;
        this.targetPitch = targetPitch;
        this.targetSkew = targetSkew;
        this.tagId = tagId;

        addRequirements(swerve);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        var latestResults = camera.getAllUnreadResults();
        if (!latestResults.isEmpty()) {
            PhotonPipelineResult latestResult = latestResults.get(latestResults.size() - 1);
            if(!latestResult.getTargets().isEmpty()) {
                for (PhotonTrackedTarget target : latestResult.getTargets()) {
                    if (target.fiducialId == tagId) {
                        swerve.applyRequest(() ->
                            new SwerveRequest.RobotCentric().withVelocityX(pitchController.calculate(target.pitch, targetPitch)) // Drive forward with negative Y (forward)
                                .withVelocityY(skewController.calculate(target.skew, targetSkew)) // Drive left with negative X (left)
                                    .withRotationalRate(yawController.calculate(target.yaw, targetYaw)));
                        return;
                    }
                }
            }
        }
        swerve.applyRequest(() ->
            new SwerveRequest.RobotCentric().withVelocityX(0) // Drive forward with negative Y (forward)
                .withVelocityY(0) // Drive left with negative X (left)
                    .withRotationalRate(0));
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
    @Override
    public void end(boolean interrupted) {}
}