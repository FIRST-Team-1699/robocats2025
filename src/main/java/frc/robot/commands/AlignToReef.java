package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.utils.LimelightHelpers;

public class AlignToReef extends Command {
    public static final double targetTZ = -.48;
    public static final double leftTargetTX = -.17;
    public static final double rightTargetTX = .17;
    public static final double tolerance = .02;
    private CommandSwerveDrivetrain swerve;
    private boolean left;

    // private final TrapezoidProfile.Constraints translationConstraints = new TrapezoidProfile.Constraints(1, 1);
    // private final TrapezoidProfile.Constraints thetaConstraints = new TrapezoidProfile.Constraints(1, 1);
    // private final ProfiledPIDController xController = new ProfiledPIDController(.8, 0, 0, translationConstraints);
    // private final ProfiledPIDController forwardController = new ProfiledPIDController(1, 0, 0, translationConstraints);
    // private final ProfiledPIDController thetaController = new ProfiledPIDController(.4, 0, 0, thetaConstraints);
    private final PIDController forwardController = new PIDController(4, 0, 0);
    private final PIDController horizontalController = new PIDController(5, 0, 0);
    private final PIDController rotationalController = new PIDController(.1, 0, 0.01);

    public AlignToReef(CommandSwerveDrivetrain swerve, boolean left) {
        this.swerve = swerve;
        this.left = left;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        if(LimelightHelpers.getTV("limelight")) {
            double[] cameraPoseInTagSpace = LimelightHelpers.getBotPose_TargetSpace("limelight");
            double forwardOutput = MathUtil.clamp(forwardController.calculate(cameraPoseInTagSpace[2], targetTZ), -.8, .8);
            double horizontalOutput = MathUtil.clamp(-horizontalController.calculate(cameraPoseInTagSpace[0], left ? leftTargetTX : rightTargetTX), -.8, .8);
            double rotationalOutput = MathUtil.clamp(-rotationalController.calculate(cameraPoseInTagSpace[4], 0), -1.5, 1.5);
            if(inTolerance(cameraPoseInTagSpace[2], targetTZ, tolerance)) {
                forwardOutput = 0;
            }

            if(inTolerance(cameraPoseInTagSpace[0], left ? leftTargetTX : rightTargetTX, tolerance)) {
                horizontalOutput = 0;
            } else {
                if(Math.abs(Math.abs(targetTZ) - Math.abs(cameraPoseInTagSpace[2])) < .05) {
                    forwardOutput = 0;
                    System.out.println("WAITING FOR HORIZONTAL ALIGNMENT");
                }
            }

            if(inTolerance(cameraPoseInTagSpace[4], 0, 1)) {
                rotationalOutput = 0;
            }

            swerve.setControl(new SwerveRequest.RobotCentric().withVelocityX(forwardOutput).withVelocityY(horizontalOutput).withRotationalRate(rotationalOutput));
            System.out.println(cameraPoseInTagSpace[4]);
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
        // TODO Auto-generated method stub
        return super.isFinished();
    }
    
    @Override
    public void end(boolean interrupted) {
        // TODO Auto-generated method stub
        super.end(interrupted);
    }

    private static boolean inTolerance(double valueOne, double valueTwo, double tolerance) {
        return Math.abs(Math.abs(valueOne) - Math.abs(valueTwo)) < tolerance;
    }
}
