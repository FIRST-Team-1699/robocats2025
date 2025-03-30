package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.utils.LimelightHelpers;

public class AlignToReef extends Command {
    public static final double targetTZ = -.48;
    public static final double leftTargetTX = -.17;
    public static final double rightTargetTX = .17;
    public static final double tolerance = .04;
    private CommandSwerveDrivetrain swerve;
    private boolean left;

    private final PIDController forwardController = new PIDController(4, 0, 0);
    private final PIDController horizontalController = new PIDController(7, 0, 0.1);
    private final PIDController rotationalController = new PIDController(.1, 0, 0.01);
    private boolean thetaInTolerance;
    private boolean forwardInTolerance;
    private boolean horizontalInTolerance;

    public AlignToReef(CommandSwerveDrivetrain swerve, boolean left) {
        this.swerve = swerve;
        this.left = left;
        addRequirements(swerve);
        thetaInTolerance = false;
        forwardInTolerance = false;
        horizontalInTolerance = false;
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        if(LimelightHelpers.getTV("limelight")) {
            double[] cameraPoseInTagSpace = LimelightHelpers.getBotPose_TargetSpace("limelight");
            double forwardOutput = MathUtil.clamp(forwardController.calculate(cameraPoseInTagSpace[2], targetTZ), -1.5, 1.5);
            double horizontalOutput = MathUtil.clamp(-horizontalController.calculate(cameraPoseInTagSpace[0], left ? leftTargetTX : rightTargetTX), -1.5, 1.5);
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
                // if(Math.abs(Math.abs(targetTZ) - Math.abs(cameraPoseInTagSpace[2])) < .6) {
                //     forwardOutput = 0;
                //     System.out.println("WAITING FOR HORIZONTAL ALIGNMENT");
                // }
                horizontalInTolerance = false;
            }

            if(inTolerance(cameraPoseInTagSpace[4], 0, 1)) {
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
        swerve.setControl(new SwerveRequest.RobotCentric());
    }

    private static boolean inTolerance(double valueOne, double valueTwo, double tolerance) {
        return Math.abs(Math.abs(valueOne) - Math.abs(valueTwo)) < tolerance;
    }
}
