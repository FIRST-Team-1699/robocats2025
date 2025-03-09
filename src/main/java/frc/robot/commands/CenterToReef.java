package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.util.ReefDistanceSensor;

public class CenterToReef extends Command {
    private CommandSwerveDrivetrain swerve;
    private ReefDistanceSensor leftSensor, rightSensor, centerSensor;
    private ProfiledPIDController xController, thetaController;
    private boolean alignRight;

    public CenterToReef(CommandSwerveDrivetrain swerve, ReefDistanceSensor leftSensor, ReefDistanceSensor rightSensor, ReefDistanceSensor centerSensor, boolean alignRight) {
        this.swerve = swerve;
        this.leftSensor = leftSensor;
        this.rightSensor = rightSensor;
        this.centerSensor = centerSensor;
        addRequirements(swerve);

        xController = new ProfiledPIDController(0.001, 0, 0, new TrapezoidProfile.Constraints(2, .5));
        thetaController = new ProfiledPIDController(0.001, 0, 0, new TrapezoidProfile.Constraints(2, .5));

        this.alignRight = alignRight;
    }

    @Override
    public void initialize() {
        xController.setGoal(0);
        xController.setTolerance(5);
        thetaController.setGoal(0);
        thetaController.setTolerance(5);
    }

    @Override
    public void execute() {
        if(alignRight) {
            double thetaOutput = thetaController.calculate(leftSensor.getCenterError() - centerSensor.getCenterError());
            double xOutput = xController.calculate(-centerSensor.getCenterError());
            swerve.applyRequest(() -> new SwerveRequest.RobotCentric().withVelocityY(0).withVelocityX(xOutput).withRotationalRate(thetaOutput));
        } else {
            double thetaOutput = thetaController.calculate(rightSensor.getCenterError() - centerSensor.getCenterError());
            double xOutput = xController.calculate(-centerSensor.getCenterError());
            swerve.applyRequest(() -> new SwerveRequest.RobotCentric().withVelocityY(0).withVelocityX(xOutput).withRotationalRate(thetaOutput));
        }
    }

    @Override
    public void end(boolean interrupted) {
        swerve.applyRequest(() -> new SwerveRequest.PointWheelsAt().withModuleDirection(Rotation2d.fromDegrees(90)));
    }

    @Override
    public boolean isFinished() {
        if(xController.atGoal() && thetaController.atGoal()) {
            return true;
        }
        return false;
    }
}
