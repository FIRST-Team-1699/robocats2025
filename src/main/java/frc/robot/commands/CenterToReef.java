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

        xController = new ProfiledPIDController(0.003, 0, 0, new TrapezoidProfile.Constraints(2, .5));
        thetaController = new ProfiledPIDController(0.01, 0, 0, new TrapezoidProfile.Constraints(2, .5));

        this.alignRight = alignRight;
    }

    @Override
    public void initialize() {
        System.out.println("CENTER TO REEF INITIALIZED");
    }

    @Override
    public void execute() {
        double rightError = rightSensor.getCenterError();
        double centerError = centerSensor.getCenterError();
        // if(alignRight) {
        //     double thetaOutput = thetaController.calculate(leftSensor.getCenterError() - centerSensor.getCenterError());
        //     double xOutput = xController.calculate(-centerSensor.getCenterError());
        //     swerve.applyRequest(() -> new SwerveRequest.RobotCentric().withVelocityY(0).withVelocityX(xOutput).withRotationalRate(thetaOutput)).schedule();;
        // } else {
        double thetaOutput;
        double xOutput = xController.calculate(-centerError, 0);
        if(rightError - centerError < 5) {
            thetaOutput = 0;
        } else {
            thetaOutput = thetaController.calculate(rightError - centerError, 0);
        }
        swerve.applyRequest(() -> new SwerveRequest.RobotCentric().withVelocityY(0).withVelocityX(xOutput).withRotationalRate(thetaOutput)).schedule();;
        // }
        System.out.println("CENTERING TO REEF");
        System.out.println("LEFT OUTPUT: " + xOutput);
        System.out.println("THETA OUTPUT: " + thetaOutput);
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("DONE CENTERING TO REEF" + interrupted);
        swerve.getDefaultCommand().schedule();
    }

    @Override
    public boolean isFinished() {
        double rightError = rightSensor.getCenterError();
        double centerError = centerSensor.getCenterError();
        if(Math.abs(Math.abs(rightError) - Math.abs(centerError)) < 5 && Math.abs(centerError) < 5) {
            return true;
        }
        return false;
    }
}
