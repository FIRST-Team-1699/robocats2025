package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.util.ReefDistanceSensor;

public class AlignReefHorizontal extends Command {
    private CommandSwerveDrivetrain swerve;
    private ReefDistanceSensor leftSensor, rightSensor, centerSensor;
    private ProfiledPIDController yController;
    private boolean alignRight;

    public AlignReefHorizontal(CommandSwerveDrivetrain swerve, ReefDistanceSensor leftSensor, ReefDistanceSensor rightSensor, ReefDistanceSensor centerSensor, boolean alignRight) {
        this.swerve = swerve;
        this.leftSensor = leftSensor;
        this.rightSensor = rightSensor;
        this.centerSensor = centerSensor;
        addRequirements(swerve);

        yController = new ProfiledPIDController(0.001, 0, 0, new TrapezoidProfile.Constraints(2, .5));

        this.alignRight = alignRight;
    }

    @Override
    public void initialize() {
        yController.setGoal(0);
        yController.setTolerance(5);
    }

    @Override
    public void execute() {
        if(alignRight) {
            swerve.applyRequest(() -> new SwerveRequest.RobotCentric().withVelocityX(0).withVelocityY(yController.calculate(rightSensor.getSideError())).withRotationalRate(0)).schedule();;
        } else {
            swerve.applyRequest(() -> new SwerveRequest.RobotCentric().withVelocityX(0).withVelocityY(yController.calculate(leftSensor.getSideError())).withRotationalRate(0)).schedule();;
        }
    }

    @Override
    public void end(boolean interrupted) {
        swerve.applyRequest(() -> new SwerveRequest.PointWheelsAt().withModuleDirection(Rotation2d.fromDegrees(90))).schedule();;
    }

    @Override
    public boolean isFinished() {
        if(yController.atGoal()) {
            return true;
        }
        return false;
    }
}
