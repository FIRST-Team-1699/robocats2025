// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.SwerveConstants;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.RotateWristSubsystem;
import frc.robot.subsystems.TiltWristSubsystem;
import frc.robot.subsystems.IntakeSubsystem.IntakeSpeed;
import frc.robot.subsystems.RotateWristSubsystem.RotatePosition;
import frc.robot.subsystems.TiltWristSubsystem.TiltPosition;

public class RobotContainer {
    // IO DEVICES
    private final CommandXboxController driverController = new CommandXboxController(0);
    private final CommandXboxController operatorController = new CommandXboxController(1);

    // SWERVE COMMANDS
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(SwerveConstants.kTranslationalDeadband).withRotationalDeadband(SwerveConstants.kRotationalDeadband) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    // SWERVE CONFIG
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final Telemetry logger = new Telemetry(SwerveConstants.kMaxSpeed);

    // WRIST ROTATE
    private RotateWristSubsystem rotateWrist = new RotateWristSubsystem();

    // WRIST TILT
    private TiltWristSubsystem tiltWrist = new TiltWristSubsystem();

    // INTAKE 
    private IntakeSubsystem intake = new IntakeSubsystem();

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driverController.getLeftY() * SwerveConstants.kMaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driverController.getLeftX() * SwerveConstants.kMaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-driverController.getRightX() * SwerveConstants.kMaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        driverController.a().whileTrue(drivetrain.applyRequest(() -> brake));
        driverController.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-driverController.getLeftY(), -driverController.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        driverController.back().and(driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driverController.back().and(driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driverController.start().and(driverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driverController.start().and(driverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading
        driverController.y().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        // setup logger
        drivetrain.registerTelemetry(logger::telemeterize);

        // OPERATOR CONTROLLER
        operatorController.leftTrigger()
            .onTrue(rotateWrist.setPosition(RotatePosition.HORIZONTAL)
            .andThen(rotateWrist.waitUntilAtSetpoint()));
        operatorController.rightTrigger()
            .onTrue(rotateWrist.setPosition(RotatePosition.VERTICAL)
            .andThen(rotateWrist.waitUntilAtSetpoint()));

        operatorController.povRight().onTrue(tiltWrist.setPosition(TiltPosition.STORED)
            .andThen(tiltWrist.waitUntilAtSetpoint()));
        operatorController.povUp().onTrue(tiltWrist.setPosition(TiltPosition.CORAL_STATION_INTAKE)
            .andThen(tiltWrist.waitUntilAtSetpoint()));
        operatorController.povDown().onTrue(tiltWrist.setPosition(TiltPosition.GROUND_INTAKE)
            .andThen(tiltWrist.waitUntilAtSetpoint()));

        operatorController.x()
            .onTrue(intake.setWaitingIntake(IntakeSpeed.CORAL));
        operatorController.y()
            .onTrue(intake.setWaitingIntake(IntakeSpeed.ALGAE));
        operatorController.a()
            .onTrue(intake.setWaitingIntake(IntakeSpeed.DESCORE_ALGAE));
        operatorController.b()
            .onTrue(intake.setWaitingIntake(IntakeSpeed.STOP));

        operatorController.rightBumper()
            .whileTrue(intake.runIntake())
            .onFalse(intake.stopMotorCommand());
        operatorController.leftBumper()
            .onTrue(intake.runOutake())
            .onFalse(intake.stopMotorCommand());
    }

    public Command getAutonomousCommand() {
        return AutoBuilder.buildAuto("Test Auto");
    }
}
