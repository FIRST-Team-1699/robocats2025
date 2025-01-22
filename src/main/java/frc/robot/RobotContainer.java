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
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorPositions;
import frc.robot.commands.ElevatorLevelCommand;

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

    
    private final ElevatorSubsystem elevator = new ElevatorSubsystem();

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        // Driver
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

        // Operator
        //@KEVIN, Would you rather me do these Method Commands with ElevatorLevelCommand or how it is now?
        operatorController.povDown()
            .onTrue(elevator.elevatorHeight(ElevatorPositions.L_ONE)
                .andThen(elevator.waitUntilAtSetpoint()));
        operatorController.povLeft()
            .onTrue(elevator.elevatorHeight(ElevatorPositions.L_TWO)
                .andThen(elevator.waitUntilAtSetpoint()));
        operatorController.povRight()
            .onTrue(elevator.elevatorHeight(ElevatorPositions.L_THREE)
                .andThen(elevator.waitUntilAtSetpoint()));
        operatorController.povUp()
            .onTrue(elevator.elevatorHeight(ElevatorPositions.L_FOUR)
                .andThen(elevator.waitUntilAtSetpoint()));
        operatorController.b()
            .onTrue(elevator.elevatorHeight(ElevatorPositions.COBRA_STANCE)
                .andThen(elevator.waitUntilAtSetpoint()));
        operatorController.a()
            .onTrue(elevator.elevatorHeight(ElevatorPositions.GROUND_INTAKE)
                .andThen(elevator.waitUntilAtSetpoint()));
        operatorController.y()
            .onTrue(elevator.elevatorHeight(ElevatorPositions.SOURCE_INTAKE)
                .andThen(elevator.waitUntilAtSetpoint()));
        operatorController.back()
            .onTrue(elevator.elevatorHeight(ElevatorPositions.STORED)
                .andThen(elevator.waitUntilAtSetpoint()));

    }

    public Command getAutonomousCommand() {
        return AutoBuilder.buildAuto("Test Auto");
    }
}
