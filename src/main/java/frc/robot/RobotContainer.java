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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.PivotSubsystem.PivotPosition;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorPosition;

public class RobotContainer {
    // IO DEVICES
    private final CommandXboxController driverController = new CommandXboxController(0);
    private final CommandXboxController operatorController = new CommandXboxController(1);

    // // SWERVE COMMANDS
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(SwerveConstants.kTranslationalDeadband).withRotationalDeadband(SwerveConstants.kRotationalDeadband) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    // SWERVE 
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final Telemetry logger = new Telemetry(SwerveConstants.kMaxSpeed);

    private final ElevatorSubsystem elevator = new ElevatorSubsystem();
    private final PivotSubsystem pivot = new PivotSubsystem();
    private final IntakeSubsystem intake = new IntakeSubsystem();

    // LEDController ledcontroller = new LEDController(pivot, elevator, rotateWrits, tiltWrist);

    // if(!pivot.isAtSetpoint())

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

        // pivot.setDefaultCommand(pivot.printPosition());
        // elevator.setDefaultCommand(elevator.printPosition());

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
        // Raise/Lower Elvator
        // operatorController.povDown()
        //     .onTrue(elevator.setPosition(ElevatorPosition.L_ONE)
        //         .andThen(elevator.waitUntilAtSetpoint()));
        // operatorController.povLeft()
        //     .onTrue(elevator.setPosition(ElevatorPosition.L_TWO)
        //         .andThen(elevator.waitUntilAtSetpoint()));
        // operatorController.povRight()
        //     .onTrue(elevator.setPosition(ElevatorPosition.L_THREE)
        //         .andThen(elevator.waitUntilAtSetpoint()));
        // operatorController.povUp()
        //     .onTrue(elevator.setPosition(ElevatorPosition.L_FOUR)
        //         .andThen(elevator.waitUntilAtSetpoint()));
        // operatorController.b()
        //     .onTrue(elevator.setPosition(ElevatorPosition.COBRA_STANCE)
        //         .andThen(elevator.waitUntilAtSetpoint()));
        // operatorController.a()
        //     .onTrue(elevator.setPosition(ElevatorPosition.GROUND_INTAKE)
        //         .andThen(elevator.waitUntilAtSetpoint()));
        // operatorController.y()
        //     .onTrue(elevator.setPosition(ElevatorPosition.CORAL_STATION_INTAKE)
        //         .andThen(elevator.waitUntilAtSetpoint()));
        // operatorController.back()
        //     .onTrue(elevator.setPosition(ElevatorPosition.STORED)
        //         .andThen(elevator.waitUntilAtSetpoint()));
        // Zero
        // operatorController.leftBumper()
        //     .onTrue(new ZeroElevatorCommand(elevator));
        // operatorController.leftBumper()
        //     .whileTrue(elevator.toggleLowerLimit(false).andThen(elevator.lowerElevator().andThen(elevator.stopMotor().alongWith(elevator.resetEncoder().alongWith(elevator.toggleLowerLimit(true))))));
        // operatorController.leftBumper()
        //     .whileTrue(
        //         elevator.zeroElevator()
        //             .handleInterrupt(elevator::stopMotorManual)
        //         );

        // operatorController.povUp().whileTrue(elevator.setRaw(.2)).onFalse(elevator.setRaw(0));
        // operatorController.povDown().whileTrue(elevator.setRaw(-.2)).onFalse(elevator.setRaw(0));
        // operatorController.x().onTrue(elevator.setPosition(ElevatorPosition.STORED));
        // operatorController.y().onTrue(elevator.setPosition(ElevatorPosition.PID_TESTING).onlyIf(() -> pivot.currentTargetPosition != PivotPosition.STORED));

        // Operator Controller
        // operatorController.povUp().onTrue(pivot.setPosition(PivotPosition.L_FOUR)
        //     .andThen(pivot.waitUntilAtSetpoint()));
        // operatorController.povLeft().onTrue(pivot.setPosition(PivotPosition.L_ONE)
        //     .andThen(pivot.waitUntilAtSetpoint()));
        // operatorController.povDown().onTrue(pivot.setPosition(PivotPosition.L_TWO)
        //     .andThen(pivot.waitUntilAtSetpoint()));
        // operatorController.povRight().onTrue(pivot.setPosition(PivotPosition.L_THREE)
        //     .andThen(pivot.waitUntilAtSetpoint()));
        // operatorController.x().onTrue(pivot.setPosition(PivotPosition.GROUND_INTAKE)
        //     .andThen(pivot.waitUntilAtSetpoint()));
        // operatorController.y().onTrue(pivot.setPosition(PivotPosition.CORAL_STATION_INTAKE)
        //     .andThen(pivot.waitUntilAtSetpoint()));
        // operatorController.leftStick().onTrue(pivot.setPosition(PivotPosition.STORED)
        //     .andThen(pivot.waitUntilAtSetpoint()));
        // operatorController.rightStick().onTrue(pivot.setPosition(PivotPosition.COBRA_STANCE)
        //     .andThen(pivot.waitUntilAtSetpoint()));

        // operatorController.povRight().whileTrue(pivot.setRaw(.2)).onFalse(pivot.setRaw(0));
        // operatorController.povLeft().whileTrue(pivot.setRaw(-.2)).onFalse(pivot.setRaw(0));
        // operatorController.a().onTrue(pivot.setPosition(PivotPosition.STORED).onlyIf(() -> elevator.currentTargetPosition == ElevatorPosition.STORED));
        // operatorController.b().onTrue(pivot.setPosition(PivotPosition.TESTING_PID));

        driverController.rightTrigger().whileTrue(intake.runIntake(.3)).onFalse(intake.stopMotorCommand());
        driverController.leftTrigger().whileTrue(intake.runIntake(-.3)).onFalse(intake.stopMotorCommand());
    }

    public Command getAutonomousCommand() {
        return AutoBuilder.buildAuto("Coral station lower Auto");
    }
}
