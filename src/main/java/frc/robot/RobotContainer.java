// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ReefSensorConstants;
import frc.robot.Constants.SwerveConstants;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import java.util.Map;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SelectCommand;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LEDController;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.RotateWristSubsystem;
import frc.robot.subsystems.TiltWristSubsystem;
import frc.robot.subsystems.RotateWristSubsystem.RotatePosition;
import frc.robot.subsystems.TiltWristSubsystem.TiltPosition;
import frc.robot.util.ReefDistanceSensor;
import frc.robot.subsystems.PivotSubsystem;
// import frc.robot.subsystems.ReefSensorSubsystem;
import frc.robot.subsystems.PivotSubsystem.PivotPosition;
import frc.robot.subsystems.ElevatorSubsystem;
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
      
    // WRIST ROTATE
    private final RotateWristSubsystem rotateWrist = new RotateWristSubsystem();

    // WRIST TILT
    private final TiltWristSubsystem tiltWrist = new TiltWristSubsystem();

    private final ElevatorSubsystem elevator = new ElevatorSubsystem();
    private final PivotSubsystem pivot = new PivotSubsystem();
    private final IntakeSubsystem intake = new IntakeSubsystem();

    private final ReefDistanceSensor rightSensor = new ReefDistanceSensor(ReefSensorConstants.kRightID, ReefSensorConstants.kRightDistanceFromCenter, ReefSensorConstants.kRightDistanceFromSide);
    private final ReefDistanceSensor leftSensor = new ReefDistanceSensor(ReefSensorConstants.kLeftID, ReefSensorConstants.kLeftDistanceFromCenter, ReefSensorConstants.kLeftDistanceFromSide);
    private final ReefDistanceSensor centerSensor = new ReefDistanceSensor(ReefSensorConstants.kCenterID, ReefSensorConstants.kCenterDistanceFromCenter, ReefSensorConstants.kCenterDistanceFromSide);

    LEDController ledcontroller = new LEDController(elevator, pivot, tiltWrist, rotateWrist, intake);

    // if(!pivot.isAtSetpoint())

    // LED

    public RobotContainer() {
        // Adding commands so that they can be seen by pathplanner
        NamedCommands.registerCommand("Stored", pivot.moveToSafePosition()
                .andThen(pivot.waitUntilAtSetpoint())
                .andThen(elevator.setPosition(ElevatorPosition.STORED)
                .alongWith(tiltWrist.setPosition(TiltPosition.STORED)
                .alongWith(rotateWrist.setPosition(RotatePosition.VERTICAL)))
                .andThen(tiltWrist.waitUntilAtSetpoint())
                .andThen(elevator.waitUntilAtSetpoint())
                .andThen(pivot.setPosition(PivotPosition.STORED))));
        NamedCommands.registerCommand("Outake", intake.runIntake(-.5));
        NamedCommands.registerCommand("Move L4", elevator.setPosition(ElevatorPosition.STORED)
                .andThen(pivot.setPosition(PivotPosition.L_FOUR))
                .andThen(pivot.waitUntilAtSetpoint())
                .andThen(elevator.setPosition(ElevatorPosition.L_FOUR)
                .alongWith(rotateWrist.setPosition(RotatePosition.VERTICAL)
                .alongWith(tiltWrist.setPosition(TiltPosition.L_FOUR)))));

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

        // driverController.a().whileTrue(drivetrain.applyRequest(() -> brake));
        // driverController.b().whileTrue(drivetrain.applyRequest(() ->
        //     point.withModuleDirection(new Rotation2d(-driverController.getLeftY(), -driverController.getLeftX()))
        // ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // driverController.back().and(driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // driverController.back().and(driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // driverController.start().and(driverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // driverController.start().and(driverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // driverController.a()
        //     .onTrue(
        //         elevator.setPosition(ElevatorPosition.STORED)
        //         .andThen(elevator.waitUntilAtSetpoint())
        //         .andThen(pivot.setPosition(PivotPosition.CLIMB_RAISE))
        //         .alongWith(tiltWrist.setPosition(TiltPosition.CLIMB))
        //         .alongWith(rotateWrist.setPosition(RotatePosition.VERTICAL)));
        
        // driverController.x()
        //     // .onTrue(pivot.setClimbPosition());
        //     .whileTrue(pivot.setRaw(-.25))
        //     .onFalse(pivot.setRaw(0));  

        // driverController.b()
        //     // .onTrue(pivot.setClimbPosition());
        //     .whileTrue(pivot.setRaw(-.35))
        //     .onFalse(pivot.setRaw(0)); 

        // reset the field-centric heading
        driverController.y().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        // setup logger
        drivetrain.registerTelemetry(logger::telemeterize);

        driverController.rightTrigger()
            .onTrue(
                elevator.setPosition(ElevatorPosition.STORED)
                .andThen(elevator.waitUntilAtSetpoint())
                .andThen(pivot.setPosition(PivotPosition.SAFE_POSITION)
                .alongWith(tiltWrist.setPosition(TiltPosition.STORED).alongWith(rotateWrist.setPosition(RotatePosition.HORIZONTAL))))
                .andThen(pivot.waitUntilAtSetpoint())
                .andThen(elevator.setPosition(ElevatorPosition.GROUND_INTAKE))
                .andThen(elevator.waitUntilAtSetpoint())
                .andThen(tiltWrist.setPosition(TiltPosition.GROUND_INTAKE_HORIZONTAL)
                .alongWith(pivot.setPosition(PivotPosition.GROUND_INTAKE)))
                .andThen(intake.runIntake(.4))
            )
            .onFalse(
                new SelectCommand<>( 
                    Map.of(
                        true, 
                        pivot.moveToSafePosition()
                        .alongWith(tiltWrist.setPosition(TiltPosition.PRIME)
                        .alongWith(rotateWrist.setPosition(RotatePosition.VERTICAL)))
                        .andThen(pivot.waitUntilAtSetpoint())
                        .andThen(elevator.setPosition(ElevatorPosition.PRIME)
                        .andThen(tiltWrist.waitUntilAtSetpoint())
                        .andThen(elevator.waitUntilAtSetpoint())
                        .andThen(pivot.setPosition(PivotPosition.PRIME))),
                        false,
                        pivot.moveToSafePosition()
                        .alongWith(tiltWrist.setPosition(TiltPosition.STORED)
                        .alongWith(rotateWrist.setPosition(RotatePosition.VERTICAL)))
                        .andThen(pivot.waitUntilAtSetpoint())
                        .andThen(elevator.setPosition(ElevatorPosition.STORED)
                        .andThen(tiltWrist.waitUntilAtSetpoint())
                        .andThen(elevator.waitUntilAtSetpoint())
                        .andThen(pivot.setPosition(PivotPosition.STORED)))), 
                    intake::hasPiece
                ).alongWith(intake.stopMotorCommand())
            );

        driverController.leftTrigger()
        .onTrue(
            elevator.setPosition(ElevatorPosition.STORED)
            .andThen(elevator.waitUntilAtSetpoint())
            .andThen(pivot.setPosition(PivotPosition.SAFE_POSITION)
            .alongWith(tiltWrist.setPosition(TiltPosition.STORED)
            .alongWith(rotateWrist.setPosition(RotatePosition.VERTICAL))))
            .andThen(pivot.waitUntilAtSetpoint())
            .andThen(elevator.setPosition(ElevatorPosition.GROUND_INTAKE))
            .andThen(elevator.waitUntilAtSetpoint())
            .andThen(tiltWrist.setPosition(TiltPosition.GROUND_INTAKE_VERTICAL)
            .alongWith(pivot.setPosition(PivotPosition.GROUND_INTAKE)))
            .andThen(intake.runIntake(.4))
        )
            .onFalse(
                new SelectCommand<>( 
                    Map.of(
                        true, 
                        pivot.moveToSafePosition()
                        .alongWith(tiltWrist.setPosition(TiltPosition.PRIME)
                        .alongWith(rotateWrist.setPosition(RotatePosition.VERTICAL)))
                        .andThen(pivot.waitUntilAtSetpoint())
                        .andThen(elevator.setPosition(ElevatorPosition.PRIME)
                        .andThen(tiltWrist.waitUntilAtSetpoint())
                        .andThen(elevator.waitUntilAtSetpoint())
                        .andThen(pivot.setPosition(PivotPosition.PRIME))),
                        false,
                        pivot.moveToSafePosition()
                        .alongWith(tiltWrist.setPosition(TiltPosition.STORED)
                        .alongWith(rotateWrist.setPosition(RotatePosition.VERTICAL)))
                        .andThen(pivot.waitUntilAtSetpoint())
                        .andThen(elevator.setPosition(ElevatorPosition.STORED)
                        .andThen(tiltWrist.waitUntilAtSetpoint())
                        .andThen(elevator.waitUntilAtSetpoint())
                        .andThen(pivot.setPosition(PivotPosition.STORED)))), 
                    intake::hasPiece
                ).alongWith(intake.stopMotorCommand())
            );

        // operatorController.x().onTrue(elevator.setPosition(ElevatorPosition.STORED));
        // operatorController.y().onTrue(elevator.setPosition(ElevatorPosition.PID_TESTING).onlyIf(() -> pivot.currentTargetPosition != PivotPosition.STORED));

        // Operator Controller
        operatorController.rightTrigger().whileTrue(intake.runIntake(.4)).onFalse(intake.stopMotorCommand());
        operatorController.leftTrigger().whileTrue(intake.runIntake(-.3)).onFalse(intake.stopMotorCommand());


        operatorController.a()
            .onTrue(
                new SelectCommand<>( 
                    Map.of(true, 
                    pivot.moveToSafePosition()
                    .alongWith(tiltWrist.setPosition(TiltPosition.PRIME)
                    .alongWith(rotateWrist.setPosition(RotatePosition.VERTICAL)))
                    .andThen(pivot.waitUntilAtSetpoint())
                    .andThen(elevator.setPosition(ElevatorPosition.PRIME)
                    .andThen(tiltWrist.waitUntilAtSetpoint())
                    .andThen(elevator.waitUntilAtSetpoint())
                    .andThen(pivot.setPosition(PivotPosition.PRIME))),
                    false,
                    pivot.moveToSafePosition()
                    .alongWith(tiltWrist.setPosition(TiltPosition.STORED)
                    .alongWith(rotateWrist.setPosition(RotatePosition.VERTICAL)))
                    .andThen(pivot.waitUntilAtSetpoint())
                    .andThen(elevator.setPosition(ElevatorPosition.STORED)
                    .andThen(tiltWrist.waitUntilAtSetpoint())
                    .andThen(elevator.waitUntilAtSetpoint())
                    .andThen(pivot.setPosition(PivotPosition.STORED)))), 
                    intake::hasPiece
                )
            );    
        
        operatorController.povUp()
            .onTrue(
                (elevator.setPosition(ElevatorPosition.STORED)
                .andThen(pivot.setPosition(PivotPosition.L_FOUR))
                .andThen(pivot.waitUntilAtSetpoint())
                .andThen(elevator.setPosition(ElevatorPosition.L_FOUR)
                .alongWith(rotateWrist.setPosition(RotatePosition.VERTICAL)
                .alongWith(tiltWrist.setPosition(TiltPosition.L_FOUR)))))
                .unless(pivot.isInGroundIntakePosition())
            );
        
        operatorController.b()
            .onTrue(
                (elevator.moveToSafePosition()
                .andThen(elevator.waitUntilAtSetpoint())
                .andThen(elevator.setPosition(ElevatorPosition.STORED))
                .andThen(pivot.setPosition(PivotPosition.CORAL_STATION_INTAKE))
                .andThen(pivot.waitUntilAtSetpoint())
                .andThen(elevator.setPosition(ElevatorPosition.CORAL_STATION_INTAKE)
                .alongWith(rotateWrist.setPosition(RotatePosition.HORIZONTAL)
                .alongWith(tiltWrist.setPosition(TiltPosition.CORAL_STATION_INTAKE)))))
                .unless(pivot.isInGroundIntakePosition())
            );

        operatorController.povRight()
            .onTrue(
                (elevator.moveToSafePosition()
                .andThen(elevator.waitUntilAtSetpoint())
                .andThen(elevator.setPosition(ElevatorPosition.STORED))
                .andThen(pivot.setPosition(PivotPosition.L_THREE))
                .andThen(pivot.waitUntilAtSetpoint())
                .andThen(elevator.setPosition(ElevatorPosition.L_THREE)
                .alongWith(rotateWrist.setPosition(RotatePosition.VERTICAL)
                .alongWith(tiltWrist.setPosition(TiltPosition.L_THREE)))))
                .unless(pivot.isInGroundIntakePosition())
            );

        operatorController.povLeft()
            .onTrue(
                (elevator.moveToSafePosition()
                .andThen(elevator.waitUntilAtSetpoint())
                .andThen(elevator.setPosition(ElevatorPosition.STORED))
                .andThen(pivot.setPosition(PivotPosition.L_TWO))
                .andThen(pivot.waitUntilAtSetpoint())
                .andThen(elevator.setPosition(ElevatorPosition.L_TWO)
                .alongWith(rotateWrist.setPosition(RotatePosition.VERTICAL)
                .alongWith(tiltWrist.setPosition(TiltPosition.L_TWO)))))
                .unless(pivot.isInGroundIntakePosition())
            );
        
        operatorController.povDown()
            .onTrue(
                (elevator.moveToSafePosition()
                .andThen(elevator.waitUntilAtSetpoint())
                .andThen(elevator.setPosition(ElevatorPosition.STORED))
                .andThen(pivot.setPosition(PivotPosition.L_ONE))
                .andThen(pivot.waitUntilAtSetpoint())
                .andThen(elevator.setPosition(ElevatorPosition.L_ONE)
                .alongWith(rotateWrist.setPosition(RotatePosition.HORIZONTAL)
                .alongWith(tiltWrist.setPosition(TiltPosition.L_ONE)))))
                .unless(pivot.isInGroundIntakePosition())
            );

        operatorController.y()
            .onTrue(
                elevator.setPosition(ElevatorPosition.STORED)
                .andThen(pivot.setPosition(PivotPosition.ALGAE_DESCORE_L_THREE))
                .andThen(pivot.waitUntilAtSetpoint())
                .andThen(elevator.setPosition(ElevatorPosition.ALGAE_DESCORE_L_THREE)
                .alongWith(rotateWrist.setPosition(RotatePosition.HORIZONTAL)
                .alongWith(tiltWrist.setPosition(TiltPosition.ALGAE_DESCORE_L_THREE))))
                // elevator.setPosition(ElevatorPosition.STORED)
                // .andThen(elevator.waitUntilAtSetpoint())
                // .andThen(pivot.setPosition(PivotPosition.SAFE_POSITION)
                // .alongWith(tiltWrist.setPosition(TiltPosition.STORED).alongWith(rotateWrist.setPosition(RotatePosition.HORIZONTAL))))
                // .andThen(pivot.waitUntilAtSetpoint())
                // .andThen(elevator.setPosition(ElevatorPosition.GROUND_INTAKE))
                // .andThen(elevator.waitUntilAtSetpoint())
                // .andThen(tiltWrist.setPosition(TiltPosition.GROUND_INTAKE)
                // .alongWith(pivot.setPosition(PivotPosition.GROUND_INTAKE)))
            );

        operatorController.x()
            .onTrue(
                elevator.setPosition(ElevatorPosition.STORED)
                .andThen(pivot.setPosition(PivotPosition.ALGAE_DESCORE_L_TWO))
                .andThen(pivot.waitUntilAtSetpoint())
                .andThen(elevator.setPosition(ElevatorPosition.ALGAE_DESCORE_L_TWO)
                .alongWith(rotateWrist.setPosition(RotatePosition.HORIZONTAL)
                .alongWith(tiltWrist.setPosition(TiltPosition.ALGAE_DESCORE_L_TWO))))
            );

        operatorController.rightBumper()
            .onTrue(tiltWrist.setPosition(TiltPosition.L_THREE_PECK).onlyIf(tiltWrist.isInL3Position())
                .andThen(tiltWrist.setPosition(TiltPosition.L_FOUR_PECK).onlyIf(tiltWrist.isInL4Position()))
                .andThen(tiltWrist.setPosition(TiltPosition.L_TWO_PECK).onlyIf(tiltWrist.isInL2Position()))
            )
            .onFalse(tiltWrist.setPosition(TiltPosition.L_FOUR).onlyIf(tiltWrist.isInL4PeckPosition())
                .andThen(tiltWrist.setPosition(TiltPosition.L_THREE).onlyIf(tiltWrist.isInL3PeckPosition()))
                .andThen(tiltWrist.setPosition(TiltPosition.L_TWO).onlyIf(tiltWrist.isInL2PeckPosition()))
            );
    }

    public Command getAutonomousCommand() 
    {
        return AutoBuilder.buildAuto("Coral station lower Auto");
    }
}