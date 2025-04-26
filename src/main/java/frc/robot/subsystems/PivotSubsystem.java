
package frc.robot.subsystems;

import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.TiltWristSubsystem.TiltPosition;
import frc.robot.utils.Servo;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

import java.util.concurrent.ThreadPoolExecutor.AbortPolicy;
import java.util.function.BooleanSupplier;

import java.util.function.BooleanSupplier;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class PivotSubsystem extends SubsystemBase implements AutoCloseable {
    // TODO: USE ABSOLUTE ENCODER FOR INITALIZATION, RELATIVE OTHERWISE
    // MOTORS
    private SparkMax leadMotor, followMotor;
    // ENCODERS
    private RelativeEncoder encoder;
    private AbsoluteEncoder absoluteEncoder;
    // FEEDBACK CONTROLLER
    private SparkClosedLoopController feedbackController;
    // CONFIGS
    SparkMaxConfig leadConfig, followConfig;
    // CURRENT POSITION
    public PivotPosition currentTargetPosition;

    private ShuffleboardTab pivotTab;

    private Timer timer;
    private TrapezoidProfile trapezoid;
    private TrapezoidProfile climbTrapezoid;
    private ArmFeedforward feedforward;

    private double startingVelocity = 0;
    private double startingPosition;
    private boolean shouldMove;

    /** Constructs a pivot. */
    public PivotSubsystem() {
        // MOTORS
        leadMotor = new SparkMax(PivotConstants.kLeaderID, MotorType.kBrushless);
        followMotor = new SparkMax(PivotConstants.kFollowerID, MotorType.kBrushless);
        // ENCODERS
        absoluteEncoder = leadMotor.getAbsoluteEncoder();
        encoder = leadMotor.getEncoder();
        // PID/FEEDBACK CONTROLLER
        feedbackController = leadMotor.getClosedLoopController();
        // SETS TARGET POSITION
        currentTargetPosition = PivotPosition.STORED;
        startingPosition = currentTargetPosition.rotations;
        // CONFIGS
        leadConfig = new SparkMaxConfig();
        followConfig = new SparkMaxConfig();
        // CONFIGURE MOTORS
        configureMotors();
        configureShuffleboard();

        timer = new Timer();
        trapezoid = new TrapezoidProfile(new TrapezoidProfile.Constraints(200, 700));
        climbTrapezoid = new TrapezoidProfile(new TrapezoidProfile.Constraints(150, 450));
        feedforward = new ArmFeedforward(0, 0.523, 2.6);
        shouldMove = false;
    }

    /** Sets the configurations for each motor. */
    private void configureMotors() {
        // LEADER CONFIG
        leadConfig
            .inverted(PivotConstants.kInverted)
            .idleMode(PivotConstants.kIdleMode)
            .smartCurrentLimit(PivotConstants.kStallLimit, PivotConstants.kFreeLimit);
        leadConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            .pidf(PivotConstants.kP, PivotConstants.kI, PivotConstants.kD, PivotConstants.kFF, ClosedLoopSlot.kSlot0)
            .pidf(PivotConstants.kP, PivotConstants.kI, PivotConstants.kD, PivotConstants.kFF, ClosedLoopSlot.kSlot1)
            .outputRange(PivotConstants.kMinimumOutputLimit, PivotConstants.kMaximumOutputLimit, ClosedLoopSlot.kSlot0)
            .outputRange(PivotConstants.kMinimumClimbOutputLimit, PivotConstants.kMaximumClimbOutputLimit, ClosedLoopSlot.kSlot1);
        leadConfig.encoder
            .positionConversionFactor(PivotConstants.kPositionConversionFactor);
        leadConfig.absoluteEncoder
            .positionConversionFactor(PivotConstants.kPositionConversionFactor)
            .velocityConversionFactor(PivotConstants.kPositionConversionFactor)
            .zeroOffset(PivotConstants.kOffset)
            .zeroCentered(true)
            .inverted(PivotConstants.kAbsoluteEncoderInverted);
        leadConfig.softLimit
            .forwardSoftLimit(PivotConstants.kMaximumRotationLimit)
            .forwardSoftLimitEnabled(true)
            .reverseSoftLimit(PivotConstants.kMinimumRotationLimit)
            .reverseSoftLimitEnabled(true);
        leadMotor.configureAsync(leadConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        // FOLLOWER CONFIG
        followConfig.apply(leadConfig);
        followConfig.follow(leadMotor, PivotConstants.kFollowerInverted);
        followMotor.configureAsync(followConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    private void configureShuffleboard() {
        pivotTab = Shuffleboard.getTab("Pivot");
    }

    /** Changes hieght/angle of pivot.
     * @param pivotPosition
     * enum that has height value for target position.
     */
    @Deprecated
    public Command setOldPosition(PivotPosition pivotPosition) {
        return runOnce(() -> {
            Servo.getInstance().disableServo();
            currentTargetPosition = pivotPosition;
            feedbackController.setReference(pivotPosition.getRotations(), SparkBase.ControlType.kPosition, ClosedLoopSlot.kSlot0);
        });
    }

    public Command setPosition(PivotPosition position) {
        return runOnce(
            () -> {
                shouldMove = true;
                currentTargetPosition = position;
                timer.reset();
                timer.start();
                startingPosition = absoluteEncoder.getPosition();
                startingVelocity = absoluteEncoder.getVelocity();
            }
        );
    }

    private void runPID() {
        TrapezoidProfile.State setpoint = trapezoid.calculate(timer.get() + 0.02, new TrapezoidProfile.State(startingPosition, startingVelocity), new TrapezoidProfile.State(currentTargetPosition.rotations, 0));
        if(currentTargetPosition == PivotPosition.CLIMB_LOWER) {
            setpoint = climbTrapezoid.calculate(timer.get() + 0.02, new TrapezoidProfile.State(startingPosition, startingVelocity), new TrapezoidProfile.State(currentTargetPosition.rotations, 0));
        }
        feedbackController.setReference(setpoint.position, SparkBase.ControlType.kPosition, ClosedLoopSlot.kSlot0, feedforward.calculate(Rotation2d.fromDegrees(setpoint.position).getRadians(), Rotation2d.fromDegrees(setpoint.velocity).getRadians()));
    }

    public double getPositionRadians() {
        return Rotation2d.fromDegrees(absoluteEncoder.getPosition() + 90).getRadians();
    }

    public Command tuneFeedforwardPosition() {
        return startRun(() -> {
            timer.reset();
            timer.start();
        },
        () -> {
            TrapezoidProfile.State setpoint = trapezoid.calculate(timer.get() + .02, new TrapezoidProfile.State(-102, 0), new TrapezoidProfile.State(-90, 0));
            leadMotor.setVoltage(feedforward.calculate(Rotation2d.fromDegrees(setpoint.position).getRadians(), Rotation2d.fromDegrees(5).getRadians()));
        });
    }

    /** Changes height/angle of pivot.
     * @param pivotPosition
     * enum that has height value for target position.
     */
    public Command setClimbPosition() {
        return runOnce(() -> {
            currentTargetPosition = PivotPosition.CLIMB_LOWER;
            feedbackController.setReference(PivotPosition.CLIMB_LOWER.getRotations(), SparkBase.ControlType.kPosition, ClosedLoopSlot.kSlot1);
        });
    }

    public Command waitUntilAtSetpoint() {
        return new WaitUntilCommand(() -> {
            return isAtSetpoint();
        });
    }

    public Command waitUntilAtClimbSetpoint() {
        return new WaitUntilCommand(() -> {
            return isAtClimbSetpoint();
        });
    }

    /**Returns if currentTargetPosition is not at ground intake
     */
    public Command moveToSafePosition() {
        return setPosition(PivotPosition.SAFE_POSITION).onlyIf(() -> !currentTargetPosition.canElevatorRetractFromHere());
    }

    public boolean isAtSetpoint() {
        return getError() < PivotConstants.kTolerance;
    }

    public boolean isAtClimbSetpoint() {
        return getError() < 3.1;
    }

    public boolean isAtLEDTolerance() {
        return getError() < 2.5;
    }
    
    private double getError() {
        return Math.abs(Math.abs(getPosition()) - Math.abs(currentTargetPosition.getRotations()));
    }

    public BooleanSupplier isInGroundIntakePosition() {
        return () -> currentTargetPosition == PivotPosition.GROUND_INTAKE;
    }

    public boolean boolIsGroundIntakePosition() {
        return currentTargetPosition == PivotPosition.GROUND_INTAKE;
    }

    public void disableMovement() {
        shouldMove = false;
    }

    public double getPosition() {
        return absoluteEncoder.getPosition();
    }

    public Command printPosition() {
        return run(() -> System.out.println(getPosition()));
    }

    public BooleanSupplier isClimbReady() {
        return () -> { return isAtSetpoint() && currentTargetPosition == PivotPosition.CLIMB_RAISE; };
    }

    public void setIdleMode(IdleMode idleMode) {
        leadConfig.idleMode(idleMode);
        leadMotor.configureAsync(leadConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        followConfig.idleMode(idleMode);
        followMotor.configureAsync(followConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    @Override
    public void close() {
        leadMotor.close();
        followMotor.close();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Actual Pivot Angle", absoluteEncoder.getPosition());
        SmartDashboard.putNumber("Wanted Pivot Angle", currentTargetPosition.getRotations());
        SmartDashboard.putNumber("Pivot Error", getError());
        SmartDashboard.putBoolean("Pivot At Setpoint", isAtSetpoint());
        SmartDashboard.putNumber("Output Current", leadMotor.getOutputCurrent());
        SmartDashboard.putBoolean("Safe Zone", currentTargetPosition.canElevatorRetractFromHere());

        // pivotTab.("Setpoint", currentTargetPosition.getRotations());
        // pivotTab.add("Current Position", absoluteEncoder.getPosition());
        // pivotTab.add("At Setpoint", isAtSetpoint());
        if(shouldMove) {
            runPID();
        } else {
            leadMotor.set(0);
        }
    }
    
    /**Enum, holds position of pivot.
     * @param degreePositionOne
     * Height Pivot must reach to get to state.
     */
    public enum PivotPosition {
        STORED(-70), PRIME(-60), SAFE_POSITION(-75), COBRA_STANCE(-1),
        CLIMB_RAISE(-25), CLIMB_LOWER(-106.5),

        ALGAE_INTAKE(-1), ALGAE_DESCORE_L_TWO(-67), ALGAE_DESCORE_L_THREE(-47),
        GROUND_INTAKE(-95), CORAL_STATION_INTAKE(-8), // -50

        L_ONE(-70), L_TWO(-60), L_THREE(0), L_FOUR(0),
        L_FOUR_FRONT(-22), L_THREE_FRONT(-36);
        private double rotations;
        PivotPosition(double rotations) {
            this.rotations = rotations;
        }

        public double getRotations() {
            return this.rotations;
        }

        public boolean canElevatorRetractFromHere() {
            return this.rotations > PivotConstants.kUnsafePosition;
        }
    }
}