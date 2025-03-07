package frc.robot.subsystems;

import frc.robot.Constants.PivotConstants;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

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
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
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
        // CONFIGS
        leadConfig = new SparkMaxConfig();
        followConfig = new SparkMaxConfig();
        // CONFIGURE MOTORS
        configureMotors();
        configureShuffleboard();
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
            .pidf(PivotConstants.kMAXMotionP, PivotConstants.kMAXMotionI, PivotConstants.kMAXMotionD, PivotConstants.kMAXMotionFF, ClosedLoopSlot.kSlot1)
            .outputRange(PivotConstants.kMinimumOutputLimit, PivotConstants.kMaximumOutputLimit, ClosedLoopSlot.kSlot0)
            .outputRange(PivotConstants.kMinimumOutputLimit, PivotConstants.kMaximumOutputLimit, ClosedLoopSlot.kSlot1)
        .maxMotion
            .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal, ClosedLoopSlot.kSlot1)
            .maxAcceleration(PivotConstants.kMAXMotionMaxAcceleration, ClosedLoopSlot.kSlot1)
            .maxVelocity(PivotConstants.kMAXMotionMaxVelocity, ClosedLoopSlot.kSlot1)
            .allowedClosedLoopError(PivotConstants.kMAXMotionAllowedError, ClosedLoopSlot.kSlot1);
        leadConfig.encoder
            .positionConversionFactor(PivotConstants.kPositionConversionFactor);
        leadConfig.absoluteEncoder
            .positionConversionFactor(PivotConstants.kPositionConversionFactor)
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
    public Command setPosition(PivotPosition pivotPosition) {
        return runOnce(() -> {
            currentTargetPosition = pivotPosition;
            feedbackController.setReference(pivotPosition.getRotations(), SparkBase.ControlType.kPosition, ClosedLoopSlot.kSlot0);
        });
    }

    /** Changes height/angle of pivot.
     * @param pivotPosition
     * enum that has height value for target position.
     */
    public Command setClimbPosition() {
        return runOnce(() -> {
            currentTargetPosition = PivotPosition.CLIMB_LOWER;
            feedbackController.setReference(PivotPosition.CLIMB_LOWER.getRotations(), SparkBase.ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot1);
        });
    }

    public Command waitUntilAtSetpoint() {
        return new WaitUntilCommand(() -> {
            return isAtSetpoint();
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
    
    private double getError() {
        return Math.abs(Math.abs(getPosition()) - Math.abs(currentTargetPosition.getRotations()));
    }

    /**Ensures that motor is set to 0 after triggering bottomLimitSwitch*/
    public Command stopMotorCommand() {
        return runOnce(() -> {
            leadMotor.set(0);
        });
    }

    public Command setRaw(double percentage) {
        return runOnce(() -> {
            leadMotor.set(percentage);
        });
    }

    public double getPosition() {
        return absoluteEncoder.getPosition();
    }

    public Command printPosition() {
        return run(() -> System.out.println(getPosition()));
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
        SmartDashboard.putBoolean("Safe Zone", isRobotPositionSafe());
        SmartDashboard.putNumber("Output Current", leadMotor.getOutputCurrent());
        SmartDashboard.putBoolean("Safe Zone", currentTargetPosition.canElevatorRetractFromHere());

        // pivotTab.("Setpoint", currentTargetPosition.getRotations());
        // pivotTab.add("Current Position", absoluteEncoder.getPosition());
        // pivotTab.add("At Setpoint", isAtSetpoint());
    }
    
    /**Enum, holds position of pivot.
     * @param degreePositionOne
     * Height Pivot must reach to get to state.
     */
    public enum PivotPosition {
        STORED(-102), PRIME(0), SAFE_POSITION(-80), COBRA_STANCE(-1),
        CLIMB_RAISE(-25), CLIMB_LOWER(-50),

        ALGAE_INTAKE(-1), ALGAE_DESCORE_L_TWO(-1), ALGAE_DESCORE_L_THREE(-1),
        GROUND_INTAKE(-95), CORAL_STATION_INTAKE(-50),

        L_ONE(-60), L_TWO(-50), L_THREE(0), L_FOUR(0);
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