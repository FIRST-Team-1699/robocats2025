package frc.robot.subsystems;

import frc.robot.Constants.PivotConstants;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
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

public class PivotSubsystem implements Subsystem {
    //TODO: USE ABSOLUTE ENCODER FOR INITALIZATION, RELATIVE OTHERWISE
    // MOTORS
    private SparkMax leadMotor, followMotor;
    // ENCODERS
    private RelativeEncoder encoder;
    private AbsoluteEncoder absoluteEncoder;
    // FEEDBACK CONTROLLER
    private SparkClosedLoopController feedbackController;
    // CURRENT POSITION
    private PivotPosition currentTargetPosition;

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
        // CONFIGURE MOTORS
        configureMotors();
    }

    /** Sets the configurations for each motor. */
    private void configureMotors() {
        SparkMaxConfig leadConfig = new SparkMaxConfig();
        // LEADER CONFIG
        leadConfig
            .inverted(PivotConstants.kInverted)
            .idleMode(IdleMode.kBrake)
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
        leadConfig.softLimit
            .forwardSoftLimit(PivotConstants.kMaximumRotationLimit)
            .forwardSoftLimitEnabled(true)
            .reverseSoftLimit(PivotConstants.kMinimumRotationLimit)
            .reverseSoftLimitEnabled(true);
        leadMotor.configureAsync(leadConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        // FOLLOWER CONFIG
        SparkMaxConfig followConfig = new SparkMaxConfig();
        followConfig.apply(leadConfig);
        followConfig.follow(leadMotor, PivotConstants.kFollowerInverted);
        followMotor.configureAsync(followConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /** Changes hieght/angle of pivot.
     * @param pivotPosition
     * enum that has height value for target position.
     */
    public Command setPosition(PivotPosition pivotPosition) {
        return runOnce(() -> {
            currentTargetPosition = pivotPosition;
            feedbackController.setReference(pivotPosition.getDegrees(), SparkBase.ControlType.kPosition);
        });
    }

    /**gets the direction of Pivot, used for determining order of command groups.
     * @param newPosition
     * The new position to determine direction
     */
    public boolean isPivotRising(PivotPosition newPosition) {
        return (newPosition.getDegrees() - currentTargetPosition.getDegrees() > 0 );
    }

    public Command waitUntilAtSetpoint() {
        return new WaitUntilCommand(() -> {
            return isAtSetpoint();
        });
    }

    public boolean isAtSetpoint() {
        return getError() < PivotConstants.kTolerance;
    }
    
    private double getError() {
        return Math.abs(Math.abs(encoder.getPosition()) - Math.abs(currentTargetPosition.getDegrees()));
    }

    /**Ensures that motor is set to 0 after triggering bottomLimitSwitch*/
    public Command stopMotorCommand() {
        return runOnce(() -> {
            leadMotor.set(0);
        });
    }

    public Command setRaw(double percentage) {
        return run(() -> {
            leadMotor.set(percentage);
        });
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Actual Pivot Angle", absoluteEncoder.getPosition());
        SmartDashboard.putNumber("Wanted Pivot Angle", currentTargetPosition.getDegrees());
    }
    
    /**Enum, holds position of pivot.
     * @param degreePositionOne
     * Height Pivot must reach to get to state.
     */
    public enum PivotPosition{
        STORED(-1), PRIME(-1), COBRA_STANCE(-1),

        ALGAE_INTAKE(-1), ALGAE_DESCORE_L_TWO(-1), ALGAE_DESCORE_L_THREE(-1),
        GROUND_INTAKE(-1), CORAL_STATION_INTAKE(-1),

        L_ONE(-1), L_TWO(-1), L_THREE(-1), L_FOUR(-1);
        private double degrees;
        PivotPosition(double degrees) {
            this.degrees = degrees;
        }

        public double getDegrees() {
            return this.degrees;
        }
    }
}