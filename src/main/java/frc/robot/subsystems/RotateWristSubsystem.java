package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkAbsoluteEncoder;
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

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

import frc.robot.Constants.RotateWristConstants;

public class RotateWristSubsystem extends SubsystemBase {
    private SparkMax motor;
    private SparkAbsoluteEncoder absoluteEncoder;
    private SparkClosedLoopController feedbackController;

    private SparkMaxConfig motorConfig;

    private RotatePosition currentTargetPosition;

    /**Constructor for Subsystem */
    public RotateWristSubsystem() {
        motor = new SparkMax(RotateWristConstants.kMotorID, MotorType.kBrushless);

        absoluteEncoder = motor.getAbsoluteEncoder();

        feedbackController = motor.getClosedLoopController();

        currentTargetPosition = RotatePosition.VERTICAL;

        configureMotors();
    }

    /**Configures motor, encoder and closed loop for subsystem  */
    private void configureMotors() {
        motorConfig = new SparkMaxConfig();
        
        motorConfig
            .inverted(RotateWristConstants.kInverted)
            .idleMode(RotateWristConstants.kIdleMode)
            .smartCurrentLimit(RotateWristConstants.kStallLimit, RotateWristConstants.kFreeLimit);
        motorConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            .pidf(RotateWristConstants.kP, RotateWristConstants.kI, RotateWristConstants.kD, RotateWristConstants.kFF, ClosedLoopSlot.kSlot0)
            .pidf(RotateWristConstants.kMAXMotionP, RotateWristConstants.kMAXMotionI, RotateWristConstants.kMAXMotionD, RotateWristConstants.kMAXMotionFF, ClosedLoopSlot.kSlot1)
            .outputRange(RotateWristConstants.kMinimumOutputLimit, RotateWristConstants.kMaximumOutputLimit, ClosedLoopSlot.kSlot0)
            .outputRange(RotateWristConstants.kMinimumOutputLimit, RotateWristConstants.kMaximumOutputLimit, ClosedLoopSlot.kSlot1)
        .maxMotion
            .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal, ClosedLoopSlot.kSlot1)
            .maxAcceleration(RotateWristConstants.kMAXMotionMaxAcceleration, ClosedLoopSlot.kSlot1)
            .maxVelocity(RotateWristConstants.kMAXMotionMaxVelocity, ClosedLoopSlot.kSlot1)
            .allowedClosedLoopError(RotateWristConstants.kMAXMotionAllowedError, ClosedLoopSlot.kSlot1);
        motorConfig.encoder
            .positionConversionFactor(RotateWristConstants.kPositionConversionFactor);
        motorConfig.absoluteEncoder
            .positionConversionFactor(RotateWristConstants.kPositionConversionFactor)
            .zeroOffset(RotateWristConstants.kOffset)
            .zeroCentered(RotateWristConstants.kZeroCentered)
            .inverted(RotateWristConstants.kAbsoluteEncoderInverted);
        // motorConfig.softLimit
        //     .forwardSoftLimit(RotateWristConstants.kMaximumRotationLimit)
        //     .forwardSoftLimitEnabled(true)
        //     .reverseSoftLimit(RotateWristConstants.kMinimumRotationLimit)
        //     .reverseSoftLimitEnabled(true);
            
        motor.configureAsync(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /**Sets rotate position for writs
     * @param currentTargetPosition
     * Sets the currentTargetPosition to object and to PID controller
     */
    public Command setPosition(RotatePosition currentTargetPosition) {
        return runOnce(() -> {
            this.currentTargetPosition = currentTargetPosition;
            feedbackController.setReference(currentTargetPosition.degrees, SparkBase.ControlType.kPosition);
        });
    }

    /**Waits until within an acceptable range for PID (Tolerence), via calling isAtSetpoint */
    public Command waitUntilAtSetpoint() {
        return new WaitUntilCommand(() -> {
            return isAtSetpoint();
        });
    }

    /**Returns boolean if getError is within tolerence*/
    public boolean isAtSetpoint() {
        return getError() < RotateWristConstants.kTolerance;
    }

    public boolean isAtLEDTolerance() {
        return getError() < 3;
    }

    /**Returns double, representing error between target position and actual position */
    public double getError() {
        return Math.abs(Math.abs(currentTargetPosition.degrees) - Math.abs(absoluteEncoder.getPosition()));
    }

    /**Returns a command to stop motor */
    public Command stopMotorCommand() {
        return runOnce(() -> {
            motor.set(0);
        });
    }

    public Command setRaw(double percentage) {
        return runOnce(() -> {
            motor.set(percentage);
        });
    }

    public double getPosition() {
        return absoluteEncoder.getPosition();
    }

    public Command printPosition() {
        return run(() -> System.out.println(getPosition()));
    }

    public void setIdleMode(IdleMode idleMode) {
        motorConfig.idleMode(idleMode);
        motor.configureAsync(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public BooleanSupplier isVertical() {
        return () -> currentTargetPosition == RotatePosition.VERTICAL;
    }

    public BooleanSupplier isVerticalFlipped() {
        return () -> currentTargetPosition == RotatePosition.VERTICAL_FLIPPED;
    }

    public RotatePosition getRotatePosition() {
        return currentTargetPosition;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Actual Rotate Wrist Angle", absoluteEncoder.getPosition());
        SmartDashboard.putNumber("Wanted Rotate Wrist Angle", currentTargetPosition.degrees);
        SmartDashboard.putBoolean("At Rotate Setpoint", isAtSetpoint());
    }

    /**Contains desired position for rotational positions */
    public enum RotatePosition {
        VERTICAL(90), HORIZONTAL(0), VERTICAL_FLIPPED(-90);
        double degrees;
        private RotatePosition(double rotationDegrees) {
            this.degrees = rotationDegrees;
        }
    }
}