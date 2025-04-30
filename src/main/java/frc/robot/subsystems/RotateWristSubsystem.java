package frc.robot.subsystems;

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

    public RotateWristSubsystem() {
        motor = new SparkMax(RotateWristConstants.kMotorID, MotorType.kBrushless);

        absoluteEncoder = motor.getAbsoluteEncoder();

        feedbackController = motor.getClosedLoopController();

        currentTargetPosition = RotatePosition.VERTICAL;

        configureMotors();
    }

    /** Sets the configurations for each motor. */
    private void configureMotors() {
        motorConfig = new SparkMaxConfig();
        
        motorConfig
            .inverted(RotateWristConstants.kInverted)
            .idleMode(RotateWristConstants.kIdleMode)
            .smartCurrentLimit(RotateWristConstants.kStallLimit, RotateWristConstants.kFreeLimit);
        motorConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            .pidf(RotateWristConstants.kP, RotateWristConstants.kI, RotateWristConstants.kD, RotateWristConstants.kFF, ClosedLoopSlot.kSlot0)
            .outputRange(RotateWristConstants.kMinimumOutputLimit, RotateWristConstants.kMaximumOutputLimit, ClosedLoopSlot.kSlot0);
        motorConfig.encoder
            .positionConversionFactor(RotateWristConstants.kPositionConversionFactor);
        motorConfig.absoluteEncoder
            .positionConversionFactor(RotateWristConstants.kPositionConversionFactor)
            .zeroOffset(RotateWristConstants.kOffset)
            .zeroCentered(RotateWristConstants.kZeroCentered)
            .inverted(RotateWristConstants.kAbsoluteEncoderInverted);
            
        motor.configureAsync(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /** Sets the target rotation of the wrist. */
    public Command setPosition(RotatePosition currentTargetPosition) {
        return runOnce(() -> {
            this.currentTargetPosition = currentTargetPosition;
            feedbackController.setReference(currentTargetPosition.degrees, SparkBase.ControlType.kPosition);
        });
    }

    public Command waitUntilAtSetpoint() {
        return new WaitUntilCommand(() -> {
            return isAtSetpoint();
        });
    }

    public boolean isAtSetpoint() {
        return getError() < RotateWristConstants.kTolerance;
    }

    public boolean isAtLEDTolerance() {
        return getError() < 3;
    }

    public double getError() {
        return Math.abs(Math.abs(currentTargetPosition.degrees) - Math.abs(absoluteEncoder.getPosition()));
    }

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

    public boolean isVertical() {
        return currentTargetPosition == RotatePosition.VERTICAL;
    }

    public boolean isVerticalFlipped() {
        return currentTargetPosition == RotatePosition.VERTICAL_FLIPPED;
    }

    public RotatePosition getRotatePosition() {
        return currentTargetPosition;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Actual Rotate Wrist Angle", absoluteEncoder.getPosition());
        SmartDashboard.putNumber("Wanted Rotate Wrist Angle", currentTargetPosition.degrees);
        SmartDashboard.putBoolean("At Rotate Setpoint", isAtSetpoint());
        SmartDashboard.putBoolean("IsVertical", isVertical());
        SmartDashboard.putBoolean("IsFlipped", isVerticalFlipped());
    }

    /** Enum for rotation setpoints. */
    public enum RotatePosition {
        VERTICAL(90), HORIZONTAL(0), VERTICAL_FLIPPED(-90);
        double degrees;
        private RotatePosition(double rotationDegrees) {
            this.degrees = rotationDegrees;
        }
    }
}