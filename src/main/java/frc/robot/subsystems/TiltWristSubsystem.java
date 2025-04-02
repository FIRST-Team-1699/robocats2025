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

import frc.robot.Constants.TiltWristConstants;

public class TiltWristSubsystem extends SubsystemBase {
    private SparkMax motor;
    private SparkAbsoluteEncoder absoluteEncoder;
    private SparkClosedLoopController feedbackController;

    private SparkMaxConfig motorConfig;

    private TiltPosition currentTargetPosition;

    /**Constructor for Subsystem */
    public TiltWristSubsystem() {
        motor = new SparkMax(TiltWristConstants.kMotorID, MotorType.kBrushless);

        absoluteEncoder = motor.getAbsoluteEncoder();

        feedbackController = motor.getClosedLoopController();

        currentTargetPosition = TiltPosition.STORED;

        configureMotors();
    }

    /**Configures motor, encoder and closed loop for subsystem  */
    private void configureMotors() {
        motorConfig = new SparkMaxConfig();
        
        motorConfig
            .inverted(TiltWristConstants.kInverted)
            .idleMode(TiltWristConstants.kIdleMode)
            .smartCurrentLimit(TiltWristConstants.kStallLimit, TiltWristConstants.kFreeLimit);
        motorConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            .pidf(TiltWristConstants.kP, TiltWristConstants.kI, TiltWristConstants.kD, TiltWristConstants.kFF, ClosedLoopSlot.kSlot0)
            .pidf(TiltWristConstants.kMAXMotionP, TiltWristConstants.kMAXMotionI, TiltWristConstants.kMAXMotionD, TiltWristConstants.kMAXMotionFF, ClosedLoopSlot.kSlot1)
            .outputRange(TiltWristConstants.kMinimumOutputLimit, TiltWristConstants.kMaximumOutputLimit, ClosedLoopSlot.kSlot0)
            .outputRange(TiltWristConstants.kMinimumOutputLimit, TiltWristConstants.kMaximumOutputLimit, ClosedLoopSlot.kSlot1)
        .maxMotion
            .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal, ClosedLoopSlot.kSlot1)
            .maxAcceleration(TiltWristConstants.kMAXMotionMaxAcceleration, ClosedLoopSlot.kSlot1)
            .maxVelocity(TiltWristConstants.kMAXMotionMaxVelocity, ClosedLoopSlot.kSlot1)
            .allowedClosedLoopError(TiltWristConstants.kMAXMotionAllowedError, ClosedLoopSlot.kSlot1);
        motorConfig.encoder
            .positionConversionFactor(TiltWristConstants.kPositionConversionFactor);
        motorConfig.absoluteEncoder
            .positionConversionFactor(TiltWristConstants.kPositionConversionFactor)
            .zeroOffset(TiltWristConstants.kOffset)
            .zeroCentered(TiltWristConstants.kZeroCentered)
            .inverted(TiltWristConstants.kAbsoluteEncoderInverted);
        // motorConfig.softLimit
        //     .forwardSoftLimit(TiltWristConstants.kMaximumRotationLimit)
        //     .forwardSoftLimitEnabled(true)
        //     .reverseSoftLimit(TiltWristConstants.kMinimumRotationLimit)
        //     .reverseSoftLimitEnabled(true);

        motor.configureAsync(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /**Sets Tilt position for writs
     * @param currentTargetPosition
     * Sets the currentTargetPosition to object and to PID controller
     * @param targetTiltPosition
     * Integer, if the target position-- being targeted-- is tilt position one or two.
     */
    public Command setPosition(TiltPosition currentTargetPosition) {
        return runOnce(() -> {
            this.currentTargetPosition = currentTargetPosition;
            feedbackController.setReference(currentTargetPosition.degreePosition, SparkBase.ControlType.kPosition);
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
        return getError() < TiltWristConstants.kTolerance;
    }

    /**Returns double, representing error between target position and actual position */
    public double getError() {
        return Math.abs(Math.abs(currentTargetPosition.degreePosition) - Math.abs(absoluteEncoder.getPosition()));
    }

    public boolean isAtLEDTolerance() {
        return getError() < 3.0;
    }
    /**returns command to stop motor */
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

    public TiltPosition getTargetPosition() {
        return currentTargetPosition;
    }

    public BooleanSupplier isInL4Position() {
        return () -> currentTargetPosition == TiltPosition.L_FOUR;
    }

    public BooleanSupplier isInL3Position() {
        return () -> currentTargetPosition == TiltPosition.L_THREE;
    }

    public BooleanSupplier isInL2Position() {
        return () -> currentTargetPosition == TiltPosition.L_TWO;
    }

    public BooleanSupplier isInL4FrontPosition() {
        return () -> currentTargetPosition == TiltPosition.L_FOUR_FRONT;
    }

    public BooleanSupplier isInL4PeckPosition() {
        return () -> currentTargetPosition == TiltPosition.L_FOUR_PECK;
    }

    public BooleanSupplier isInL3PeckPosition() {
        return () -> currentTargetPosition == TiltPosition.L_THREE_PECK;
    }

    public BooleanSupplier isInL2PeckPosition() {
        return () -> currentTargetPosition == TiltPosition.L_TWO_PECK;
    }

    public BooleanSupplier isInL4FrontPeckPosition() {
        return () -> currentTargetPosition == TiltPosition.L_FOUR_FRONT_PECK;
    }

    public BooleanSupplier isInL3FrontPosition() {
        return () -> currentTargetPosition == TiltPosition.L_THREE_FRONT;
    }

    public BooleanSupplier isInL3FrontPeckPosition() {
        return () -> currentTargetPosition == TiltPosition.L_THREE_FRONT_PECK;
    }

    public BooleanSupplier isInL2L3L4() {
        return () -> currentTargetPosition == TiltPosition.L_FOUR || currentTargetPosition == TiltPosition.L_THREE || currentTargetPosition == TiltPosition.L_TWO;
    }

    public BooleanSupplier isInL3L4() {
        return () -> currentTargetPosition == TiltPosition.L_FOUR || currentTargetPosition == TiltPosition.L_THREE || currentTargetPosition == TiltPosition.L_FOUR_FRONT || currentTargetPosition == TiltPosition.L_THREE_FRONT;
    }

    public Command printPosition() {
        return run(() -> System.out.println(getPosition()));
    }

    public void setIdleMode(IdleMode idleMode) {
        motorConfig.idleMode(idleMode);
        motor.configureAsync(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Actual Tilt Wrist Angle", absoluteEncoder.getPosition());
        SmartDashboard.putNumber("Wanted Tilt Wrist Angle", currentTargetPosition.degreePosition);
        SmartDashboard.putBoolean("At Tilt Setpoint", isAtSetpoint());
        SmartDashboard.putBoolean("Is In Scoring Tilt", isInL2L3L4().getAsBoolean());
    }

    /**Contains desired position for rotational positions */
    public enum TiltPosition {
        STORED(-110), PRIME(-30), COBRA_STANCE(-1),

        CLIMB_UPPER(0), CLIMB_LOWER(-60),

        ALGAE_INTAKE(-1), ALGAE_DESCORE_L_TWO(20), ALGAE_DESCORE_L_THREE(20),

        GROUND_INTAKE_HORIZONTAL(55), GROUND_INTAKE_VERTICAL(15), CORAL_STATION_INTAKE(-90), // -1

        L_ONE(25), L_TWO(-15), L_THREE(-15), L_FOUR(-30), L_FOUR_FRONT(5), L_THREE_FRONT(0),
        L_TWO_PECK(20), L_THREE_PECK(-70), L_FOUR_PECK(-75), L_FOUR_FRONT_PECK(55), L_THREE_FRONT_PECK(40);

        double degreePosition;
        private TiltPosition(double degreePosition) {
            this.degreePosition = degreePosition;
        }
    }
}
