package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.TiltWristConstants;

public class TiltWristSubsystem extends SubsystemBase {
    // TODO: CONFIG/ DECIDE DYNAMIC V. DEFAULT
    private final MotionMagicVoltage m_request = new MotionMagicVoltage(TiltWristConstants.kOffset);

    private TalonFX motor;

    private TiltPosition currentTargetPosition;

    private TalonFXConfiguration talonConfigs;
    private MotorOutputConfigs motorConfigs;
    private FeedbackConfigs feedback;

    /**Constructor for Subsystem */
    public TiltWristSubsystem() {
        motor = new TalonFX(TiltWristConstants.kMotorID);

        currentTargetPosition = TiltPosition.STORED;

        configureMotors();
    }

    /**Configures motor, encoder and closed loop for subsystem  */
    private void configureMotors() {
        talonConfigs = new TalonFXConfiguration();
        motorConfigs = new MotorOutputConfigs();
        feedback = new FeedbackConfigs();

        motorConfigs.Inverted = TiltWristConstants.kInverted;
        motorConfigs.PeakForwardDutyCycle = TiltWristConstants.kForwardLimit;
        motorConfigs.PeakReverseDutyCycle = TiltWristConstants.kReverseLimit;
        motorConfigs.NeutralMode = TiltWristConstants.kIdle;

        talonConfigs.Slot0.GravityType = TiltWristConstants.kGravityCounter;
        talonConfigs.Slot0.StaticFeedforwardSign = TiltWristConstants.kFeedForward;
        // EXAMPLE K VALUES
        talonConfigs.Slot0.kS = TiltWristConstants.kS;
        talonConfigs.Slot0.kV = TiltWristConstants.kV;
        talonConfigs.Slot0.kA = TiltWristConstants.kA;
        talonConfigs.Slot0.kP = TiltWristConstants.kP;
        talonConfigs.Slot0.kI = TiltWristConstants.kI;
        talonConfigs.Slot0.kD = TiltWristConstants.kD;

        // EXAMPLE MOTION VALUES
        talonConfigs.MotionMagic.MotionMagicCruiseVelocity = TiltWristConstants.kMotionMagicVelocity;
        talonConfigs.MotionMagic.MotionMagicAcceleration = TiltWristConstants.kMotionMagicAcceleration;
        talonConfigs.MotionMagic.MotionMagicJerk =  TiltWristConstants.kMotionMagicJerk;

        feedback.SensorToMechanismRatio = TiltWristConstants.kPositionConversionFactor;

        motor.getConfigurator().apply(talonConfigs.Slot0);
        motor.getConfigurator().apply(talonConfigs.MotionMagic);
        motor.getConfigurator().apply(motorConfigs);
        motor.getConfigurator().apply(feedback);
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
            // COMMENTING THIS CODE OUT AS SAFETY
            motor.setControl(m_request.withPosition(currentTargetPosition.degreePosition));
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
        return Math.abs(Math.abs(currentTargetPosition.degreePosition) - Math.abs(getPosition()));
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
        return motor.getPosition().getValueAsDouble();
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
        return () -> currentTargetPosition == TiltPosition.L_FOUR || currentTargetPosition == TiltPosition.L_THREE || currentTargetPosition == TiltPosition.L_TWO || currentTargetPosition == TiltPosition.L_FOUR_FRONT || currentTargetPosition == TiltPosition.L_THREE_FRONT;
    }

    public BooleanSupplier isInL3L4() {
        return () -> currentTargetPosition == TiltPosition.L_FOUR || currentTargetPosition == TiltPosition.L_THREE || currentTargetPosition == TiltPosition.L_FOUR_FRONT || currentTargetPosition == TiltPosition.L_THREE_FRONT;
    }

    public Command printPosition() {
        return run(() -> System.out.println(getPosition()));
    }

    public void setIdleMode(NeutralModeValue idleMode) {
        motorConfigs.NeutralMode = idleMode;
        motor.getConfigurator().apply(motorConfigs);
        
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Actual Tilt Wrist Angle", getPosition());
        SmartDashboard.putNumber("Wanted Tilt Wrist Angle", currentTargetPosition.degreePosition);
        SmartDashboard.putBoolean("At Tilt Setpoint", isAtSetpoint());
        SmartDashboard.putBoolean("Is In Scoring Tilt", isInL2L3L4().getAsBoolean());
    }

    /**Contains desired position for rotational positions */
    public enum TiltPosition {
        STORED(-10), PRIME(-10), COBRA_STANCE(-1),

        CLIMB_UPPER(0), CLIMB_LOWER(-60),

        ALGAE_INTAKE(-1), 
        
        ALGAE_DESCORE_L_TWO(20), ALGAE_DESCORE_L_THREE(20),

        GROUND_INTAKE_HORIZONTAL(55), GROUND_INTAKE_VERTICAL(35), CORAL_STATION_INTAKE(-90), // -1

        L_ONE(25), L_TWO(-15), L_THREE(-15), L_FOUR(-30), L_FOUR_FRONT(5), L_THREE_FRONT(0),
        L_TWO_PECK(20), L_THREE_PECK(-70), L_FOUR_PECK(-75), L_FOUR_FRONT_PECK(55), L_THREE_FRONT_PECK(40),

        BARGE_SCORE(-24.5);

        double degreePosition;
        private TiltPosition(double degreePosition) {
            this.degreePosition = degreePosition;
        }
    }
}
