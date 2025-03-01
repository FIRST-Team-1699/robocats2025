package frc.robot.subsystems;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.TiltWristConstants;

public class TiltWristSubsystem implements Subsystem {
    private SparkMax motor;
    private SparkAbsoluteEncoder encoder;
    private SparkClosedLoopController feedbackController;

    private SparkMaxConfig motorConfig;

    private TiltPosition currentTargetPosition;

    /**Constructor for Subsystem */
    public TiltWristSubsystem() {
        motor = new SparkMax(-1, MotorType.kBrushless);

        encoder = motor.getAbsoluteEncoder();

        feedbackController = motor.getClosedLoopController();

        configureMotors();
    }

    /**Configures motor, encoder and closed loop for subsystem  */
    private void configureMotors() {
        motorConfig = new SparkMaxConfig();
        
        motorConfig
            .inverted(false) 
            .idleMode(IdleMode.kBrake);
        motorConfig.absoluteEncoder
            .positionConversionFactor(TiltWristConstants.kPositionConversionFactor);
        motorConfig.closedLoop
            .outputRange(-1,1)
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            .pidf(TiltWristConstants.kP, TiltWristConstants.kI, TiltWristConstants.kD, TiltWristConstants.kFF);
        // motorConfig.softLimit
        //     .forwardSoftLimit(TiltWristConstants.kMAX_LIMIT)
        //     .reverseSoftLimit(TiltWristConstants.kMIN_LIMIT)
        //     .forwardSoftLimitEnabled(true)
        //     .reverseSoftLimitEnabled(true);
        motor.configureAsync(motorConfig, SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
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
        return Math.abs(Math.abs(currentTargetPosition.degreePosition) - Math.abs(encoder.getPosition()));
    }

    /**returns command to stop motor */
    public Command stopMotorCommand() {
        return runOnce(() -> {
            motor.set(0);
        });
    }

    public Command setRaw(double degree) {
        return runOnce(() -> {
            motor.set(degree);
        });
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Actual Tilt Wrist Angle", encoder.getPosition());
        SmartDashboard.putNumber("Wanted Tilt Wrist Angle", currentTargetPosition.degreePosition);
    }

    /**Contains desired position for rotational positions */
    public enum TiltPosition {
        STORED(-1), PRIME(-1), COBRA_STANCE(-1),

        ALGAE_INTAKE(-1), ALGAE_DESCORE_PART_ONE(-1), ALGAE_DESCORE_PART_TWO(-1),
        GROUND_INTAKE(-1), CORAL_STATION_INTAKE(-1),

        L_ONE(-1), L_TWO(-1), L_THREE(-1), L_FOUR(-1);
        double degreePosition;
        private TiltPosition(double degreePosition) {
            this.degreePosition = degreePosition;
        }
    }
}
