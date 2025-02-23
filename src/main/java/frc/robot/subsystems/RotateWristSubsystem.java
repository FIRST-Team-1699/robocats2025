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
import frc.robot.Constants.RotateWristConstants;

public class RotateWristSubsystem implements Subsystem {
    private SparkMax motor;
    private SparkAbsoluteEncoder encoder;
    private SparkClosedLoopController feedbackController;

    private SparkMaxConfig motorConfig;

    private RotatePosition currentTargetPosition;

    /**Constructor for Subsystem */
    public RotateWristSubsystem() {
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
            .positionConversionFactor(RotateWristConstants.kConversionFactor);
        motorConfig.closedLoop
            .outputRange(-1,1)
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            .pidf(RotateWristConstants.kP, RotateWristConstants.kI, RotateWristConstants.kD, RotateWristConstants.kFF);
        // motorConfig.softLimit
        //     .forwardSoftLimit(RotateWristConstants.kMAX_LIMIT)
        //     .reverseSoftLimit(RotateWristConstants.kMIN_LIMIT)
        //     .forwardSoftLimitEnabled(true)
        //     .reverseSoftLimitEnabled(true);
        motor.configureAsync(motorConfig, SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
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

    /**Returns double, representing error between target position and actual position */
    public double getError() {
        return Math.abs(Math.abs(currentTargetPosition.degrees) - Math.abs(encoder.getPosition()));
    }

    /**Returns a command to stop motor */
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
        SmartDashboard.putNumber("Actual Rotate Wrist Angle", encoder.getPosition());
        SmartDashboard.putNumber("Wanted Rotate Wrist Angle", currentTargetPosition.degrees);
    }

    /**Contains desired position for rotational positions */
    public enum RotatePosition {
        VERTICAL(-1), HORIZONTAL(-1);
        double degrees;
        private RotatePosition(double rotationDegrees) {
            this.degrees = rotationDegrees;
        }
    }
}