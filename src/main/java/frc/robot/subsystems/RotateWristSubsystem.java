package frc.robot.subsystems;

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

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.RotateConstants;
import frc.robot.Constants.TiltConstants;

public class RotateWristSubsystem implements Subsystem{
    
    private SparkMax motor;
    private SparkClosedLoopController feedbackController;
    private SparkAbsoluteEncoder targetEncoder; //named by Evan
    private RotatationalWristPosition targetPosition;
    
    /**Constructor for RotateWristSubsystem class */
    public RotateWristSubsystem() {
        // MOTOR CONSTRUCTOR
        motor = new SparkMax(-1, MotorType.kBrushless);
        // DEFAULT TARGET POSITION
        targetPosition = RotatationalWristPosition.HORIZONTAL;
        // CREATES FEEDBACK CONTROLLER 
        feedbackController = motor.getClosedLoopController();
        // CREATES TARGET ENCODER
        targetEncoder = motor.getAbsoluteEncoder();

        configureMotors();
    }
    
    /**Method, configures motor configuration and applies it to motor */
    private void configureMotors() {
        // CONSTRUCTS MOTORCONFIG
        SparkMaxConfig motorConfig = new SparkMaxConfig();
        // MOTORCONFIG
        motorConfig
            .idleMode(IdleMode.kBrake)
            .inverted(false);
        // SETS ENCODER OF MOTORCONFIG
        motorConfig.encoder
            .positionConversionFactor(TiltConstants.kConversionFactor);
        // PPIIIIIIIIDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDFF
        motorConfig.closedLoop
            .pidf(RotateConstants.kP, RotateConstants.kI, RotateConstants.kD, RotateConstants.kFF)
            .outputRange(-1, 1)
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
        // APPLIES MOTORCONFIG TO MOTOR
        motor.configureAsync(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /** Command that sets the rotation and target position of the wrist
     * @param position
     * What rotation you want to move the wrist to
     */
    public Command setRotation(RotatationalWristPosition position) {
        return runOnce(() -> {
            targetPosition = position;
            feedbackController.setReference(position.degrees, SparkBase.ControlType.kPosition);
        });
    }

    /** Command that waits until rotation set point is reached within the tolerance*/
    public Command waitUntilAtSetpoint() {
        return new WaitUntilCommand(() -> {
            return isAtSetpoint();
        });
    }

    public boolean isAtSetpoint() {
        return getError() < RotateConstants.kTolerance;
    }

    private double getError() {
        return Math.abs(Math.abs(targetPosition.degrees) - Math.abs(targetEncoder.getPosition()));
    }

    /**Enum, holds positional data (degrees) */
    public enum RotatationalWristPosition{
        VERICAL(90), HORIZONTAL(0);

        private final double degrees;
        /**Constructor for WristRotPos
         * @param degrees
         * The angle for a positiom
         */
        private RotatationalWristPosition(double degrees) {
            this.degrees = degrees;
        }
    }
}
