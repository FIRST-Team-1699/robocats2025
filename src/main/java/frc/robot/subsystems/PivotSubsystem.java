package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PivotConstants;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class PivotSubsystem implements Subsystem {
    private SparkMax leftMotor, rightMotor;
    private RelativeEncoder targetEncoder;
    private SparkClosedLoopController feedbackController;
    private PivotPos targetPos;

    private double pivotError;


    public PivotSubsystem() {
        // MOTORS
        rightMotor = new SparkMax(-1, MotorType.kBrushless);
        leftMotor = new SparkMax(-1, MotorType.kBrushless);
        // ENCODER
        targetEncoder = rightMotor.getEncoder();
        // PID/FEEDBACK CONTROLLER
        feedbackController = rightMotor.getClosedLoopController();
        // SETS TARGET POSITION
        targetPos = PivotPos.BASE;
        
        configureMotors();
    }
    /*Cool, motors are being configured her */
    private void configureMotors() {
        // CONFIGURATION CONSTRUCTORS
        SparkMaxConfig leftConfig = new SparkMaxConfig();
        SparkMaxConfig rightConfig = new SparkMaxConfig();
        // CONFIGURATIONS
            // RIGHT MOTOR
        rightConfig
            .inverted(true) // TODO: CONFIRM
            .idleMode(IdleMode.kBrake);
        rightConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pidf(PivotConstants.kP, PivotConstants.kI, PivotConstants.kD, -1) 
            .outputRange(-1.0, 1.0);
        rightConfig.encoder
            .positionConversionFactor(-1)
            .velocityConversionFactor(-1);
        rightMotor.configureAsync(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
            // LEFT MOROR
        leftConfig.apply(leftConfig);
        leftConfig.follow(rightMotor);
        leftConfig.inverted(true); // TODO: CONFIRM
        leftMotor.configureAsync(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
    /** Changes hieght/angle of pivot.
     * @param pivot
     * enum that has height value for target position.
     */
    public Command setPivot(PivotPos pivot) {
        return runOnce(() -> {
            feedbackController.setReference(pivot.pivotHeight, SparkBase.ControlType.kPosition); //TODO: get manufacturing + general additional data to be cool :)
        }
        );
    }

    public Command waitUntilAtSetpoint() {
        return new WaitUntilCommand(() -> {
            pivotError = Math.abs(targetEncoder.getPosition() - targetPos.pivotHeight);
            return pivotError < PivotConstants.kTOLERENCE;
        });
    }
    // TODO: MAKE A METHOD TO RETURN IF PIVOT HAS REACHED HEIGHT
    /**Enum, holds position of pivot.
     * @param pivotHeight
     * Height Pivot must reach to get to state.
     */
    public enum PivotPos{
        GROUND(-1), SOURCE(-1), BASE(-1), COBRA_STANCE(-1),
        L_ONE(-1), L_TWO(-1), L_THREE(-1), L_FOUR(-1);
        private final double pivotHeight;
        PivotPos(double pivotHeight) {
            this.pivotHeight = pivotHeight;
        }
    }
}