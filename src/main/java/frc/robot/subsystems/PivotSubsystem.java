package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PivotConstants;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

import com.revrobotics.AbsoluteEncoder;
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
    //TODO: USE ABSOLUTE ENCODER FOR INITALIZATION, RELATIVE OTHERWISE
    private SparkMax followMotor, leadMotor;

    private RelativeEncoder targetRelativeEncoder;
    private AbsoluteEncoder targetAbsoluteEncoder;

    private SparkClosedLoopController feedbackController;
    private PivotPosition currentTargetPosition;

    public PivotSubsystem() {
        // MOTORS
        //Right
        leadMotor = new SparkMax(-1, MotorType.kBrushless);
        //Left
        followMotor = new SparkMax(-1, MotorType.kBrushless);
        // ENCODERS
        targetAbsoluteEncoder = leadMotor.getAbsoluteEncoder();

        targetRelativeEncoder = leadMotor.getEncoder();


        // PID/FEEDBACK CONTROLLER
        feedbackController = leadMotor.getClosedLoopController();
        // SETS TARGET POSITION
        currentTargetPosition = PivotPosition.STORED;
        
        configureMotors();
    }
    /*Cool, motors are being configured her */
    private void configureMotors() {
        // CONFIGURATION CONSTRUCTORS
        SparkMaxConfig followConfig = new SparkMaxConfig();
        SparkMaxConfig leadConfig = new SparkMaxConfig();
        // CONFIGURATIONS
            // RIGHT MOTOR
        leadConfig
            .inverted(true) // TODO: CONFIRM
            .idleMode(IdleMode.kBrake);
        leadConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pidf(PivotConstants.kP, PivotConstants.kI, PivotConstants.kD, -1) 
            .outputRange(-0.5, 0.5);
        leadConfig.encoder
            .positionConversionFactor(PivotConstants.kPOSITIONAL_CONVERSION-PivotConstants.kStoredToZeroDegrees); //TODO: VERIFY THAT THIS IS WHAT WE WANT
        leadConfig.softLimit
            .forwardSoftLimitEnabled(true)
            .reverseSoftLimitEnabled(true)
            .forwardSoftLimit(PivotConstants.kMAX_LIMIT)
            .reverseSoftLimit(PivotConstants.kMIN_LIMIT);

        leadMotor.configureAsync(leadConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
            // LEFT MOROR
        followConfig.apply(followConfig);
        followConfig.follow(leadMotor);
        followConfig.inverted(true); // TODO: CONFIRM
        followMotor.configureAsync(followConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /** Changes hieght/angle of pivot.
     * @param pivotPosition
     * enum that has height value for target position.
     */
    public Command setPosition(PivotPosition pivotPosition) {
        return runOnce(() -> {
            currentTargetPosition = pivotPosition;
            feedbackController.setReference(pivotPosition.degrees, SparkBase.ControlType.kPosition);
        });
    }

    /**gets the direction of Pivot, used for determining order of command groups.
     * @param newPosition
     * The new position to determine direction
     */
    public boolean isPivotRising(PivotPosition newPosition) {
        return (newPosition.degrees-currentTargetPosition.degrees > 0 );
    }

    public Command waitUntilAtSetpoint() {
        return new WaitUntilCommand(() -> {
            return isAtSetpoint();
        });
    }

    public boolean isAtSetpoint() {
        return getError() < PivotConstants.kTOLERANCE;
    }
    
    private double getError() {
        return Math.abs(Math.abs(targetRelativeEncoder.getPosition()) - Math.abs(currentTargetPosition.degrees));
    }

    /**Ensures that motor is set to 0 after triggering bottomLimitSwitch*/
    public Command stopMotorCommand() {
        return runOnce(() -> {
            leadMotor.set(0);
        });
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Actual Pivot Angle", targetAbsoluteEncoder.getPosition());
        SmartDashboard.putNumber("Wanted Pivot Angle", currentTargetPosition.degrees);
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
        double degrees;
        PivotPosition(double degrees) {
            this.degrees = degrees;
        }
    }
}