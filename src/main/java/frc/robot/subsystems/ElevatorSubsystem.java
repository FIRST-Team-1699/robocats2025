package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase{
    private SparkMax leftMotor, rightMotor;
    private RelativeEncoder targetRelativeEncoder;
    private SparkClosedLoopController feedbackController;
    private SparkMaxConfig rightConfig, leftConfig;
    private ElevatorPositions currentTargetPosition;
    private SparkLimitSwitch bottomLimitSwitch;

    private static double elevatorError;


    /** Constructs an elevator. */
    public ElevatorSubsystem() {
        // MOTOR CONTROLLERS
        leftMotor = new SparkMax(-1, MotorType.kBrushless);
        rightMotor = new SparkMax(-1, MotorType.kBrushless);
    
        // RELATIVE ENCODERS
        targetRelativeEncoder = leftMotor.getEncoder();

        // PID CONTROLLER
        feedbackController = leftMotor.getClosedLoopController();
        
        // POSITION
        currentTargetPosition = ElevatorPositions.STORED;

        bottomLimitSwitch = leftMotor.getReverseLimitSwitch();

        configureMotors();
    }

    /** Sets the configurations for each motor. */
    private void configureMotors() {
        // CONFIGURATIONS
        leftConfig = new SparkMaxConfig();
        rightConfig = new SparkMaxConfig();

            // LEFT MOTOR
        leftConfig
            .inverted(true) // TODO: CONFIRM
            .idleMode(IdleMode.kBrake);
        leftConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pidf(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD, -1) 
            .outputRange(-1.0, 1.0);
        leftConfig.softLimit
            // TODO: Assuming in CM, possibly fix later (One CM heigher than L4, within tolerance)
            .forwardSoftLimitEnabled(true)
            .reverseSoftLimitEnabled(true)

            .forwardSoftLimit(ElevatorConstants.kMAX_LIMIT)
            .reverseSoftLimit(ElevatorConstants.kMIN_LIMIT);
        leftConfig.limitSwitch
            .forwardLimitSwitchEnabled(true) // TODO: Check for changes w/ design (wether or not we will be using limit switches)
            .reverseLimitSwitchEnabled(true)

            .reverseLimitSwitchType(LimitSwitchConfig.Type.kNormallyOpen)
            .setSparkMaxDataPortConfig();
        leftConfig.encoder
            .positionConversionFactor(-1)
            .velocityConversionFactor(-1);
            // APPLIES LEFT CONFIG TO RIGHT MOTOR
        leftMotor.configureAsync(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
            // RIGHT MOTOR
        rightConfig.apply(leftConfig);
        rightConfig.follow(leftMotor);
        rightConfig.inverted(true); // TODO: CONFIRM,
            // APPLIES RIGHT CONFIG TO RIGHT MOTOR
        rightMotor.configureAsync(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
    // COMMAND FACTORIES TO REACH ENUM HEIGHT

    /** Sets the target height of the elevator. 
     * @param ElevatorPositions
     * The taregt position: including state and height.
    */
    public Command setHeight(ElevatorPositions pos) {
        return runOnce(() -> {
            // CHANGES CURRENT TARGET TO POS
            currentTargetPosition = pos;
            // SETS FEEDBACKCONTROLLER TO POS
            feedbackController.setReference(pos.heightCentimeters, SparkBase.ControlType.kPosition);
        });
    }

    /**Waits until elevator reaches position within Tolerance.
     * @param ElevatorPositions
     * Enum for elevator height options. 
     */
    public Command waitUntilAtSetpoint() {
        return new WaitUntilCommand(() -> {
            // SETS ELEVATOR ERROR
            elevatorError = Math.abs(Math.abs(targetRelativeEncoder.getPosition())- Math.abs(currentTargetPosition.heightCentimeters));
            // TEST FOR IF ELEVATORERROR IS IN TOLERANCE OF TARGETPOSITION
            return (elevatorError < ElevatorConstants.kTOLERANCE);
        });
    }
    // COMMAND FACTORIES TO ZERO ELEVATOR

    /**Runs a WaitUntilCommand, waits until elevator reaches bottom */
    public Command lowerElevator() {
        return new WaitUntilCommand(() -> {
            leftMotor.set(-0.2);
            return isAtBottom();
        });
    }
    /**Resets encoder to 0 after zeroing */
    public Command resetEncoder() {
        return runOnce(() -> {
            targetRelativeEncoder.setPosition(0);
        });
    }
    /**Ensures that motor is set to 0 after triggering bottomLimitSwitch*/
    public Command resetMotor() {
        return runOnce(() -> {
            rightMotor.set(0);
        });
    }

    /**Enables or disables reverseSoftLimmit
     * @param enabled
     * considers wether (based on boolean data) lowerLimit is to be disable or enabled
     */
    public Command lowerLimit(boolean enabled) {
        return runOnce(()-> {
            leftConfig.softLimit.reverseSoftLimitEnabled(enabled);
            leftMotor.configureAsync(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        });
    }

    /**Returns if bottomLimitSwitch has been reached */
    public boolean isAtBottom() {
        return bottomLimitSwitch.isPressed();
    }
    
    /** Enum for elevator height options. Contains heightCentimeters, which is the target height in centimeters. */
    public enum ElevatorPositions {
        // ENUMS FOR POSITIONS
            // LEVEL POSITIONS
        L_ONE(45.72), L_TWO(80.01), L_THREE(120.97), L_FOUR(182.88),
            // NON-LEVEL POSTIONS
        GROUND_INTAKE(-1), SOURCE_INTAKE(-1), COBRA_STANCE(-1), STORED(0);
            // CONSTRUCTOR FOR ENUM'S HEIGHT (CM)
        public final double heightCentimeters;
        /**Constrcutor for height for ElevatorPositions (Enum for Elevator poses)
         * @param heightCentimeters
         * verticle movement in centimeters
        */
        ElevatorPositions (double heightCentimeters) {
            this.heightCentimeters = heightCentimeters;
        }
    }
}
