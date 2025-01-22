package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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

public class ElevatorSubsystem extends SubsystemBase{
    private SparkMax leftMotor, rightMotor;
    private RelativeEncoder leftRelativeEncoder, rightRelativeEncoder;
    private SparkClosedLoopController feedbackController;
    private ElevatorPositions currentTargetPosition;

    /** Constructs an elevator. */
    public ElevatorSubsystem() {
        // MOTOR CONTROLLERS
        leftMotor = new SparkMax(-1, MotorType.kBrushless);
        rightMotor = new SparkMax(-1, MotorType.kBrushless);
    
        // RELATIVE ENCODERS
        leftRelativeEncoder = leftMotor.getEncoder();
        rightRelativeEncoder = rightMotor.getEncoder();

        // PID CONTROLLER
        feedbackController = leftMotor.getClosedLoopController();
        
        // POSITION
        currentTargetPosition = ElevatorPositions.STORED;

        configureMotors();
    }

    /** Sets the configurations for each motor. */
    private void configureMotors() {
        SparkMaxConfig leftConfig = new SparkMaxConfig();
        SparkMaxConfig rightConfig = new SparkMaxConfig();

        leftConfig
            .inverted(true) // TODO: CONFIRM
            .idleMode(IdleMode.kBrake);
        leftConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pidf(-1.0, 0.0, 0.0, 0.0)
            .outputRange(-1.0, 1.0);
        leftConfig.encoder
            .positionConversionFactor(-1)
            .velocityConversionFactor(-1);
        leftMotor.configureAsync(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        rightConfig = new SparkMaxConfig();
        rightConfig.apply(leftConfig);
        rightConfig.follow(leftMotor);
        rightConfig.inverted(true); // TODO: CONFIRM
        rightMotor.configureAsync(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    // TODO: TAKE ENUM AND NOT DOUBLE
    /** Sets the target height of the elevator. 
     * @param heightCentimeters
     * The target height in centimeters.
    */
    public Command changeElevatorHeight(double heightCentimeters) {
        return runOnce(() -> {
            // currentTargetPosition = ELEVATOR POSITION FROM PARAMETER
            feedbackController.setReference(heightCentimeters, SparkBase.ControlType.kVoltage);
            //Verify with kevin
        });
    }

    public Command waitUntilAtSetpoint() {
        return new WaitUntilCommand(() -> true); // TODO: CHANGE CONDITION TO SUBTRACT SETPOINT FROM MEASURED AND SEE IF WITHIN TOLERANCE
    }

    // METHOD TO SET POINT BASED ON HEIGHT

    /** Enum for elevator height options. Contains heightCentimeters, which is the target height in centimeters. */
    public enum ElevatorPositions {
        // TODO: Work with Miles to figure out height elevator should go
        L_ONE(45.72), L_TWO(80.01), L_THREE(120.97), L_FOUR(182.88),
        GROUND_INTAKE(-1), SOURCE_INTAKE(-1), COBRA_STANCE(-1), STORED(0);
        public final double heightCentimeters;
        ElevatorPositions (double heightCentimeters) {
            this.heightCentimeters = heightCentimeters;
        }
    }
}
