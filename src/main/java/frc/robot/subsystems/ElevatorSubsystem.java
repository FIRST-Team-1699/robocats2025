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

import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase{
    private SparkMax leftMotor, rightMotor;
    private RelativeEncoder targetRelativeEncoder;
    private SparkClosedLoopController feedbackController;
    private ElevatorPositions currentTargetPosition;
    //THE FOLLOWING CODE SHOULD BE REMOVED WHEN ELEVATOR CONSTANTS ARE VISIBLE ON THIS FILE (Only to verify methods accuracy)
    public static final double kELEVATOR_P = -1;
    public static final double kELEVATOR_I = -1;
    public static final double kELEVATOR_D = -1;

    public static final double kTOLEANCE = 1.0;
    //@KEVIN, if you could hint/ tell me the issue for why constants from Constants.java aren't constanting, that would be great :)



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
            .pidf(kELEVATOR_P, kELEVATOR_I, kELEVATOR_D, -1) 
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

    /** Sets the target height of the elevator. 
     * @param ElevatorPositions
     * The taregt position: including state and height.
    */
    public Command elevatorHeight(ElevatorPositions pos) {
        return runOnce(() -> {
            currentTargetPosition = pos;
            feedbackController.setReference(pos.heightCentimeters, SparkBase.ControlType.kVoltage);
        });
    }

    /**Waits until elevator reaches position, then returns waut until Command.
     * @param ElevatorPositions
     * Enum for elevator height options. 
     */
    public Command waitUntilAtSetpoint() {
        return new WaitUntilCommand(() -> {
            return (Math.abs(targetRelativeEncoder.getPosition())- Math.abs(currentTargetPosition.heightCentimeters) >= -kTOLEANCE 
                && targetRelativeEncoder.getPosition()-currentTargetPosition.heightCentimeters <= kTOLEANCE);
        });
    };
    
    //@KEVIN, do I need to use the following code for the isFinished() method in ElevatorLevelCommand,
    //or does WaitUntilAtSetpoint() work for that?

    // public BooleanSupplier getElevatorDone() {
    //         return BooleanSupplier(Math.abs(targetRelativeEncoder.getPosition())- currentTargetPosition.heightCentimeters >= -kTOLEANCE 
    //         && targetRelativeEncoder.getPosition()-currentTargetPosition.heightCentimeters <= kTOLEANCE);
    //     };

    /** Enum for elevator height options. Contains heightCentimeters, which is the target height in centimeters. */
    public enum ElevatorPositions {
        // TODO: Work with Miles to figure out height elevator should go
        L_ONE(45.72), L_TWO(80.01), L_THREE(120.97), L_FOUR(182.88),
        GROUND_INTAKE(-1), SOURCE_INTAKE(-1), COBRA_STANCE(-1), 
        STORED(0);
        public final double heightCentimeters;
        ElevatorPositions (double heightCentimeters) {
            this.heightCentimeters = heightCentimeters;
        }
    }
}
