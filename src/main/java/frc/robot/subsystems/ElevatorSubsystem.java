package frc.robot.subsystems;

import frc.robot.Constants.ElevatorConstants;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
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

public class ElevatorSubsystem extends SubsystemBase {
    // MOTORS
    private SparkMax leadMotor, followMotor;
    // ENCODER
    private RelativeEncoder encoder;
    // FEEDBACK CONTROLLER
    private SparkClosedLoopController feedbackController;
    // CONFIGS
    SparkMaxConfig leadConfig, followConfig;
    // CURRENT POSITION
    public ElevatorPosition currentTargetPosition;

    /** Constructs an elevator. */
    public ElevatorSubsystem() {
        // MOTOR CONTROLLERS
        leadMotor = new SparkMax(ElevatorConstants.kLeaderID, MotorType.kBrushless);
        followMotor = new SparkMax(ElevatorConstants.kFollowerID, MotorType.kBrushless);
        // ENCODER
        encoder = leadMotor.getEncoder();
        // PID CONTROLLER
        feedbackController = leadMotor.getClosedLoopController();
        // POSITION
        currentTargetPosition = ElevatorPosition.STORED;
        // CONFIGS
        leadConfig = new SparkMaxConfig();
        followConfig = new SparkMaxConfig();
        // CONFIGURE MOTORS
        configureMotors();
    }

    /** Sets the configurations for each motor. */
    private void configureMotors() {
        // LEADER CONFIG
        leadConfig
            .inverted(ElevatorConstants.kInverted)
            .idleMode(ElevatorConstants.kIdleMode)
            .smartCurrentLimit(ElevatorConstants.kStallLimit, ElevatorConstants.kFreeLimit);
        leadConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pidf(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD, ElevatorConstants.kFF, ClosedLoopSlot.kSlot0)
            .pidf(ElevatorConstants.kMAXMotionP, ElevatorConstants.kMAXMotionI, ElevatorConstants.kMAXMotionD, ElevatorConstants.kMAXMotionFF, ClosedLoopSlot.kSlot1) 
            .outputRange(ElevatorConstants.kMinimumOutputLimit, ElevatorConstants.kMaximumOutputLimit, ClosedLoopSlot.kSlot0)
            .outputRange(ElevatorConstants.kMinimumOutputLimit, ElevatorConstants.kMaximumOutputLimit, ClosedLoopSlot.kSlot1)
        .maxMotion
            .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal, ClosedLoopSlot.kSlot1)
            .maxAcceleration(ElevatorConstants.kMAXMotionMaxAcceleration, ClosedLoopSlot.kSlot1)
            .maxVelocity(ElevatorConstants.kMAXMotionMaxVelocity, ClosedLoopSlot.kSlot1)
            .allowedClosedLoopError(ElevatorConstants.kMAXMotionAllowedError, ClosedLoopSlot.kSlot1);
        leadConfig.softLimit
            .forwardSoftLimit(ElevatorConstants.kMaximumRotationLimit)
            .forwardSoftLimitEnabled(true)
            .reverseSoftLimit(ElevatorConstants.kMinimumRotationLimit)
            .reverseSoftLimitEnabled(true);
        leadMotor.configureAsync(leadConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        // FOLLOWER CONFIG
        followConfig.apply(leadConfig);
        followConfig.follow(leadMotor, ElevatorConstants.kFollowerInverted);
        followMotor.configureAsync(followConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    // COMMAND FACTORIES TO REACH ENUM HEIGHT
    public Command setRaw(double percent) {
        return runOnce(() -> {
            leadMotor.set(percent);
        });
    }

    /** Sets the target height of the elevator. 
     * @param ElevatorPosition
     * The taregt position: including state and height.
    */
    public Command setPosition(ElevatorPosition position) {
        return runOnce(() -> {
            // CHANGES CURRENT TARGET TO POS
            currentTargetPosition = position;
            // SETS FEEDBACKCONTROLLER TO POS
            feedbackController.setReference(position.rotations, SparkBase.ControlType.kPosition, ClosedLoopSlot.kSlot0);
        });
    }

    /** Sets the target height of the elevator using trapezoidal profiling. 
     * @param ElevatorPosition
     * The taregt position: including state and height.
    */
    public Command setPositionSmartMotion(ElevatorPosition position) {
        return runOnce(() -> {
            // CHANGES CURRENT TARGET TO POS
            currentTargetPosition = position;
            // SETS FEEDBACKCONTROLLER TO POS
            feedbackController.setReference(position.rotations, SparkBase.ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot1);
        });
    }

    public Command moveToSafePosition() {
        return setPosition(ElevatorPosition.SAFE_POSITION).onlyIf(() -> !currentTargetPosition.shouldPivotMoveFromHere());
    }

    /**Waits until elevator reaches position within Tolerance.
     * @param ElevatorPosition
     * Enum for elevator height options. 
     */
    public Command waitUntilAtSetpoint() {
        return new WaitUntilCommand(() -> {
            // TEST FOR IF ELEVATORERROR IS IN TOLERANCE OF TARGETPOSITION
            return isAtSetpoint();
        });
    }
    
    public boolean isAtSetpoint() {
        return (getElevatorError() < ElevatorConstants.kTolerance);
    }

    private double getElevatorError() {
        return Math.abs(Math.abs(encoder.getPosition()) - Math.abs(currentTargetPosition.rotations));
    }

    /**Resets encoder to 0*/
    public Command resetEncoder() {
        return runOnce(() -> {
                encoder.setPosition(0);
            });
    }
    /**Ensures that motor is set to 0 after triggering bottomLimitSwitch*/
    public Command stopMotorCommand() {
        return runOnce(() -> {
            leadMotor.set(0);
        });
    }

    /** Stops the motor manually, ignoring all commands. */
    public void stopMotorManual() {
        leadMotor.set(0);
    }

    public Command printPosition() {
        return runOnce(() -> System.out.println(encoder.getPosition()));
    }

    public void setIdleMode(IdleMode idleMode) {
        leadConfig.idleMode(idleMode);
        leadMotor.configureAsync(leadConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        followConfig.idleMode(idleMode);
        followMotor.configureAsync(followConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Elevator Height", encoder.getPosition());
        SmartDashboard.putNumber("Target Elevator Height", currentTargetPosition.getRotations());
        SmartDashboard.putBoolean("Elevator at Setpoint", isAtSetpoint());
        SmartDashboard.putBoolean("Elevator at safe return point", currentTargetPosition.shouldPivotMoveFromHere());
    }
    
    /** Enum for elevator height options. Contains heightCentimeters, which is the target height in centimeters. */
    public enum ElevatorPosition {
        // ENUMS FOR POSITIONS
        STORED(0), PRIME(0), COBRA_STANCE(-1), SAFE_POSITION(7),
        
        CLIMB(10),

        ALGAE_INTAKE(-1), ALGAE_DESCORE_L_TWO(4), ALGAE_DESCORE_L_THREE(18),
      
        GROUND_INTAKE(7), CORAL_STATION_INTAKE(0),

        L_ONE(0), L_TWO(6), L_THREE(9), L_FOUR(47);

        private double rotations;
        /**Constrcutor for height for ElevatorPositions (Enum for Elevator poses)
        * @param rotations
        * verticle movement in centimeters
        */
        ElevatorPosition(double rotations) {
            this.rotations = rotations;
        }

        public double getRotations() {
            return this.rotations;
        }

        public boolean shouldPivotMoveFromHere() {
            return this.rotations <= ElevatorConstants.kUnsafePosition;
        }
    }
}