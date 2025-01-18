package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

public class ElevatorSubsystem implements Subsystem {
    private SparkMax leaderMotor;
    private RelativeEncoder encoder;
    private SparkClosedLoopController controller;
    private SparkMaxConfig motorConfig;
    private ElevatorSetpoints setpoint;
    private final int kTolerance;

    public ElevatorSubsystem() {
        leaderMotor = new SparkMax(-1, MotorType.kBrushless);
        encoder = leaderMotor.getEncoder();
        controller = leaderMotor.getClosedLoopController();

        motorConfig = new SparkMaxConfig();
        motorConfig.encoder
            .positionConversionFactor(1)
            .velocityConversionFactor(1);
        motorConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .outputRange(-1, 1)
            // SLOT 0, RAW POSITION
            .pid(0, 0, 0)
            // SLOT 1, SMARTMOTION
            .pid(0, 0, 0, ClosedLoopSlot.kSlot1);
        motorConfig.smartCurrentLimit(60, 60);

        leaderMotor.configureAsync(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        kTolerance = 1;
    }

    public Command targetHome() {
        return runOnce(() -> {
            controller.setReference(ElevatorSetpoints.HOME.kMotorPosition, ControlType.kPosition);
            setpoint = ElevatorSetpoints.HOME;
        });
    }

    public Command runToHome() {
        return runOnce(() -> setpoint = ElevatorSetpoints.HOME).andThen(run(() -> controller.setReference(
            ElevatorSetpoints.HOME.kMotorPosition, ControlType.kPosition))
            .until(() -> Math.abs(Math.abs(encoder.getPosition()) - Math.abs(encoder.getPosition())) < kTolerance));
    }

    public enum ElevatorSetpoints {
        HOME(0),
        L1(5);

        public final int kMotorPosition;

        private ElevatorSetpoints(int motorPosition) {
            this.kMotorPosition = motorPosition;
        }
    }
}