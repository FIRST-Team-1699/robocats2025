package frc.robot.subsystems;

import frc.robot.Constants.IntakeConstants;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import java.util.function.BooleanSupplier;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

    private SparkMax motor;
      
    private SparkMaxConfig config;

    private IntakeSpeed currentIntakeSpeed;

    private ShuffleboardTab intakeTab;

    public IntakeSubsystem() {
        motor = new SparkMax(IntakeConstants.kMotorID, MotorType.kBrushless);

        currentIntakeSpeed = IntakeSpeed.STOP;

        configureMotors();

        intakeTab = Shuffleboard.getTab("Intake");
    }

    private void configureMotors() {
        config = new SparkMaxConfig();

        config
            .inverted(IntakeConstants.kInverted) 
            .idleMode(IdleMode.kBrake);
        config.limitSwitch
            .forwardLimitSwitchEnabled(false)
            .reverseLimitSwitchEnabled(false);
        config.openLoopRampRate(.5);
            // .forwardLimitSwitchType(Type.kNormallyOpen);

        motor.configureAsync(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
    /**Prepares the speed of arm for when trigger is pressed
     * @param intakeSpeed
     * The speed of the intake being prepared for
     */
    public Command setWaitingIntake(IntakeSpeed intakeSpeed) {
        return runOnce(() ->this.currentIntakeSpeed = intakeSpeed);
    }
    /**Used to run intake based on speed defined by the ArmState enum and the IntakeSpeed enum inside of it. Will run or stop Intake.
     * @param toReverseIntake
     * Boolean to determine to reverse or run intake
     */
    public Command runIntake() {
        return runOnce(() -> {
            motor.set(currentIntakeSpeed.speed);
        });
    }

    public Command runOutake() {
        return runOnce(() -> {
            motor.set(-currentIntakeSpeed.speed);
        });
    }

    public boolean isRunning() {
        return motor.get() != 0;
    }

    public Command stopMotorCommand() {
        return runOnce(() -> motor.set(0));
    }

    public Command runIntake(double percentage) {
        return runOnce(() -> motor.set(percentage));
    }

    public boolean hasPiece() {
        return motor.getReverseLimitSwitch().isPressed();
    }

    // public Command setRaw(double speed) {
    //     return runOnce(()-> {
    //         motor.set(speed);
    //     });
    // }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Intake is at hard limit", hasPiece());
        SmartDashboard.putNumber("Wanted intake speed", currentIntakeSpeed.speed);
        SmartDashboard.putNumber("Current intake speed", motor.get());

    // intakeTab.add("Speed", motor.get());
    // intakeTab.add("Is Running", isRunning());
    }

    public enum IntakeSpeed {
        CORAL(.1), ALGAE(.1), DESCORE_ALGAE(.1), //TODO: Change values to verify differing intake speeds
        STOP(0);
        double speed;
        IntakeSpeed(double speed) {
            this.speed = speed;
        }
    }
}