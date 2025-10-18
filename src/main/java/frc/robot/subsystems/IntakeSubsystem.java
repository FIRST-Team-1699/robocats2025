package frc.robot.subsystems;

import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.TiltWristSubsystem.TiltPosition;
import frc.robot.utils.BeamBreak;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import java.util.function.BooleanSupplier;

import com.revrobotics.spark.SparkMax;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

    private TalonFX motor;

    private MotorOutputConfigs motorConfigs;

    private IntakeSpeed currentIntakeSpeed;

    private ShuffleboardTab intakeTab;

    public IntakeSubsystem() {
        motor = new TalonFX(IntakeConstants.kMotorID);

        currentIntakeSpeed = IntakeSpeed.STOP;

        configureMotors();

        intakeTab = Shuffleboard.getTab("Intake");
    }

    private void configureMotors() {
        // TODO: ABSTRACT
        motorConfigs = new MotorOutputConfigs();

        motorConfigs.Inverted = InvertedValue.Clockwise_Positive;
        motorConfigs.PeakForwardDutyCycle = IntakeConstants.kForwardLimit;
        motorConfigs.PeakReverseDutyCycle = IntakeConstants.kReverseLimit;
        motorConfigs.NeutralMode = IntakeConstants.kIdle;


        motor.getConfigurator().apply(motorConfigs);
        motor.setControl(new VoltageOut(0));
    }
    /**Prepares the speed of arm for when trigger is pressed
     * @param intakeSpeed
     * The speed of the intake being prepared for
     */
    public Command setWaitingIntake(IntakeSpeed intakeSpeed) {
        return runOnce(() ->this.currentIntakeSpeed = intakeSpeed);
    }
    // /**Used to run intake based on speed defined by the ArmState enum and the IntakeSpeed enum inside of it. Will run or stop Intake.
    //  * @param toReverseIntake
    //  * Boolean to determine to reverse or run intake
    //  */
    // public Command outtake() {
    //     return runOnce(() -> motor.set(IntakeSpeed.OUTTAKE.speed));
    // }

    public boolean isRunning() {
        return motor.get() != 0 && motor.get() != .05;
    }

    public Command stopMotorCommand() {
        // return runOnce(() -> motor.set(0.05));
        return runOnce(() -> motor.set(IntakeSpeed.STOP.speed));
    }

    public Command runCoralIntake() {
        return runOnce(() -> motor.set(IntakeSpeed.OUTTAKE.speed));
    }

    public Command runCoralGroundIntake() {
        return runOnce(() -> motor.set(IntakeSpeed.OUTTAKE.speed));
    }

    public Command runALgaeIntake() {
        return runOnce(() -> motor.set(IntakeSpeed.INTAKE.speed));
    }

    public Command runCoralOuttake() {
        return runOnce(() -> motor.set(IntakeSpeed.INTAKE.speed));
    }

    public Command runALgaeOuttake() {
        return runOnce(() -> motor.set(IntakeSpeed.OUTTAKE.speed));
    }

    // public boolean flipSensorActive() {
    //     return motor.getForwardLimitSwitch().isPressed();
    // }

    // public Command setRaw(double speed) {
    //     return runOnce(()-> {
    //         motor.set(speed);
    //     });
    // }

    @Override
    public void periodic() {
        // SmartDashboard.putBoolean("Intake is at hard limit", hasPiece());
        SmartDashboard.putNumber("Wanted intake speed", currentIntakeSpeed.speed);
        SmartDashboard.putNumber("Current intake speed", motor.get());
        SmartDashboard.putBoolean("BeamBreak Triggered", BeamBreak.hasCoral().getAsBoolean());
        SmartDashboard.putNumber("BeamBreak distance", BeamBreak.getDistance());


    // intakeTab.add("Speed", motor.get());
    // intakeTab.add("Is Running", isRunning());
        if(BeamBreak.hasCoral().getAsBoolean()) {
            motor.set(IntakeSpeed.STOP.speed);
        }
    }

    public enum IntakeSpeed {
        // CORAL(.1), ALGAE(.1), DESCORE_ALGAE(.1), //TODO: Change values to verify differing intake speeds
        // STOP(0);
        INTAKE(-70), //PLACES ON FRONT FOR CORAL
        OUTTAKE(60), //INTAKES CORAL
        STOP(0);
        double speed;
        IntakeSpeed(double speed) {
            this.speed = speed;
        }
    }
}