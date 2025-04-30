package frc.robot.subsystems;

import frc.robot.Constants.IntakeConstants;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

    private SparkMax motor;
      
    private SparkMaxConfig config;

    public IntakeSubsystem() {
        motor = new SparkMax(IntakeConstants.kMotorID, MotorType.kBrushless);

        configureMotors();
    }

    private void configureMotors() {
        config = new SparkMaxConfig();

        config
            .inverted(IntakeConstants.kInverted) 
            .idleMode(IdleMode.kBrake);
        config.limitSwitch
            .forwardLimitSwitchEnabled(false)
            .reverseLimitSwitchEnabled(false);
        config.openLoopRampRate(.1);

        motor.configureAsync(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public boolean isRunning() {
        return motor.get() != 0 && motor.get() != .05;
    }

    public Command stopMotorCommand() {
        return runOnce(() -> motor.set(0.05));
    }

    public Command setIntakeSpeed(double percentage) {
        return runOnce(() -> motor.set(percentage));
    }

    public boolean hasPiece() {
        return motor.getReverseLimitSwitch().isPressed();
    }

    public boolean flipSensorActive() {
        return motor.getForwardLimitSwitch().isPressed();
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Intake is at hard limit", hasPiece());
        SmartDashboard.putNumber("Current intake speed", motor.get());
        SmartDashboard.putBoolean("Flip Sensor Triggered", flipSensorActive());
    }
}