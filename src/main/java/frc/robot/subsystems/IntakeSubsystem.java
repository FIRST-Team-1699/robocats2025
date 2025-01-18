package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class IntakeSubsystem extends SubsystemBase{
    private SparkMax motor;
    private SparkMaxConfig motorConfig;

    public IntakeSubsystem() {
        motor = new SparkMax(-1, MotorType.kBrushless);
        
        motorConfig.idleMode(IdleMode.kBrake);
        motor.configureAsync(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public Command intake() {
        return runOnce(() -> {
            motor.set(0.5);
        });
    }

    public Command outtake() {
        return runOnce(() -> {
            motor.set(-0.5);
        });
    }
    
    public Command stop() {
        return runOnce(() -> {
            motor.set(0);
        });
    }
}
