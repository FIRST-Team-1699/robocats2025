package frc.robot.subsystems;



import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem implements Subsystem {
 
    private SparkMax motor;

    public IntakeSubsystem() {
        motor = new SparkMax(-1, MotorType.kBrushless);

        configureMotors();
    }

    private void configureMotors() {
        SparkMaxConfig config = new SparkMaxConfig();

        config.idleMode(IdleMode.kBrake);
        config.inverted(false); // TODO: CHANGE LATER

        motor.configureAsync(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public Command intake() {
        return runOnce(() -> motor.set(IntakeConstants.kIntakeSpeed));
    }

    public Command outtake() {
        return runOnce(() -> motor.set(IntakeConstants.kOuttakeSpeed));
    }

    public Command stop() {
        return runOnce(() -> motor.set(0));
    }
    //TODO: make things enums
}