package frc.robot.subsystems;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.Subsystem;

public class RotateWrist implements Subsystem{
    
    private SparkMax motor;
    private SparkClosedLoopController feedbackController;
    private SparkAbsoluteEncoder targetEncoder; //named by Evan

    public RotateWrist() {
        motor = new SparkMax(-1, MotorType.kBrushless);

        configureMotors();
    }

    private void configureMotors() {
        SparkMaxConfig motorConfig = new SparkMaxConfig();

        motorConfig
            .idleMode(IdleMode.kBrake)
            .inverted(false);
        motorConfig.encoder
            .velocityConversionFactor(-1)
            .positionConversionFactor(-1);
        motorConfig.closedLoop
            .pidf(-1, -1, -1, -1); //possibly more (ended here)

    }
}
