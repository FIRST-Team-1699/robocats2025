package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class ElevatorSubsystem extends SubsystemBase{
    private SparkMax leftMotor, rightMotor;
    private RelativeEncoder leftRelativeEncoder, rightRelativeEncoder;
    private SparkMaxConfig idleConfig;
    private PIDController pidController;

    public ElevatorSubsystem() {
        idleConfig.idleMode(IdleMode.kBrake);
        idleConfig.idleMode(IdleMode.kBrake);

        leftMotor = new SparkMax(-1, MotorType.kBrushless);
        leftMotor.configureAsync(idleConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        // leftRelativeEncoder = leftMotor.getAlternateEncoder(Type.kQuadrature, 8192);

        rightMotor = new SparkMax(-1, MotorType.kBrushless);
        rightMotor.configureAsync(idleConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        // rightRelativeEncoder = leftMotor.getAlternateEncoder(Type.kQuadrature, 8192);

        PIDController pidController = new PIDController(-1,-1,-1);
    }

    public Command changeEleavtorHeight(double heightCentimeters) {
        return runOnce(() -> {
            leftMotor.set(pidController.calculate(leftMotor.get(), heightCentimeters));
            rightMotor.set(pidController.calculate(rightMotor.get(), heightCentimeters));
            //Verify with kevin
        });
    }

    public Command stopElevator(double currentHeightCentimeters) {
        return run(() -> {
            leftMotor.set(pidController.calculate(leftMotor.get(), currentHeightCentimeters));
            rightMotor.set(pidController.calculate(rightMotor.get(), currentHeightCentimeters));
        });
    }

    public enum HeightPosistions {
        //Work with Miles to figure out height elevator should go
        LONE(45.72), LTWO(80.01), LTHREE(120.97), LFOUR(182.88),
        GROUNDINTAKE(-1), SOURCEINTAKE(-1),
        COBRASTANCE(-1);
        public final double heightCentimeters;
        HeightPosistions (double heightCentimeters) {
            this.heightCentimeters = heightCentimeters;
        }
    }
}
