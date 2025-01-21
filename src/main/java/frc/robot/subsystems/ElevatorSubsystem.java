package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
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
    private SparkMaxConfig followerConfig;
    private SparkClosedLoopController pid;

    public ElevatorSubsystem() {
        idleConfig.idleMode(IdleMode.kBrake);
        followerConfig.follow(leftMotor);

        //Note that leftMotor == leadMotor
        leftMotor = new SparkMax(-1, MotorType.kBrushless);
        leftMotor.configureAsync(idleConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        leftRelativeEncoder = leftMotor.getAlternateEncoder();
        // leftRelativeEncoder = leftMotor.getAlternateEncoder(Type.kQuadrature, 8192);

        rightMotor = new SparkMax(-1, MotorType.kBrushless);
        rightMotor.configureAsync(idleConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightMotor.configureAsync(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightRelativeEncoder = leftMotor.getAlternateEncoder();

        pid = leftMotor.getClosedLoopController();
    }

    public Command changeEleavtorHeight(double heightCentimeters) {
        return runOnce(() -> {
            pid.setReference(heightCentimeters, SparkBase.ControlType.kVoltage);
            //Verify with kevin
        });
    }

    public Command stopElevator(HeightPositions heightPosition) {
        return run(() -> {
            pid.setReference(heightPosition.heightCentimeters, SparkBase.ControlType.kVoltage); 
        });
    }

    public enum HeightPositions {
        //Work with Miles to figure out height elevator should go
        LONE(45.72), LTWO(80.01), LTHREE(120.97), LFOUR(182.88),
        GROUNDINTAKE(-1), SOURCEINTAKE(-1),
        COBRASTANCE(-1);
        public final double heightCentimeters;
        HeightPositions (double heightCentimeters) {
            this.heightCentimeters = heightCentimeters;
        }
    }
}
