package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.AlternateEncoderConfig.Type;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
    private SparkMaxConfig motorConfig;

    private SparkMax upperMotor;
    private SparkMax lowerMotor;

    private RelativeEncoder upperEncoder; 
    private RelativeEncoder lowerEncoder;

    public ElevatorSubsystem () {
        motorConfig.idleMode(IdleMode.kBrake);

        upperMotor = new SparkMax(-1, MotorType.kBrushless);
        upperMotor.configureAsync(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        lowerMotor = new SparkMax(-1, MotorType.kBrushless);
        lowerMotor.configureAsync(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // upperEncoder = upperElevatorMotor.getAlternateEncoder(Type.kQuadrature, 8192);
        // lowerElevatorEncoder = lowerElevatorMotor.getAlternateEncoder(Type.kQuadrature, 8192);
    }

    public Command pickLevel(int level) {
        return runOnce(() -> {
            //Higeht = centemeters
            switch (level) {
                case 0 :
                    upperMotor.set(0);
                    lowerMotor.set(0);
                case 1 :
                    upperMotor.set(46);
                    lowerMotor.set(46);
                case 2 :
                    upperMotor.set(87.63); //Hieght is set to this       
                    lowerMotor.set(87.63);
                case 3 :
                    upperMotor.set(121);   
                    lowerMotor.set(121);   
                case 4 :
                    upperMotor.set(183);    
                    lowerMotor.set(183);  
            }
        });
    }

}
