import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PivotSubsystem extends SubsystemBase{
    SparkMax leftMotor, rightMotor;
    RelativeEncoder encoder;
    SparkMaxConfig config, follow;
    SparkClosedLoopController feedbackController;
    public PivotSubsystem() {
        leftMotor = new SparkMax(-1, MotorType.kBrushless);
        rightMotor = new SparkMax(-1, MotorType.kBrushless);

        encoder = rightMotor.getEncoder();

        feedbackController = rightMotor.getClosedLoopController();

        configureMotors();
    }
    /*Cool, motors are being configured her */
    private void configureMotors() {
        config
            .inverted(true);
            .idleMode(IdleMode.kBrake);
        follow
            .follow(rightMotor);
        leftMotor.
    }

    public Command levelOnePivot() {
        return runOnce(() -> {
            feedbackController.setReference(46.0, SparkBase.ControlType.kVoltage); //TODO: get manufacturing + general additional data to be cool :)
        }
        );
    }
    public enum PivotAngles{
        GROUND(-1), SOURCE(-1), REST(-1), COBRA_STANCE(-1),
        L_ONE(-1), L_TWO(-1), L_THREE(-1), L_FOUR(-1)
        private final double pivotHeight;
        public PivotAngles(double pivotHeight) {
            this.pivotHeight = pivotHeight;
        }
    }
}