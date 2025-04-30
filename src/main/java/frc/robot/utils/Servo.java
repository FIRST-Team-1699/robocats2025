package frc.robot.utils;

import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Singleton for the servo which triggers the hooks to hold our climber down. */
public class Servo extends SubsystemBase {
    private static Servo instance;

    public static Servo getInstance() {
        if(instance == null) {
            instance = new Servo();
        }
        return instance;
    }

    private Relay servo;

    private Servo() {
        servo = new Relay(0);
        servo.setSafetyEnabled(false);
        servo.setDirection(Relay.Direction.kBoth);
        servo.set(Relay.Value.kOff);
    }

    public Command activateServo() {
        return runOnce(() -> servo.set(Relay.Value.kOn));
    }

    public Command deactivateServo() {
        return runOnce(() -> servo.set(Relay.Value.kOff));
    }

    public void enableServo() {
        servo.set(Relay.Value.kOn);
    }

    public void disableServo() {
        servo.set(Relay.Value.kOff);
    }
}
