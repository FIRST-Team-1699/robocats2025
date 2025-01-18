package frc.robot.subsystems;

import Command;
import SparkMax;
import RelativeEncoder;
import MotorType;
import SparkClosedLoopController;
import ControlType;

public class ElevatorSubsystem implements Subsystem {
    private SparkMax leaderMotor;
    private RelativeEncoder encoder;
    private SparkClosedLoopController controller;

    public ElevatorSubsystem() {
        leaderMotor = new SparkMax(-1, MotorType.kBrushless);
        encoder = leaderMotor.getEncoder();
        controller = leaderMotor.getClosedLoopController();
    }

    public Command targetHome() {
        return runOnce(() -> controller.setReference(0, ControlType.kPosition));
    }
}