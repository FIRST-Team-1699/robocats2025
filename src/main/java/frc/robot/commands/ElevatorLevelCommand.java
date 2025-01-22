package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorPositions;

public class ElevatorLevelCommand extends Command {
    private ElevatorSubsystem elevatorSubsystem;
    private ElevatorPositions heightPosition;

    public ElevatorLevelCommand(ElevatorSubsystem elevatorSubsystem, ElevatorPositions heightPosition) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.heightPosition = heightPosition;

        addRequirements(elevatorSubsystem);
    }

    public void initalize() {
        elevatorSubsystem.elevatorHeight(heightPosition);
    }

    public void execute() {}

    public boolean isFinished() {
        return (Boolean.parseBoolean(String.valueOf(new ElevatorSubsystem().waitUntilAtSetpoint())));
        //Figure out parameters to put here. DONE :)
    }

    public void end(boolean isInterrupted) {}
}
