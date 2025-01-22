package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorPositions;

public class ElevatorLevelCommand extends Command {
    private ElevatorSubsystem elevator;
    private ElevatorPositions heightPosition;

    public ElevatorLevelCommand(ElevatorSubsystem elevator, ElevatorPositions heightPosition) {
        this.elevator = elevator;
        this.heightPosition = heightPosition;

        addRequirements(elevator);
    }

    public void initalize() {
        elevator.elevatorHeight(heightPosition);
    }

    public void execute() {}

    public boolean isFinished() {
        return (elevator.hasReachedPoint().getAsBoolean());
        //Figure out parameters to put here. DONE :)
    }

    public void end(boolean isInterrupted) {}
}
