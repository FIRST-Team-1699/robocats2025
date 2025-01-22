package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorPositions;

public class ElevatorLevelCommand extends Command {
    private ElevatorSubsystem elevator;
    private ElevatorPositions heightPosition;

    /**The constructor for the ElevatorLevelCommand class.
     * @param ElevatorSubsystem 
     * contains crucial methods for controlling elevator's positioning.
     * @param ElevatorPositions
     * Enum for determining target position.
     */
    public ElevatorLevelCommand(ElevatorSubsystem elevator, ElevatorPositions heightPosition) {
        // CONSTRUCTORS
        this.elevator = elevator;
        this.heightPosition = heightPosition;

        addRequirements(elevator);
    }
    // STANDARDIZED COMMANDS
    public void initalize() {
        //CHANGES ELEVATOR HEIGHT TO SPECIFIED POSITION
        elevator.elevatorHeight(heightPosition);
    }

    public void execute() {}

    public boolean isFinished() {
        //ENDS COMMAND WHEN IN TOLANCE TO TARGET POSITION
        return (elevator.hasReachedPoint().getAsBoolean());
    }

    public void end(boolean isInterrupted) {}
}
