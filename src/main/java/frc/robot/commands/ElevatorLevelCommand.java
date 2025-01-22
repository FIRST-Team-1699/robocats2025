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

    // TODO: MAKE THIS SEND ENUM RATHER THAN TAKING THE HEIGHT OUT OF IT FIRST
    public void initalize() {
        elevatorSubsystem.changeElevatorHeight(heightPosition.heightCentimeters);
    }

    public void execute() {}

    public boolean isFinished() {
        return false;
        //Figure out parameters to put here
    }

    public void end(boolean isInterrupted) {}
}
