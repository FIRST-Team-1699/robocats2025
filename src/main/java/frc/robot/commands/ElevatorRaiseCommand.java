package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class ElevatorRaiseCommand extends Command {
    private ElevatorSubsystem elevatorSubsystem;

    public ElevatorRaiseCommand(ElevatorSubsystem elevatorSubsystem) {
        this.elevatorSubsystem = elevatorSubsystem;
        addRequirements(elevatorSubsystem);
    }

    public void initalize() {
        
    }

    public void execute() {

    }

    public boolean isFinished() {
        return false;
        //Will probably figure out later
    }

    public void end (boolean isInterrupted) {
        
    }
}
