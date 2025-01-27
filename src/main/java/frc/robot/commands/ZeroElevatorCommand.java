package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ZeroElevatorCommand extends Command{
    private ElevatorSubsystem elevator;
    // CONSTRUCTOR
    public ZeroElevatorCommand(ElevatorSubsystem elevator) {
        this.elevator = elevator;
        
        addRequirements(elevator);
    }

    public void initalize() {
        // SETS CONDITIONS FOR ZEROING
        elevator.lowerLimit(false);
        // LOWERS UNTIL REACHING LIMIT SWITCH
        elevator.lowerElevator();
    }

    public void execute() {

    }
    
    public boolean isFinished() {
        // FINISHES IF REVERSELIMITSWITCH IS TRIGGERED
        return elevator.isAtBottom();
    }

    public void end(boolean isInterrupted) {
        if (isInterrupted) {
            // RESETS MOTOR/ENCODER
            elevator.resetMotor();
            elevator.resetEncoder();
            // ENABLES SOFTLIMITSWITCH FOR NORMAL RAISING/LOWERING
            elevator.lowerLimit(true);
        }
    }
}
