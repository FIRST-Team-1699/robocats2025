package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.Command;

public class ZeroElevator extends ElevatorSubsystem{
    // @KEVIN, Probably too mcuh abstraction, but I think this couls be better (and am excited for you to yell at me and tell me why Im wrong)
    private final ElevatorSubsystem elevator;

    public ZeroElevator(ElevatorSubsystem elevator) {
        this.elevator = elevator;
    }

    /**Runs a Command Group to zero elevator */
    public Command initalizeZero() {
        return runOnce(() -> {
            // SETS CONDITIONS FOR ZEROING
            elevator.toggleLowerLimit(false);
            // LOWERS UNTIL REACHING LIMIT SWITCH
            elevator.lowerElevator();
        });
    }
    /**Runs a Command Group in prepration of elevator completeing zero */
    public Command finishZero() {
        return runOnce(() -> {
            // RESETS MOTOR/ENCODER
            elevator.stopMotorCommand();
            elevator.resetEncoder();
            // ENABLES SOFTLIMITSWITCH FOR NORMAL RAISING/LOWERING
            elevator.toggleLowerLimit(true);
        });
    }
}
