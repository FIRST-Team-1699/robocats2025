package frc.robot.subsystems.elevator;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class ZeroElevator extends ElevatorSubsystem{
    // @KEVIN, Probably too much abstraction, but I think this could be better 
    //(and am excited for you to yell at me, so I can learn why this is wrong on discord :))

    // IRREVELANT DECLARATION
    // private final ElevatorSubsystem elevator;
    
    // HAD A CONSTRUCTOR, BUT IS POINTLESS FOR PURPOSE OF CLASS
    // public ZeroElevator(ElevatorSubsystem elevator) {
    //     this.elevator = elevator;
    // }


    // COMMAND FACTORIES TO ZERO ELEVATOR

    /** Stops the motor manually, used if Command is interrupted. */
    public void stopMotorManual() {
        leadMotor.set(0);
    }

    /**Runs a WaitUntilCommand, waits until elevator reaches bottom */
    private Command waitWhileLowerElevator() {
        return new WaitUntilCommand(() -> {
            leadMotor.set(-0.2);
            return isAtBottom();
        });
    }
    /**Resets encoder to 0 after zeroing */
    private Command resetEncoder() {
        return runOnce(() -> {
            targetRelativeEncoder.setPosition(0);
        });
    }
    /**Ensures that motor is set to 0 after triggering bottomLimitSwitch*/
    private Command stopMotorCommand() {
        return runOnce(() -> {
            leadMotor.set(0);
        });
    }

    /**Enables or disables reverseSoftLimmit
     * @param enabled
     * considers wether (based on boolean data) lowerLimit is to be disable or enabled
     */
    private Command toggleLowerLimit(boolean enabled) {
        return runOnce(()-> {
            leftConfig.softLimit.reverseSoftLimitEnabled(enabled);
            leadMotor.configureAsync(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        });
    }

    /**Returns if bottomLimitSwitch has been reached, should not be used publicly */
    private boolean isAtBottom() {
        return bottomLimitSwitch.isPressed();
    }

    /**Runs a Command Group to zero elevator */
    public Command subsequentialZeroCommandGroup() {
        return new SequentialCommandGroup(
            // SETS CONDITIONS FOR ZEROING
            toggleLowerLimit(false),
            // LOWERS UNTIL REACHING LIMIT SWITCH, WAITUNTILCOMMAND
            waitWhileLowerElevator(),
            // RESETS MOTOR/ENCODER
            stopMotorCommand(),
            resetEncoder(),
            // ENABLES SOFTLIMITSWITCH FOR NORMAL RAISING/LOWERING
            toggleLowerLimit(true)
        );
    }
}
