package frc.robot.commands.command_groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorSetpoints;

public class SequentialArmOut extends SequentialCommandGroup {
    public SequentialArmOut(ElevatorSubsystem elevator, ElevatorSetpoints endExtension) {
        addRequirements(elevator);
        
        addCommands(elevator.targetHome());
    }
}
