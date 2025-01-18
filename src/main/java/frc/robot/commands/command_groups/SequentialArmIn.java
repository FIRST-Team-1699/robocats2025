package frc.robot.commands.command_groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorSetpoints;

public class SequentialArmIn extends SequentialCommandGroup {
    public SequentialArmIn(ElevatorSubsystem elevator, ElevatorSetpoints endExtension) {
        addRequirements(elevator);
        
        addCommands(elevator.targetHome());
    }
}
