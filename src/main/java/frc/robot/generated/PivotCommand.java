package frc.robot.generated;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.PivotSubsystem.PivotPosition;

public class PivotCommand extends Command{
    private PivotSubsystem pivot;
    private PivotPosition pos;

    public PivotCommand(PivotSubsystem pivot, PivotPosition pos) {
        this.pivot = pivot;
        this.pos = pos;
    }

    public void Initalize() {
        pivot.setPivot(pos);
    }

    public void execute() {
    }
    
    public boolean isFinished() {
        return false;
        //TODO: ADD A CONDITIONAL TO END COMMAND
    }

    public void end(boolean isInterrupted) {
    }
}
