package frc.robot.commands;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends Command{
    private IntakeSubsystem intakeSubsystem;

    public IntakeCommand(IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);
    }

    public void initalize() {
        intakeSubsystem.intake();
    }

    public void execute() {
}

    public boolean isFinished() {
        return false;
        //Requires BeamBreak
}

    public void end() {
        intakeSubsystem.stop();
    }
}
