package frc.robot.commands;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.HeightPosistions;

public class ElevatorLevelCommand extends Command{
    private ElevatorSubsystem elevatorSubsystem;

    public ElevatorLevelCommand(ElevatorSubsystem elevatorSubsystem) {
        this.elevatorSubsystem = elevatorSubsystem;
        addRequirements(elevatorSubsystem);
    }

    public void initalize(HeightPosistions heightPosistion) {
        elevatorSubsystem.changeEleavtorHeight(heightPosistion.heightCentimeters);
    }

    public void execute() {
        
    }

    public boolean isFinished() {
        return false;
        //Figure out parameters to put here
    }

    public void end(boolean isInterrupted) {

    }
}
