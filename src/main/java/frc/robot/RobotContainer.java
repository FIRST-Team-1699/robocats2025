// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.PivotSubsystem;

public class RobotContainer {
    private final CommandXboxController operatorController = new CommandXboxController(1);   

    private final PivotSubsystem pivot = new PivotSubsystem();

    public RobotContainer() {
        // NOTES:
        // 1: ELEVATOR IS NOT IN BREAKMODE TO TEST FOR ENCODER VALUES
        // 2: CHECK SMART DASHBOARD FOR WHETHER ENCODERS READ 0. IF THEY DO, THERES AN ISSUE WITH THE ENCODER
        operatorController.povUp()
            .onTrue(pivot.setRaw(0.1))
            .onFalse(pivot.setRaw(0));
        operatorController.povDown()
            .onTrue(pivot.setRaw(-0.1))
            .onFalse(pivot.setRaw(0));
    }
}