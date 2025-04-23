// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import java.util.Optional;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.PrintCommand;

public class Robot extends TimedRobot {
  private Command autoCommand;

  private final RobotContainer m_robotContainer;

  private final SendableChooser<String> autoChooser;

  private final String processor3L1Descore2 = "Processor3Piece2Descore";
  private final String processor3L1NoDescore = "Processor3PieceNoDescore";
  private final String barge3L1Descore2 = "Barge3Piece2Descore";
  private final String barge3L1NoDescore = "Barge3PieceNoDescore";
  private final String center1L4 = "Center1L4";
  private final String center1L1Descore1 = "Center1L1Descore1";
  private final String doNothing = "DoNothing";
  private final String autoAlignL4Processor = "Processor L4";
  private final String autoAlignL4Barge = "Barge L4";

  private Optional<Alliance> lastAlliance;
  private String selectedAutoString;

  public Robot() {
    m_robotContainer = new RobotContainer();
    autoChooser = new SendableChooser<>();
    autoChooser.addOption("Processor 3 L1 Descore 2", processor3L1Descore2);
    autoChooser.addOption("PROCESSOR FOR 1153 NO DESCORE 3 L1", processor3L1NoDescore);
    autoChooser.addOption("Barge 3 L1 Descore 2", barge3L1Descore2);
    autoChooser.setDefaultOption("BARGE FOR 1153 NO DESCORE 3 L1", barge3L1NoDescore);
    autoChooser.addOption("Center 1 L4 Descore 1", center1L4);
    autoChooser.addOption("Do Nothing", doNothing);
    autoChooser.addOption("Auto Align L4 Processor", autoAlignL4Processor);
    autoChooser.addOption("Auto Align L4 Barge", autoAlignL4Barge);
    autoChooser.setDefaultOption("Center 1 L1 Descore 1", center1L1Descore1);
    SmartDashboard.putData(autoChooser);

    lastAlliance = DriverStation.getAlliance();
    selectedAutoString = autoChooser.getSelected();
    autoCommand = AutoBuilder.buildAuto(autoChooser.getSelected());
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run(); 
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {
    if(!DriverStation.getAlliance().equals(lastAlliance) || !autoChooser.getSelected().equalsIgnoreCase(selectedAutoString)) {
      System.out.println(DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get().toString() : "NOT READY YET");
      lastAlliance = DriverStation.getAlliance();
      selectedAutoString = autoChooser.getSelected();
      if(selectedAutoString == doNothing) {
        autoCommand = new PrintCommand("Do Nothing");
      } else {
        autoCommand = AutoBuilder.buildAuto(selectedAutoString);
      }
    }
  }

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    // CHANGES AUTO STATE TO TRUE FOR ALIGNTOREEFCOMMAND
    RobotContainer.inAuto = true;
    if (autoCommand != null) {
      autoCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {
    // CHANGES AUTO STATE TO FALSE FOR ALIGNTOREEFCOMMAND
    RobotContainer.inAuto = false;
  }

  @Override
  public void teleopInit() {
    if (autoCommand != null) {
      autoCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}
