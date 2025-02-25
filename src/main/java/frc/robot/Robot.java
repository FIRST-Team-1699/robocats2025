// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;
  private ArmSim armSim;
  private XboxController simController;

  public Robot() {
    m_robotContainer = new RobotContainer();
    if(Robot.isSimulation()) {
      armSim = new ArmSim();
      simController = new XboxController(0);
    }
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run(); 

    if(Robot.isSimulation()) {
      armSim.updateTelemetry();
    }
  }

  @Override
  public void disabledInit() {
    if(Robot.isSimulation()) {
      armSim.stopElevator();
    }
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
    if(Robot.isSimulation()) {
      if(simController.getAButton()) {
        armSim.setElevatorGoal(1);
      } else {
        armSim.setElevatorGoal(0.05);
      }
      if(simController.getBButton()) {
        armSim.setPivotGoal(Units.degreesToRadians(90));
      } else {
        armSim.setPivotGoal(Units.degreesToRadians(45));
      }
    }
  }

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
  public void simulationPeriodic() {
    armSim.simulationPeriodic();
  }
}
