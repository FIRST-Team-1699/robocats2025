package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.Command;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;


import edu.wpi.first.wpilibj.RobotController;

public class WristSubsystem implements Subsystem{
    //these motors might not end up being krakens or even falcons
    private TalonFX rotatingMotor, tiltingMotor;
    private CANSparkMax intakingMotor;
    public WristSubsystem() {
        //stuff
    }
}