package frc.robot;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.PWMSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.SimConstants;

public class ArmSim {
    private final DCMotor elevatorGearbox = DCMotor.getNEO(SimConstants.kNumElevatorMotors);
    private final DCMotor pivotGearbox = DCMotor.getNEO(SimConstants.kNumPivotMotors);

    private final PWMSparkMax elevatorMotor = new PWMSparkMax(SimConstants.kElevatorMotorPort);
    private final PWMSparkMax pivotMotor = new PWMSparkMax(SimConstants.kPivotMotorPort);

    private final Encoder elevatorEncoder = new Encoder(SimConstants.kElevatorEncoderAChannel, SimConstants.kElevatorEncoderBChannel);
    private final Encoder pivotEncoder = new Encoder(SimConstants.kPivotEncoderAChannel, SimConstants.kPivotEncoderBChannel);

    private final ProfiledPIDController elevatorController = 
        new ProfiledPIDController(SimConstants.kElevatorP, SimConstants.kElevatorI, SimConstants.kElevatorD, new TrapezoidProfile.Constraints(10, 5));
    private final ProfiledPIDController pivotController = 
        new ProfiledPIDController(SimConstants.kPivotP, SimConstants.kPivotI, SimConstants.kPivotD, new TrapezoidProfile.Constraints(30, 10));

    private final ElevatorSim elevatorSim = 
        new ElevatorSim(
            elevatorGearbox, 
            SimConstants.kElevatorGearing, 
            SimConstants.kElevatorCarriageMass, 
            SimConstants.kElevatorDrumRadius, 
            SimConstants.kMinimumElevatorLength, 
            SimConstants.kMaximumElevatorLength,
            true,
            0.5);

    private final SingleJointedArmSim pivotSim =
        new SingleJointedArmSim(
            pivotGearbox, 
            SimConstants.kPivotGearing, 
            SingleJointedArmSim.estimateMOI(SimConstants.kMaximumElevatorLength / 2.0, SimConstants.kPivotArmMass), 
            SimConstants.kMaximumElevatorLength, 
            Rotation2d.fromDegrees(SimConstants.kMinimumPivotAngle).getRadians(), 
            Rotation2d.fromDegrees(SimConstants.kMaximumPivotAngle).getRadians(), 
            true, 
            Units.degreesToRadians(90));

    private final EncoderSim elevatorEncoderSim = new EncoderSim(elevatorEncoder);
    private final EncoderSim pivotEncoderSim = new EncoderSim(pivotEncoder);

    private final PWMSim elevatorMotorSim = new PWMSim(elevatorMotor);
    private final PWMSim pivotMotorSim = new PWMSim(pivotMotor);

    private final Mechanism2d mechanism = new Mechanism2d(Units.inchesToMeters(20), Units.inchesToMeters(50));
    private final MechanismRoot2d root = mechanism.getRoot("Arm Root", Units.inchesToMeters(3), Units.inchesToMeters(4));
    private final MechanismLigament2d arm = root.append(
        new MechanismLigament2d(
            "Arm", 
            1, 
            45.0));

    public ArmSim() {
        elevatorEncoder.setDistancePerPulse(SimConstants.kElevatorEncoderDistPerPulse);
        pivotEncoder.setDistancePerPulse(SimConstants.kPivotEncoderDistPerPulse);

        SmartDashboard.putData("ArmSim", mechanism);
    }

    public void simulationPeriodic() {
        elevatorSim.setInput(elevatorMotorSim.getSpeed() * RobotController.getBatteryVoltage());
        pivotSim.setInput(pivotMotorSim.getSpeed() * RobotController.getBatteryVoltage());

        elevatorSim.update(SimConstants.kLoopTime);
        pivotSim.update(SimConstants.kLoopTime);

        elevatorEncoderSim.setDistance(elevatorSim.getPositionMeters());
        pivotEncoderSim.setDistance(pivotSim.getAngleRads());

        RoboRioSim.setVInVoltage(
            BatterySim.calculateDefaultBatteryLoadedVoltage(
                elevatorSim.getCurrentDrawAmps() + pivotSim.getCurrentDrawAmps()
            )
        );
    }

    public void setElevatorGoal(double goal) {
        elevatorController.setGoal(goal);

        double pidOutput = elevatorController.calculate(elevatorEncoder.getDistance());
        elevatorMotor.setVoltage(pidOutput);
    }

    public void setPivotGoal(double goal) {
        pivotController.setGoal(goal);

        double pidOutput = pivotController.calculate(pivotEncoder.getDistance());
        pivotMotor.setVoltage(pidOutput);
    }

    public void stopElevator() {
        elevatorController.setGoal(0.0);
        elevatorMotor.set(0.0);
    }

    public void stopPivot() {
        pivotController.setGoal(0.0);
        pivotMotor.set(0.0);
    }

    public void updateTelemetry() {
        arm.setLength(elevatorSim.getPositionMeters());
        arm.setAngle(Rotation2d.fromRadians(pivotSim.getAngleRads()));
    }
}
