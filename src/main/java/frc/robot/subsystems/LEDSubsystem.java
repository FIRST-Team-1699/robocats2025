package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.LEDConstants;
import frc.robot.utils.LimelightHelpers;

public class LEDSubsystem extends SubsystemBase {
    private AddressableLED leds;
    private AddressableLEDBuffer ledBuffer;
    private double cycleTicks, blinkTicks;

    private TargetRGB currentRGB;
    private boolean blink;

    private IntakeSubsystem intake;
    private CommandSwerveDrivetrain drivetrain;

    public LEDSubsystem(IntakeSubsystem intake, CommandSwerveDrivetrain drivetrain) {
        this.leds = new AddressableLED(LEDConstants.kPort);
        this.ledBuffer = new AddressableLEDBuffer(LEDConstants.kLEDLength);

        leds.setLength(LEDConstants.kLEDLength);

        this.cycleTicks = 0;
        this.blink = false;

        this.intake = intake;
        this.drivetrain = drivetrain;

        this.currentRGB = TargetRGB.BLUE;
        start();
        changeColor(currentRGB);
    }

    /** Changes the color of LEDs. */
    public void changeColor(TargetRGB targetRGB) {
        if(blink && blinkTicks < 10) {
            setColorDirectly(TargetRGB.NONE);
        } else {
            setColorDirectly(targetRGB);
        }
        leds.setData(ledBuffer);
    }

    public void setColorDirectly(TargetRGB targetRGB) {
        for(int i = 0; i < LEDConstants.kLEDLength; i++) {
            ledBuffer.setRGB(i, targetRGB.red, targetRGB.green, targetRGB.blue);
        }
        leds.setData(ledBuffer);
        currentRGB = targetRGB;
    }

    public void start() {
        leds.start();
    }

    public void stop() {
        leds.stop();
    }

    /** Updates LED color periodically based on subsystem states. */
    @Override
    public void periodic() {
        if(drivetrain.getState().Speeds.vxMetersPerSecond > 0.1 || drivetrain.getState().Speeds.vyMetersPerSecond > 0.1 || drivetrain.getState().Speeds.omegaRadiansPerSecond > 0.1) {
            RobotContainer.isAligned = false;
        }
        cycleTicks++;
        blinkTicks++;
        if(blinkTicks > 20) {
            blinkTicks = 0;
        }
        if(cycleTicks >= 10) {
            if(RobotContainer.isAligned) {
                changeColor(TargetRGB.GREEN);
            } else if(LimelightHelpers.getTV("limelight")) {
                changeColor(TargetRGB.RED);
            } else if(intake.hasPiece()) {
                changeColor(TargetRGB.GOLD);
            } else {
                changeColor(TargetRGB.BLUE);
            }
        }
        if(intake.isRunning()) {
            blink = true;
        } else {
            blink = false;
        }

        SmartDashboard.putBoolean("LEDs Blinking", blink);
        SmartDashboard.putNumber("Cycle Ticks", cycleTicks);
        SmartDashboard.putNumber("Blink Ticks", blinkTicks);
    }

    /** Enums for determining RGBs of LEDs */
    public enum TargetRGB {
        NONE(0, 0, 0),

        BLUE(18, 255, 148),

        RED(255, 13, 13),

        GOLD(255, 180, 0),

        GREEN(5, 255, 10);

        final int red;
        final int green;
        final int blue;

        private TargetRGB(int red, int green, int blue) {
            this.red = red;
            this.green = green;
            this.blue = blue;
        }
    }
}