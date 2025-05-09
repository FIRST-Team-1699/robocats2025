package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.LEDConstants;
import frc.robot.utils.LimelightHelpers;


// NOTE: THIS CLASS MAY NOT WORK INDEPENDENTLY BECAUSE THE REQUIRED SUBSYSTEMS ARE INVISIBLE TO IT 
public class LEDSubsystem extends SubsystemBase {
    // DECLARATIONS
    private AddressableLED leds;
    private AddressableLEDBuffer ledBuffer;
    private double cycleTicks, blinkTicks;

    private TargetRGB currentRGB;
    private boolean blink;

    private IntakeSubsystem intake;
    private CommandSwerveDrivetrain drivetrain;

    public LEDSubsystem(IntakeSubsystem intake, CommandSwerveDrivetrain drivetrain) {
        // Constructor for LED objects
        leds = new AddressableLED(LEDConstants.kPort);
        ledBuffer = new AddressableLEDBuffer(LEDConstants.kLEDLength);

        // Sets LED length to leds
        leds.setLength(LEDConstants.kLEDLength);

        cycleTicks = 0;

        // SUBSYSTEMS TO USE FOR CONDITIONALS
        this.intake = intake;
        this.drivetrain = drivetrain;

        blink = false;

        currentRGB = TargetRGB.BLUE;
        start();
        changeColor(currentRGB);
    }

    /**Changes the colot of LEDs */
    public void changeColor(TargetRGB targetRGB) {
        // STOPS FROM WASTING RESOURCES VIA RETURN
        // ALLOWS LEDS TO TURN ON/OFF, BLINK FOR INTAKE
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

    // METHODS TO MANIPULATE LEDs in-match
    /**Enables LEDs and assigns color to "START_UP" Enum */
    public void start() {
        leds.start();
    }

    /**Disables LEDs*/
    public void stop() {
        leds.stop();
    }

    /**Runs periodically, uses conditionals to change LED color */
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
            if(RobotContainer.isAligned) { // magic condition which determines that we are aligned
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

    /**enums for determining RGBs of LEDs*/
    public enum TargetRGB {
        NONE(0, 0, 0), // NONE

        BLUE(18, 255, 148), // Blue

        RED(255, 13, 13),// Red

        GOLD(255, 180, 0),// Gold

        GREEN(5, 255, 10);//Green

        // VARIABLES TO SET INTS for LEDs' enums
        int red;
        int green;
        int blue;

        /**Constructor for LEDs color
         * @param hue
         * Determines hue of LEDs
         * @param saturation
         * determines saturation of LEDs
         * @param value
         * determines value of LEDs
         */
        private TargetRGB(int red, int green, int blue) {
            this.red = red;
            this.green = green;
            this.blue = blue;
        }
    }
}