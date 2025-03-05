package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;
import frc.robot.subsystems.PivotSubsystem.PivotPosition;


// NOTE: THIS CLASS MAY NOT WORK INDEPENDENTLY BECAUSE THE REQUIRED SUBSYSTEMS ARE INVISIBLE TO IT 
public class LEDController extends SubsystemBase {
    // DECLARATIONS
    private AddressableLED leds;
    private AddressableLEDBuffer ledBuffer;
    private double cycleTicks, blinkTicks;

    private TargetRGB currentRGB;
    private boolean blink;

    private ElevatorSubsystem elevator;
    private PivotSubsystem pivot;
    private RotateWristSubsystem rotateWrist;
    private TiltWristSubsystem tiltWrist;
    private IntakeSubsystem intake;

    public LEDController(ElevatorSubsystem elevator, PivotSubsystem pivot, TiltWristSubsystem tiltWrist, RotateWristSubsystem rotateWrist, IntakeSubsystem intake) {
        // Constructor for LED objects
        leds = new AddressableLED(LEDConstants.kPort);
        ledBuffer = new AddressableLEDBuffer(LEDConstants.kLEDLength);

        // Sets LED length to leds
        leds.setLength(LEDConstants.kLEDLength);

        cycleTicks = 0;

        // SUBSYSTEMS TO USE FOR CONDITIONALS
        this.elevator = elevator;
        this.pivot = pivot;
        this.rotateWrist = rotateWrist;
        this.tiltWrist = tiltWrist;
        this.intake = intake;

        blink = false;

        currentRGB = TargetRGB.BASE;
        start();
        changeColor(currentRGB);
    }

    /**Changes the colot of LEDs */
    public void changeColor(TargetRGB targetRGB) {
        // STOPS FROM WASTING RESOURCES VIA RETURN
        if(currentRGB.equals(targetRGB)) return;
        // ALLOWS LEDS TO TURN ON/OFF, BLINK FOR INTAKE
        if(blink && blinkTicks % 20 <= 10) {
            changeColor(TargetRGB.NONE);
        } else {
            changeColor(targetRGB);
        }
        leds.setData(ledBuffer);
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
    // @Override
    // public void periodic() {
    //     cycleTicks++;
    //     blinkTicks++;
    //     if(blinkTicks > 50000) {
    //         blinkTicks = 0;
    //     }
    //     if(cycleTicks >= 10) {
    //         if(!elevator.isAtSetpoint() 
    //         || !pivot.isAtSetpoint() 
    //         || !rotateWrist.isAtSetpoint()
    //         || !tiltWrist.isAtSetpoint()) {
    //             changeColor(TargetRGB.IN_TRANSITION);
    //         } else {
    //             if(intake.hasPiece()) {
    //                 changeColor(TargetRGB.OBTAINED_CORAL);
    //             } else {
    //                 if(pivot.currentTargetPosition == PivotPosition.STORED) {
    //                     changeColor(TargetRGB.BASE);
    //                 } else {
    //                     changeColor(TargetRGB.REACHED_POSITION);
    //                 }
    //             }
    //         } 
    //         cycleTicks = 0;
    //     }
    //     if(intake.isRunning()) {
    //         blink = true;
    //     } else {
    //         blink = false;
    //     }

    //     SmartDashboard.putBoolean("LEDs Blinking", blink);
    //     SmartDashboard.putNumber("Cycle Ticks", cycleTicks);
    //     SmartDashboard.putNumber("Blink Ticks", blinkTicks);
    // }

    /**enums for determining RGBs of LEDs*/
    public enum TargetRGB {
        NONE(0, 0, 0),

        BASE(18, 148, 255), // Blue

        IN_TRANSITION(255, 13, 13),// Red

        OBTAINED_CORAL(240, 200, 0),// Gold

        IS_PECKING(255, 255, 255),// White

        REACHED_POSITION(49, 255, 13);//Green

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