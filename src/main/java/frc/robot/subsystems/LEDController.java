package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;


// NOTE: THIS CLASS MAY NOT WORK INDEPENDENTLY BECAUSE THE REQUIRED SUBSYSTEMS ARE INVISIBLE TO IT 
public class LEDController extends SubsystemBase {
    // DECLARATIONS
    private AddressableLED leds;
    private AddressableLEDBuffer ledBuffer;
    private double cycleTicks, totalTicks;

    private TargetRGB currentRGB;
    private boolean blink;

    public LEDController() {
        // Constructor for LED objects
        leds = new AddressableLED(LEDConstants.kPort);
        ledBuffer = new AddressableLEDBuffer(LEDConstants.kLEDLength);

        // Sets LED length to leds
        leds.setLength(LEDConstants.kLEDLength);

        cycleTicks = 0;

        // SUBSYSTEMS TO USE FOR CONDITIONALS
        // elevator = new ElevatorSubsystem();
        // pivot = new PivotSubsystem();
        // rotateWrist = new RotateWristSubsystem();
        // tiltWrist = new TiltWristSubsystem();
        blink = false;

        currentRGB = TargetRGB.NONE;
        start();
    }

    /**Changes the colot of LEDs */
    public void changeColor(TargetRGB targetRGB) {
        // STOPS FROM WASTING RESOURCES VIA RETURN
        if(currentRGB.equals(targetRGB)) return;
        // ALLOWS LEDS TO TURN ON/OFF, BLINK FOR INTAKE
        if(blink && totalTicks % 50 <= 25) {
            setLedBuffer(TargetRGB.NONE);
        } else {
            setLedBuffer(targetRGB);
        }
        leds.setData(ledBuffer);
    }
    /**sets the RGB of entire LED strip defined in ledBuffer
     * @param targetRGB 
     * Uses stored enum data to assign LED RGB
     */
    public void setLedBuffer(TargetRGB targetRGB) {
        for (int ledSquares = 0; ledSquares < ledBuffer.getLength(); ledSquares++) {
            ledBuffer.setRGB(ledSquares, targetRGB.red, targetRGB.blue, targetRGB.green);
        }
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
    //     // TODO USE KNOWN DATA FROM EACH SUBSYSTEM AND MAKE A DECISION FOR THE LED STATE BASED ON ALL OPF THE SUBSYSTEMS\
    //     cycleTicks++;
    //     totalTicks++;
    //     if(totalTicks > 50000) {
    //         totalTicks = 0;
    //     }
    //     if(cycleTicks >= 10) {
    //         if(!elevator.isAtSetpoint() 
    //         || !pivot.isAtSetpoint() 
    //         || !rotateWrist.isAtSetpoint()
    //         || !tiltWristSusbsytem.isAtSetpoint()) {
    //             changeColor(TargetRGB.IN_TRANSITION);
    //         } else {
    //             if(elevator.currentTargetPosition != ElevatorPositions.STORED) {
    //                 changeColor(TargetRGB.AT_POSITION);
    //             } else {
    //                 changeColor(TargetRGB.BASE);
    //             }
    //             if(intake.isRunning()) {
    //                 blink = true;
    //             } else {
    //                 blink = false;
    //             }
    //         } 
    //         cycleTicks =0;
    //     }
    // }

    /**enums for determining RGBs of LEDs*/
    public enum TargetRGB {
        NONE(0, 0, 0),

        BASE(18, 148, 255), // Blue

        IN_TRANSITION(255, 13, 13),// Red

        OBTAINED_CORAL(240, 200, 0),// Gold

        IS_PECKING(255,255,255),// White

        REACHED_POSITION(49,255, 13);//Green



        // THE Following are maybes to use

        // IS_PECKING(-1,-1,-1),

        
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
