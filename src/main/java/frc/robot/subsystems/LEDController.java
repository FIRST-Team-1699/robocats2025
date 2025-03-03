package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorPosition;


// NOTE: THIS CLASS MAY NOT WORK INDEPENDENTLY BECAUSE THE REQUIRED SUBSYSTEMS ARE INVISIBLE TO IT 
public class LEDController extends SubsystemBase {
    // DECLARATIONS
    private AddressableLED leds;
    private AddressableLEDBuffer ledBuffer;
    private double cycleTicks, totalTicks;

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
        if(blink && totalTicks % 20 <= 10) {
            changeColor(TargetRGB.NONE);
        } else {
            changeColor(targetRGB);
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
    @Override
    public void periodic() {
        // TODO USE KNOWN DATA FROM EACH SUBSYSTEM AND MAKE A DECISION FOR THE LED STATE BASED ON ALL OPF THE SUBSYSTEMS\
        cycleTicks++;
        totalTicks++;
        if(totalTicks > 50000) {
            totalTicks = 0;
        }
        if(cycleTicks >= 10) {
            if(!elevator.isAtSetpoint() 
            || !pivot.isAtSetpoint() 
            || !rotateWrist.isAtSetpoint()
            || !tiltWrist.isAtSetpoint()) {
                changeColor(TargetRGB.IN_TRANSITION);
            } else {
                if(elevator.currentTargetPosition != ElevatorPosition.STORED) {
                    changeColor(TargetRGB.REACHED_POSITION);
                } else {
                    changeColor(TargetRGB.BASE);
                }
                if(intake.isRunning()) {
                    blink = true;
                } else {
                    blink = false;
                }
            } 
            cycleTicks = 0;
        }
    }

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