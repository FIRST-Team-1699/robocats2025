package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.LEDConstants;


// NOTE: THIS CLASS MAY NOT WORK INDEPENDENTLY BECAUSE THE REQUIRED SUBSYSTEMS ARE INVISIBLE TO IT 
public class LEDController implements Subsystem {
    // DECLARATIONS
    private AddressableLED leds;
    private AddressableLEDBuffer ledBuffer;
    private double cycleTicks, totalTicks;

    private TargetHSV currentHSV;
    private boolean blink;

    private ElevatorSubsystem elevator;
    private PivotSubsystem pivot;
    private RotateWristSubsystem rotateWrist;
    private TiltWristSubsystem tiltWrist;

    /**LED Constructor is set here
     * @param port
     * Location (port) set for LED strips
     * @param ledLength
     * The total length of LED strips
     */
    public LEDController(
        PivotSubsystem pivot, ElevatorSubsystem elevator, RotateWristSubsystem rotateWrist, TiltWirstSubsystem tiltWrist) {
        // Constructor for LED objects
        leds = new AddressableLED(LEDConstants.kPort);
        ledBuffer = new AddressableLEDBuffer(LEDConstants.kLED_Length);

        // Sets LED length to leds
        leds.setLength(LEDConstants.kPort);

        cycleTicks = 0;

        // SUBSYSTEMS TO USE FOR CONDITIONALS
        elevator = new ElevatorSubsystem();
        pivot = new PivotSubsystem();
        rotateWrist = new RotateWristSubsystem();
        tiltWrist = new TiltWristSubsystem();
        currentHSV = TargetHSV.NONE;
    }
    /**Changes the colot of LEDs */
    public void changeColor(TargetHSV targetHSV) {
        // STOPS FROM WASTING RESOURCES VIA RETURN
        if(currentHSV.equals(targetHSV)) return;
        // ALLOWS LEDS TO TURN ON/OFF, BLINK FOR INTAKE
        if(blink && totalTicks % 20 <= 10) {
            setLedBuffer(TargetHSV.NONE);
        } else {
            setLedBuffer(targetHSV);
        }
        leds.setData(ledBuffer);
    }
    /**sets the HSV of entire LED strip defined in ledBuffer
     * @param targetHSV 
     * Uses stored enum data to assign LED HSV
     */
    public void setLedBuffer(TargetHSV targetHSV) {
        for (int ledSquares = 0; ledSquares < ledBuffer.getLength(); ledSquares++) {
            ledBuffer.setHSV(ledSquares, targetHSV.hue, targetHSV.saturation, targetHSV.value);
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
            || !tiltWristSusbsytem.isAtSetpoint()) {
                changeColor(TargetHSV.IN_TRANSITION);
            } else {
                if(elevator.currentTargetPosition != ElevatorPositions.STORED) {
                    changeColor(TargetHSV.AT_POSITION);
                } else {
                    changeColor(TargetHSV.BASE);
                }
                if(intake.isRunning()) {
                    blink = true;
                } else {
                    blink = false;
                }
            } 
            cycleTicks =0;
        }
    }

    /**enums for determining HSVs of LEDs*/
    public enum TargetHSV {
        NONE(0, 0, 0),

        BASE(-1,-1,-1), // BLUE

        IN_TRANSITION(-1,-1,-1),

        // THE Following are maybes to use

        // REACHED_POSITION(-1,-1,-1),

        // IS_PECKING(-1,-1,-1),

        // OBTAINED_CORAL(-1,-1,-1),

        // INTAKING(-1,-1,-1);

        
        // VARIABLES TO SET INTS for LEDs' enums
        int hue;
        int saturation;
        int value;

        /**Constructor for LEDs color
         * @param hue
         * Determines hue of LEDs
         * @param saturation
         * determines saturation of LEDs
         * @param saturation
         * determines value of LEDs
         */
        private TargetHSV(int hue, int saturation, int value) {
            this.hue = hue;
            this.saturation = saturation;
            this.value = value;
        }
    }
}
