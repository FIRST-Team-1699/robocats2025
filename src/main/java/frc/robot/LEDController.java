package frc.robot;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;



public class LEDController{
    private AddressableLED leds;
    private AddressableLEDBuffer ledBuffer;


    public LEDController(int port, int ledLength) {
        leds = new AddressableLED(port);
        ledBuffer = new AddressableLEDBuffer(ledLength);

        leds.setData(ledBuffer);
        leds.setLength(ledLength);
    }

    public void changeColor(TargetHSV targetHSV) {
        ledBuffer = setLedBuffer(targetHSV);

        leds.setData(ledBuffer);
    }

    public AddressableLEDBuffer setLedBuffer(TargetHSV targetHSV) {
        for (int ledSquares = 0; ledSquares < ledBuffer.getLength(); ledSquares++) {
            // @KEVIN, I think index refers to the LED length segments, but can you check me on that
            ledBuffer.setHSV(ledSquares, targetHSV.hue, targetHSV.saturation, targetHSV.value);
        }
        return ledBuffer;
    }
    
    public void start() {
        leds.start();
        changeColor(TargetHSV.START_UP);
    }

    public void stop() {
        leds.stop();
    }
    
    public enum TargetHSV {
        // BLUE
        START_UP(-1,-1,-1),
        // ORANGE
        IN_TRANSITION(-1,-1,-1),
        // GREEN
        REACHED_POSITION(-1,-1,-1),
        // WHITE OR OTHER COLOR
        OBTAINED_CORAL(-1,-1,-1);
        
        
        int hue;
        int saturation;
        int value;
        private TargetHSV(int hue, int saturation, int value) {
            this.hue = hue;
            this.saturation = saturation;
            this.value = value;
        }
    }
}
