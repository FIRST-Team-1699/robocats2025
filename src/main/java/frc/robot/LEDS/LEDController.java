package frc.robot.LEDS;

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

    public void changeColor(LEDCurrentHSV currentTargetColor) {
        for (int ledSquares = 0; ledSquares < ledBuffer.getLength(); ledSquares++) {
            // @KEVIN, I think index refers to the LED length segments, but can you check me on that
            ledBuffer.setHSV(ledSquares, currentTargetColor.hue, currentTargetColor.saturation, currentTargetColor.value);
        }
        leds.setData(ledBuffer);
    }
    
    public void start() {
        leds.start();
    }

    public void stop() {
        leds.stop();
    }
}
