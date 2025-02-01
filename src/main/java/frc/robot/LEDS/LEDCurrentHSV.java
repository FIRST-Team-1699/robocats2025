// COOL SOURCES: https://www.w3schools.com/colors/colors_hsl.asp & https://docs.wpilib.org/pt/2023/docs/software/hardware-apis/misc/addressable-leds.html
package frc.robot.LEDS;

public class LEDCurrentHSV {
    int hue;
    int saturation;
    int value;
    public LEDCurrentHSV() {
        this.hue = CurrentState.START_UP.hue;
        this.value = CurrentState.START_UP.saturation;
        this.saturation = CurrentState.START_UP.value;
    }

    public enum CurrentState {
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
        private CurrentState(int hue, int saturation, int value) {
            this.hue = hue;
            this.saturation = saturation;
            this.value = value;
        }
    }
}
