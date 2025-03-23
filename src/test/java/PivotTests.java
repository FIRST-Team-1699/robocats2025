import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.hal.HAL;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.PivotSubsystem.PivotPosition;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class PivotTests {
    @Test
    void l4SafePosition() {
        assertEquals(true, PivotPosition.L_FOUR.canElevatorRetractFromHere());
    }

    @Test
    void l3SafePosition() {
        assertEquals(true, PivotPosition.L_THREE.canElevatorRetractFromHere());
    }

    @Test
    void l2SafePosition() {
        assertEquals(true, PivotPosition.L_TWO.canElevatorRetractFromHere());
    }

    @Test
    void l1SafePosition() {
        assertEquals(true, PivotPosition.L_ONE.canElevatorRetractFromHere());
    }

    @Test
    void primeSafePosition() {
        assertEquals(true, PivotPosition.PRIME.canElevatorRetractFromHere());
    }


    @Test
    void coralStationSafePosition() {
        assertEquals(true, PivotPosition.CORAL_STATION_INTAKE.canElevatorRetractFromHere());
    }


    @Test
    void storedSafePosition() {
        assertEquals(true, PivotPosition.STORED.canElevatorRetractFromHere());
    }

    @Test
    void groundIntakeSafePosition() {
        assertEquals(false, PivotPosition.GROUND_INTAKE.canElevatorRetractFromHere());
    }

    @Test
    void safePositionSafePosition() {
        assertEquals(true, PivotPosition.SAFE_POSITION.canElevatorRetractFromHere());
    }
}

