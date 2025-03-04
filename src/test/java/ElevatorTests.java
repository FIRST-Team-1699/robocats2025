import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.hal.HAL;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorPosition;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class ElevatorTests {
    @Test
    void l4SafePosition() {
        assertEquals(false, ElevatorPosition.L_FOUR.shouldPivotMoveFromHere());
    }

    @Test
    void l3SafePosition() {
        assertEquals(false, ElevatorPosition.L_THREE.shouldPivotMoveFromHere());
    }

    @Test
    void l2SafePosition() {
        assertEquals(true, ElevatorPosition.L_TWO.shouldPivotMoveFromHere());
    }

    @Test
    void l1SafePosition() {
        assertEquals(true, ElevatorPosition.L_ONE.shouldPivotMoveFromHere());
    }

    @Test
    void primeSafePosition() {
        assertEquals(true, ElevatorPosition.PRIME.shouldPivotMoveFromHere());
    }


    @Test
    void coralStationSafePosition() {
        assertEquals(true, ElevatorPosition.CORAL_STATION_INTAKE.shouldPivotMoveFromHere());
    }


    @Test
    void storedSafePosition() {
        assertEquals(true, ElevatorPosition.STORED.shouldPivotMoveFromHere());
    }

    @Test
    void groundIntakeSafePosition() {
        assertEquals(true, ElevatorPosition.GROUND_INTAKE.shouldPivotMoveFromHere());
    }

    @Test
    void safePositionSafePosition() {
        assertEquals(true, ElevatorPosition.SAFE_POSITION.shouldPivotMoveFromHere());
    }
}
