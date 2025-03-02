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
        assertEquals(true, PivotSubsystem.isRobotPositionSafe(PivotPosition.L_FOUR));
    }

    @Test
    void l3SafePosition() {
        assertEquals(true, PivotSubsystem.isRobotPositionSafe(PivotPosition.L_THREE));
    }

    @Test
    void l2SafePosition() {
        assertEquals(true, PivotSubsystem.isRobotPositionSafe(PivotPosition.L_TWO));
    }

    @Test
    void l1SafePosition() {
        assertEquals(true, PivotSubsystem.isRobotPositionSafe(PivotPosition.L_ONE));
    }

    @Test
    void primeSafePosition() {
        assertEquals(true, PivotSubsystem.isRobotPositionSafe(PivotPosition.PRIME));
    }


    @Test
    void coralStationSafePosition() {
        assertEquals(true, PivotSubsystem.isRobotPositionSafe(PivotPosition.CORAL_STATION_INTAKE));
    }


    @Test
    void storedSafePosition() {
        assertEquals(false, PivotSubsystem.isRobotPositionSafe(PivotPosition.STORED));
    }

    @Test
    void groundIntakeSafePosition() {
        assertEquals(false, PivotSubsystem.isRobotPositionSafe(PivotPosition.GROUND_INTAKE));
    }

    @Test
    void safePositionSafePosition() {
        assertEquals(true, PivotSubsystem.isRobotPositionSafe(PivotPosition.SAFE_POSITION));
    }
}

