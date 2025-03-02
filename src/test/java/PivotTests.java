import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.hal.HAL;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.PivotSubsystem.PivotPosition;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class AlgorithmTests {
    @Test
    void safePositionAlgorithmWhenSafe() {
        assertEquals(true, PivotSubsystem.isRobotPositionSafe(PivotPosition.L_FOUR));
    }

    @Test
    void safePositionAlgorithmWhenUnsafe() {
        assertEquals(false, PivotSubsystem.isRobotPositionSafe(PivotPosition.GROUND_INTAKE));
    }
}

