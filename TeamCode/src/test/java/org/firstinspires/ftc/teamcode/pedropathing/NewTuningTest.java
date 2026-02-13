package org.firstinspires.ftc.teamcode.pedropathing;

import org.junit.Test;
import static org.junit.Assert.*;

@Autonomous(name )
public class NewTuningTest {

    @Test
    public void testNewTuningInstantiation() {
        // Instantiate the newTuning OpMode.
        // The constructor relies on the SelectableOpMode super constructor,
        // which takes a String and a Consumer lambda.
        // We are testing if the constructor itself can be called without immediate errors.
        // Real OpMode testing would involve mocking hardwareMap, telemetry, and gamepad
        // and calling init(), start(), and loop() methods.
        try {
            newTuning opMode = new newTuning();
            assertNotNull(opMode);
        } catch (Exception e) {
            fail("Failed to instantiate newTuning: " + e.getMessage());
        }
    }
}
