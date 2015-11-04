package org.usfirst.ftc.exampleteam.yourcodehere;

import org.swerverobotics.library.SynchronousOpMode;
import org.swerverobotics.library.interfaces.TeleOp;

/**
 * A skeletal example of a do-nothing first OpMode. Go ahead and change this code
 * to suit your needs, or create sibling OpModes adjacent to this one in the same
 * Java package.
 */
@TeleOp(name="Template Op Mode")
public class TemplateOpMode extends SynchronousOpMode {
    /* Declare here any fields you might find useful. */
    // DcMotor motorLeft = null;
    // DcMotor motorRight = null;

    @Override public void main() throws InterruptedException {
        /* Initialize our hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names you assigned during the robot configuration
         * step you did in the FTC Robot Controller app on the phone.
         */
        // this.motorLeft = this.hardwareMap.dcMotor.get("motorLeft");
        // this.motorRight = this.hardwareMap.dcMotor.get("motorRight");

        // Wait for the game to start
        waitForStart();

        // Go go gadget robot!
        while (opModeIsActive()) {
            if (updateGamepads()) {
                // The game pad state has changed. Do something with that!
            }

            telemetry.update();
            idle();
        }
    }
}
