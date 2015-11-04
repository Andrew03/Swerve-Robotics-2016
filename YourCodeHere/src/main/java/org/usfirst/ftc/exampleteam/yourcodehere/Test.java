package org.usfirst.ftc.exampleteam.yourcodehere;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;

import org.swerverobotics.library.SynchronousOpMode;
import org.swerverobotics.library.interfaces.IBNO055IMU;
import org.swerverobotics.library.interfaces.TeleOp;
import org.swerverobotics.library.internal.AdaFruitBNO055IMU;
import org.swerverobotics.library.ClassFactory;

/**
 * A skeletal example of a do-nothing first OpMode. Go ahead and change this code
 * to suit your needs, or create sibling OpModes adjacent to this one in the same
 * Java package.
 */
@TeleOp(name="Test Op Mode")
public class Test extends SynchronousOpMode {
    /* Declare here any fields you might find useful. */

    // motor declarations
    DcMotor M_driveFR   = null, // front right drive motor
            M_driveFL   = null, // front left drive motor
            M_driveBR   = null, // back right drive motor
            M_driveBL   = null, // back left drive motor
            M_pickup    = null, // pickup motor
            M_lift      = null, // lift motor
            M_hangR     = null, // right hang motor
            M_hangL     = null; // left hang motor

    // servo declarations
    Servo   S_climbersR = null, // right servo that knocks down climbers
            S_climbersL = null, // left servo that knocks down climbers
            S_liftR     = null, // right servo that supports lift
            S_liftL     = null, // left servo that supports lift
            S_basketR   = null, // right servo on the basket
            S_basketL   = null, // left servo on the basket
            S_pickupFR  = null, // front right servo of the pickup
            S_pickupFL  = null, // front left servo of the pickup
            S_pickupSR  = null, // servo on right side of the pickup
            S_pickupSL  = null, // servo on left side of the pickup
            S_hitchR    = null, // right hitch servo
            S_hitchL    = null; // left hitch servo

    // sensor declarations
    IBNO055IMU adaFruit = null;
    IBNO055IMU.Parameters   parameters = new IBNO055IMU.Parameters();


    // all of the important constants
    final float STOP                   = 0.0f,
                MAX_POWER              = 1.0f;

    // all of the constant motor powers
    final float PICKUP_POWER = 1.0f,
                LIFT_POWER = 1.0f;

    // all of the starting servo positions
    final float S_CLIMBERS_START_POS_R = 0.0f,
                S_CLIMBERS_START_POS_L = 0.0f,
                S_LIFT_START_POS_R     = 0.0f,
                S_LIFT_START_POS_L     = 0.0f,
                S_BASKET_START_POS_R   = 0.0f,
                S_BASKET_START_POS_L   = 0.0f,
                S_PICKUP_START_POS_FR  = 0.0f,
                S_PICKUP_START_POS_FL  = 0.0f,
                S_PICKUP_START_POS_SR  = 0.0f,
                S_PICKUP_START_POS_SL  = 0.0f,
                S_HITCH_START_POS_R    = 0.0f,
                S_HITCH_START_POS_L    = 0.0f;

    // motor powers
    float   M_drivePowerR = STOP,
            M_drivePowerL = STOP,
            M_pickupPower = STOP,
            M_liftPower   = STOP,
            M_hangPowerR  = STOP,
            M_hangPowerL  = STOP;

    // servo positions
    float   S_climbersPosR  = S_CLIMBERS_START_POS_R,
            S_climbersPosL  = S_CLIMBERS_START_POS_L,
            S_liftPosR      = S_LIFT_START_POS_R,
            S_liftPosL      = S_LIFT_START_POS_L,
            S_basketPosR    = S_BASKET_START_POS_R,
            S_basketPosL    = S_BASKET_START_POS_L,
            S_pickupPosFR   = S_PICKUP_START_POS_FR,
            S_pickupPosFL   = S_PICKUP_START_POS_FL,
            S_pickupPosSR   = S_PICKUP_START_POS_SR,
            S_pickupPosSL   = S_PICKUP_START_POS_SL,
            S_hitchPosR     = S_HITCH_START_POS_R,
            S_hitchPosL     = S_HITCH_START_POS_L;

    // thread stuff
    ControllerThread R_controllerThread;
    Thread T_controllerThread;


    @Override public void main() throws InterruptedException {
        /* Initialize our hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names you assigned during the robot configuration
         * step you did in the FTC Robot Controller app on the phone.
         */

        // mapping motor variables to their hardware counterparts
        this.M_driveFR  = this.hardwareMap.dcMotor.get("M_driveFR");
        this.M_driveFL  = this.hardwareMap.dcMotor.get("M_driveFL");
        this.M_driveBR  = this.hardwareMap.dcMotor.get("M_driveBR");
        this.M_driveBL  = this.hardwareMap.dcMotor.get("M_driveBL");
        this.M_pickup   = this.hardwareMap.dcMotor.get("M_pickup");
        this.M_lift     = this.hardwareMap.dcMotor.get("M_lift");
        //this.M_hangR    = this.hardwareMap.dcMotor.get("M_hangR");
        //this.M_hangL    = this.hardwareMap.dcMotor.get("M_hangL");

        // mapping servo variables to their hardware counterparts
        //this.S_climbersR    = this.hardwareMap.servo.get("S_climbersR");
        this.S_climbersL    = this.hardwareMap.servo.get("S_climbersL");
        //this.S_liftR        = this.hardwareMap.servo.get("S_liftR");
        //this.S_liftL        = this.hardwareMap.servo.get("S_liftL");
        //this.S_basketR      = this.hardwareMap.servo.get("S_basketR");
        //this.S_basketL      = this.hardwareMap.servo.get("S_basketL");
        //this.S_pickupFR     = this.hardwareMap.servo.get("S_pickupFR");
        //this.S_pickupFL     = this.hardwareMap.servo.get("S_pickupFL");
        //this.S_pickupSR     = this.hardwareMap.servo.get("S_pickupSR");
        //this.S_pickupSL     = this.hardwareMap.servo.get("S_pickupSL");
        //this.S_hitchR       = this.hardwareMap.servo.get("S_hitchR");
        //this.S_hitchL       = this.hardwareMap.servo.get("S_hitchL");



        // mapping sensor variables to their hardware counterparts
        parameters.angleunit      = IBNO055IMU.ANGLEUNIT.DEGREES;
        parameters.accelunit      = IBNO055IMU.ACCELUNIT.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        this.adaFruit = ClassFactory.createAdaFruitBNO055IMU(hardwareMap.i2cDevice.get("adaFruit"), parameters);

        R_controllerThread = new ControllerThread();
        T_controllerThread = new Thread(R_controllerThread, "Controller Tread");

        // Wait for the game to start
        waitForStart();

        // Go go gadget robot!
        while (opModeIsActive()) {
            if (updateGamepads()) {
                // The game pad state has changed. Do something with that!

                // updates all the motor powers
                this.M_driveFR.setPower(this.M_drivePowerR);
                this.M_driveFL.setPower(this.M_drivePowerL);
                this.M_driveBR.setPower(this.M_drivePowerR);
                this.M_driveBL.setPower(this.M_drivePowerL);
                this.M_pickup.setPower(this.M_pickupPower);
                this.M_lift.setPower(this.M_liftPower);
                //this.M_hangR.setPower(this.M_hangPowerR);
                //this.M_hangL.setPower(this.M_hangPowerL);

                // updates all the servo positions
                //this.S_climbersR.setPosition(this.S_CLIMBERS_START_POS_R);
                this.S_climbersL.setPosition(this.S_CLIMBERS_START_POS_L);
                //this.S_liftR.setPosition(this.S_liftPosR);
                //this.S_liftL.setPosition(this.S_liftPosL);
                //this.S_basketR.setPosition(this.S_basketPosR);
                //this.S_basketL.setPosition(this.S_basketPosL);
                //this.S_pickupFR.setPosition(this.S_pickupPosFR);
                //this.S_pickupFL.setPosition(this.S_pickupPosFL);
                //this.S_pickupSR.setPosition(this.S_pickupPosSR);
                //this.S_pickupSL.setPosition(this.S_pickupPosSL);
                //this.S_hitchR.setPosition(this.S_hitchPosR);
                //this.S_hitchL.setPosition(this.S_hitchPosL);
            }
            telemetry.update();
            idle();
        }
        T_controllerThread.interrupt();
    }
    private class ControllerThread implements Runnable {
        private final float C_STICK_TOP_THRESHOLD = 0.85f;      // least value for which stick value read from motor will be 1.0f

        // converts all of the controller sticks into more sensitive values
        // use a negative value for y axis since controller reads -1 when pushed forward
        private float convertStick(float controllerValue) {   return (float) Range.clip(Math.sin(controllerValue * Math.PI / 2 / C_STICK_TOP_THRESHOLD), -1.0d, 1.0d); }
        // the main loop function
        public void run() {
            try {
                while(!Thread.currentThread().isInterrupted()) {

                    // motor control block
                    M_drivePowerR = convertStick(-gamepad1.right_stick_y);
                    M_drivePowerL = convertStick(-gamepad1.left_stick_y);

                    // pickup control block
                    if (gamepad1.right_bumper) {
                        M_pickupPower = PICKUP_POWER;
                    } else if (gamepad1.left_bumper) {
                        M_pickupPower = -PICKUP_POWER;
                    } else {
                        M_pickupPower = STOP;
                    }

                    // lift control block
                    if(gamepad1.right_trigger > 0.0f) {
                        M_liftPower = LIFT_POWER;
                    } else if(gamepad1.left_trigger > 0.0f) {
                        M_liftPower = -LIFT_POWER;
                    } else {
                        M_liftPower = STOP;
                    }

                    Thread.sleep(10);
                }
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                return;
            }
        }

    }
}