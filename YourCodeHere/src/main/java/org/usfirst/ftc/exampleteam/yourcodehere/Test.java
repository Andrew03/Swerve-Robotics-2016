package org.usfirst.ftc.exampleteam.yourcodehere;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.robocol.Telemetry;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;

import org.swerverobotics.library.SynchronousOpMode;
import org.swerverobotics.library.interfaces.IBNO055IMU;
import org.swerverobotics.library.interfaces.TeleOp;
import org.swerverobotics.library.internal.AdaFruitBNO055IMU;
import org.swerverobotics.library.ClassFactory;

// distance is an arc length, r * angle, where r = diagonal of robot / 2 and angle is angle to turn
// distance also equals ticks * wheel radius / ticks per revolution
// solve for ticks, gets value below
// ticks needed = angle * diagonal of robot * ticks per rev / radius of wheel / 2

// get to this kind of function
/*
    void turn(double angle) {
        int val = angle / some constant
        motorL.runToPosition(val)
        motorR.runToPosition(-val)
    }
 */

// run to position vs run using encoders?


@TeleOp(name="Test Op Mode")
public class Test extends SynchronousOpMode {

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
    Servo   S_climbersKnockdownR    = null, // right servo that knocks down climbers
            S_climbersKnockdownL    = null, // left servo that knocks down climbers
            S_climbersDeposit       = null, // servo that deposits climbers
            S_liftR                 = null, // right servo that supports lift
            S_liftL                 = null, // left servo that supports lift
            S_basketTilt            = null, // right servo on the basket
            S_basketRelease         = null, // left servo on the basket
            S_pickupFL              = null, // front left servo of the pickup
            S_pickupSR              = null, // servo on right side of the pickup
            S_pickupSL              = null, // servo on left side of the pickup
            S_hitchR                = null, // right hitch servo
            S_hitchL                = null; // left hitch servo

    // sensor declarations
    IBNO055IMU adaFruit = null;
    IBNO055IMU.Parameters   parameters = new IBNO055IMU.Parameters();


    // all of the important constants
    final double    STOP                   = 0.0d,
                    MAX_POWER              = 1.0d;
    final int       TICKS_PER_REVOLUTION   = 1120;

    // all of the constant motor powers
    final double    PICKUP_POWER    = 0.8d,
                    LIFT_POWER      = 1.0d;

    // all of the starting/open servo positions
    final double    S_CLIMBERS_KNOCKDOWN_START_POS_R    = Servo.MIN_POSITION,
                    S_CLIMBERS_KNOCKDOWN_START_POS_L    = Servo.MIN_POSITION,
                    S_CLIMBERS_DEPOSIT_START_POS        = 0.90d,
                    S_LIFT_START_POS_R                  = Servo.MIN_POSITION,
                    S_LIFT_START_POS_L                  = Servo.MIN_POSITION,
                    S_BASKET_TILT_START_POS             = Servo.MIN_POSITION,
                    S_BASKET_RELEASE_START_POS          = Servo.MIN_POSITION,
                    S_PICKUP_START_POS_FL               = Servo.MIN_POSITION,
                    S_PICKUP_START_POS_SR               = Servo.MIN_POSITION,
                    S_PICKUP_START_POS_SL               = Servo.MIN_POSITION,
                    S_HITCH_START_POS_R                 = Servo.MIN_POSITION,
                    S_HITCH_START_POS_L                 = Servo.MIN_POSITION;

    // all of the ending/close servo positions
    final double    S_CLIMBERS__KNOCKDOWN_END_POS_R     = Servo.MAX_POSITION,
                    S_CLIMBERS_KNOCKDOWN_END_POS_L      = Servo.MAX_POSITION,
                    S_CLIMBERS_DEPOSIT_END_POS          = Servo.MIN_POSITION,
                    S_LIFT_END_POS_R                    = Servo.MAX_POSITION,
                    S_LIFT_END_POS_L                    = Servo.MAX_POSITION,
                    S_BASKET_TILT_END_POS               = Servo.MAX_POSITION,
                    S_BASKET_RELEASE_END_POS            = Servo.MAX_POSITION,
                    S_PICKUP_END_POS_FR                 = Servo.MAX_POSITION,
                    S_PICKUP_END_POS_FL                 = Servo.MAX_POSITION,
                    S_PICKUP_END_POS_SR                 = Servo.MAX_POSITION,
                    S_PICKUP_END_POS_SL                 = Servo.MAX_POSITION,
                    S_HITCH_END_POS_R                   = Servo.MAX_POSITION,
                    S_HITCH_END_POS_L                   = Servo.MAX_POSITION;

    // motor powers
    double  M_drivePowerR = STOP,
            M_drivePowerL = STOP,
            M_pickupPower = STOP,
            M_liftPower   = STOP,
            M_hangPowerR  = STOP,
            M_hangPowerL  = STOP;

    // servo positions
    double  S_climbersKnockdownPosR  = S_CLIMBERS_KNOCKDOWN_START_POS_R,
            S_climbersKnockdownPosL  = S_CLIMBERS_KNOCKDOWN_START_POS_L,
            S_climbersDepositPos     = S_CLIMBERS_DEPOSIT_START_POS,
            S_liftPosR               = S_LIFT_START_POS_R,
            S_liftPosL               = S_LIFT_START_POS_L,
            S_basketPosTiltPos       = S_BASKET_TILT_START_POS,
            S_basketPosReleasePos    = S_BASKET_RELEASE_START_POS,
            S_pickupPosFL            = S_PICKUP_START_POS_FL,
            S_pickupPosSR            = S_PICKUP_START_POS_SR,
            S_pickupPosSL            = S_PICKUP_START_POS_SL,
            S_hitchPosR              = S_HITCH_START_POS_R,
            S_hitchPosL              = S_HITCH_START_POS_L;

    double angleTarget, currAngle;

    enum Quadrant {
        Q1,
        Q2,
        Q3,
        Q4
    } Quadrant quadrant;

    private final float C_STICK_TOP_THRESHOLD = 0.85f;
    private double convertStick(float controllerValue) {   return Range.clip(Math.sin(controllerValue * Math.PI / 2 / C_STICK_TOP_THRESHOLD), -1.0d, 1.0d); }

    @Override public void main() throws InterruptedException {

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
        //this.S_climbersKnockdownR   = this.hardwareMap.servo.get("S_climbersKnockdownR");
        //this.S_climbersKnockdownL   = this.hardwareMap.servo.get("S_climbersKnockdownL");
        this.S_climbersDeposit      = this.hardwareMap.servo.get("S_climbersDeposit");
        //this.S_liftR                = this.hardwareMap.servo.get("S_liftR");
        //this.S_liftL                = this.hardwareMap.servo.get("S_liftL");
        //this.S_basketTilt           = this.hardwareMap.servo.get("S_basketTilt");
        //this.S_basketRelease        = this.hardwareMap.servo.get("S_basketRelease");
        //this.S_pickupFL             = this.hardwareMap.servo.get("S_pickupFL");
        //this.S_pickupSR             = this.hardwareMap.servo.get("S_pickupSR");
        //this.S_pickupSL             = this.hardwareMap.servo.get("S_pickupSL");
        //this.S_hitchR               = this.hardwareMap.servo.get("S_hitchR");
        //this.S_hitchL               = this.hardwareMap.servo.get("S_hitchL");



        // mapping sensor variables to their hardware counterparts
        parameters.angleunit      = IBNO055IMU.ANGLEUNIT.DEGREES;
        parameters.accelunit      = IBNO055IMU.ACCELUNIT.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        //this.adaFruit = ClassFactory.createAdaFruitBNO055IMU(hardwareMap.i2cDevice.get("adaFruit"), parameters)

        // fixing motor directions
        this.M_driveFR.setDirection(DcMotor.Direction.REVERSE);
        this.M_driveBR.setDirection(DcMotor.Direction.REVERSE);
        this.M_pickup.setDirection(DcMotor.Direction.REVERSE);
        this.M_lift.setDirection(DcMotor.Direction.REVERSE);
        //this.M_hangL.setDirection(DcMotor.Direction.REVERSE);



        // resets all the encoder values
        //this.M_driveFR.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        //this.M_driveFL.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        this.M_driveBR.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        this.M_driveBL.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        //this.M_pickup.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        //this.M_lift.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        //this.M_hangR.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        //this.M_hangL.setMode(DcMotorController.RunMode.RESET_ENCODERS);

        //this.M_driveBR.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        //this.M_driveBL.setMode(DcMotorController.RunMode.RUN_TO_POSITION);

        this.M_driveBR.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        this.M_driveBL.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);


        // Wait for the game to start
        waitForStart();
        //T_controllerThread.start();
        while (opModeIsActive()) {
            if (updateGamepads()) {
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

                // climber control block
                /*
                if(gamepad1.a) {
                    S_climbersKnockdownPosL = S_CLIMBERS_KNOCKDOWN_END_POS_L;
                    S_climbersKnockdownPosR = S_CLIMBERS__KNOCKDOWN_END_POS_R;
                } else if(gamepad1.b) {
                    S_climbersKnockdownPosL = S_CLIMBERS_KNOCKDOWN_START_POS_L;
                    S_climbersKnockdownPosR = S_CLIMBERS_KNOCKDOWN_START_POS_R;
                }
                */
                /*
                if(gamepad1.x) {
                    M_drivePowerR = 0.35f;
                    M_drivePowerL = -0.35f;
                } else {
                    M_drivePowerR = 0.0f;
                    M_drivePowerL = 0.0f;
                }
                */
                if(gamepad1.y) {
                    S_climbersDepositPos = S_CLIMBERS_DEPOSIT_END_POS;
                } else if(gamepad1.x) {
                    S_climbersDepositPos = S_CLIMBERS_DEPOSIT_START_POS;
                }
                /*
                if(gamepad1.dpad_up) {
                    this.M_driveBR.setMode(DcMotorController.RunMode.RESET_ENCODERS);
                    this.M_driveBL.setMode(DcMotorController.RunMode.RESET_ENCODERS);
                }
                if(gamepad1.dpad_right) {
                    this.M_driveBR.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
                    this.M_driveBL.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
                }
                if(gamepad1.dpad_down) {
                    this.M_driveBL.setTargetPosition(-2400);
                    this.M_driveBR.setTargetPosition(-2400);
                }
                if(gamepad1.dpad_left) {
                    this.M_driveBL.setTargetPosition(2400);
                    this.M_driveBR.setTargetPosition(2400);
                }
                */
            }

            //this.M_driveBL.setTargetPosition(2400);
            //this.M_driveBR.setTargetPosition(2400);

            /*
            if(this.M_driveBL.getCurrentPosition() > this.M_driveBL.getTargetPosition()) {
                this.M_drivePowerL = STOP;
            }
            if(this.M_driveBR.getCurrentPosition() > this.M_driveBR.getTargetPosition()) {
                this.M_drivePowerR = STOP;
            }
            */
            //this.M_driveBL.setPower(0.5f);

            // updates all the motor powers
            this.M_driveBR.setPower(this.M_drivePowerR);
            this.M_driveBL.setPower(this.M_drivePowerL);
            this.M_driveFR.setPower(this.M_drivePowerR);
            this.M_driveFL.setPower(this.M_drivePowerL);
            this.M_pickup.setPower(this.M_pickupPower);
            this.M_lift.setPower(this.M_liftPower);
            //this.M_hangR.setPower(this.M_hangPowerR);
            //this.M_hangL.setPower(this.M_hangPowerL);

            // updates all the servo positions
            //this.S_climbersKnockdownR.setPosition(this.S_climbersKnockdownPosR);
            //this.S_climbersKnockdownL.setPosition(this.S_climbersKnockdownPosL);
            this.S_climbersDeposit.setPosition(this.S_climbersDepositPos);
            //this.S_liftR.setPosition(this.S_liftPosR);
            //this.S_liftL.setPosition(this.S_liftPosL);
            //this.S_basketTilt.setPosition(this.S_basketPosTiltPos);
            //this.S_basketRelease.setPosition(this.S_basketPosReleasePos);
            //this.S_pickupFL.setPosition(this.S_pickupPosFL);
            //this.S_pickupSR.setPosition(this.S_pickupPosSR);
            //this.S_pickupSL.setPosition(this.S_pickupPosSL);
            //this.S_hitchR.setPosition(this.S_hitchPosR);
            //this.S_hitchL.setPosition(this.S_hitchPosL);

            //int currPos = M_driveBL.getCurrentPosition();

            telemetry.addData("R Motor Pos", M_driveBR.getCurrentPosition());
            telemetry.addData("L Motor Pos", M_driveBL.getCurrentPosition());
            telemetry.addData("R Motor Power", M_drivePowerR);
            telemetry.addData("L Motor Power", M_drivePowerL);

            telemetry.update();
            idle();
        }
    }

    private class TurnPIDThread implements Runnable {
        final double angleThreshold = 0.5d;
        float   kP = 0.0f,
                kI = 0.0f;
        float   maxPower = 1.0f,
                minPower = -1.0f,
                minPIDPower = 0.2f;
        boolean isMoving = false;
        double  currError = 0.0d,   // in degrees
                accumError = 0.0d;  // in degrees
        float   PIDValue = 0.0f,
                power = 0.0f;
        double prevTarget = 0.0d;

        @Override
        public void run() {
            try {
                while (!Thread.currentThread().isInterrupted()) {
                    if (Math.abs(prevTarget - angleTarget) > angleThreshold) {
                        prevTarget = angleTarget;
                        accumError = 0.0d;
                    }
////////////////////////////// find a way to convert degrees to 0-180 from y axis
                    if (Math.abs(angleTarget - currAngle) > angleThreshold) {
                        currError = (Math.abs(angleTarget - currAngle) > Math.abs(currAngle - angleTarget)) ? angleTarget - currAngle : currAngle - angleTarget;
                        accumError += currError;
                        PIDValue = (float) (kP * currError + kI * accumError);
                        PIDValue = Range.clip(PIDValue, -maxPower, maxPower);
                        //driveRPower = PIDValue;
                        if (Math.abs(currError) < angleThreshold) {
                            power = 0.0f;
                            isMoving = false;
                        }
                        Thread.sleep(10);
                    }
                }
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                return;
            }
        }
    }
}

