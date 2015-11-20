package org.usfirst.ftc.exampleteam.yourcodehere.AndrewLib;

import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Andrew on 11/19/2015.
 */
public class TurnPIDThread extends PIDThread {
    final private double DEGREES_TO_TICKS = 1.0d;
    public TurnPIDThread(double kP, double kI, double kD, DcMotor driveFR, DcMotor driveFL, DcMotor driveBR, DcMotor driveBL) {
        super(kP, kI, kD, driveFR, driveFL, driveBR, driveBL);
    }
    //////// need to do!!!!!!!!!!!!!!!!! ///////////////////
    // convert degrees to ticks
    // find average of 5 trials, ticks per 90 inches by driving
    // target is entered in degrees
    public void setTarget(double target) {
        this.target = target * DEGREES_TO_TICKS;
        for(int i = 0; i < 2; i++) {
            motorTargets[2 * i] = (int)(motors[2 * i].getCurrentPosition() + this.target);
            motorTargets[2 * i + 1] = (int)(motors[2 * i + 1].getCurrentPosition() - this.target);
        }
    }
}
