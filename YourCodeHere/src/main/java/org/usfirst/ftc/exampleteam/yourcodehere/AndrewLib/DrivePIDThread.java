package org.usfirst.ftc.exampleteam.yourcodehere.AndrewLib;

import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by hoangam on 11/19/2015.
 */
public class DrivePIDThread extends PIDThread {

    DrivePIDThread(double kP, double kI, double kD, DcMotor[] motors) {
        double[] kPs = new double[motors.length];
        double[] kIs = new double[motors.length];
        double[] kDs = new double[motors.length];
        super(kPs, kIs, kDs, motors);
    }
    //////// need to do!!!!!!!!!!!!!!!!! ///////////////////
    // convert ticks to inches
    public void setTarget(double target) {
        this.target = target;
        for(int i = 0; i < motorTargets.length; i++) {
            motorTargets[i] = (int)(motors[i].getCurrentPosition() + target);
        }
    }
}
