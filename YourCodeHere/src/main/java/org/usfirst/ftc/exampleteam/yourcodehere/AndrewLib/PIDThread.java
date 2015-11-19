package org.usfirst.ftc.exampleteam.yourcodehere.AndrewLib;

import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Andrew on 11/18/2015.
 */
public class PIDThread extends Thread implements PIDInterface{
    private double kP, kI, kD;
    private double target;
    private double maxPower, minPower;
    private boolean didReachTarget;
    DcMotor motors[];
    int motorTargets[];
    // enter in same motor twice if only using one
    PIDThread(double kP, double kI, double kD, DcMotor ... motors) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.motors = new DcMotor[motors.length];
        for(int i = 0; i < motors.length; i++) {
            this.motors[i] = motors[i];
        }
        this.motorTargets = new int[(int)Math.ceil(this.motors.length / 2)];
    }
    public void setTarget(double target) {
        this.target = target;
        for(int i = 0; i < motorTargets.length; i++) {
            // averages current values and adds target to get new motor targets
            motorTargets[i] = (int)((motors[2 * i].getCurrentPosition() + motors[2 * i + 1].getCurrentPosition()) / 2 + target);
        }
        didReachTarget = false;
    }
    public void setMaxPower(double maxPower) {
        this.maxPower = maxPower;
    }
    public void setMinPower(double minPower) {
        this.minPower = minPower;
    }
    private boolean isDidReachTarget() {
        return true;
    }

    @Override
    public void run() {
        try {
            while(!didReachTarget) {
                for(DcMotor motor : motors) {

                }
                sleep(10);
            }
        } catch(InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}