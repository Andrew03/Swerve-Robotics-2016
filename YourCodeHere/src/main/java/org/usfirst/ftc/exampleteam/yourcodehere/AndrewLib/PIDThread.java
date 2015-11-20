package org.usfirst.ftc.exampleteam.yourcodehere.AndrewLib;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Andrew on 11/18/2015.
 */
public class PIDThread extends Thread implements PIDInterface{
    private double[] kP, kI, kD, PIDValue;
    private int[] accumError;
    protected double    target   = 0.0d;
    protected double    maxPower = 1.0d,
                        minPower = -1.0d;
    protected double    threshold;
    DcMotor motors[];
    int motorTargets[];

    PIDThread(double kP, double kI, double kD, DcMotor[] motors) {
        this.kP = new double[motors.length];
        this.kI = new double[motors.length];
        this.kD = new double[motors.length];
        this.PIDValue = new double[motors.length];
        this.accumError = new int[motors.length];
        this.PIDValue = new double[motors.length];
        this.motors = motors;
        this.motorTargets = new int[motors.length];
        for(int i = 0; i < motors.length; i++) {
            this.kP[i] = kP;
            this.kI[i] = kI;
            this.kD[i] = kD;
            this.accumError[i] = 0;
            this.motorTargets[i] = 0;
        }
    }

    public void setTarget(double target) {
        this.target = target;
        for(int i = 0; i < motorTargets.length; i++) {
            // averages current values and adds target to get new motor targets
            motorTargets[i] = (int)(motors[i].getCurrentPosition() + target);
        }
    }

    private boolean didReachTarget() {
        for(int i = 0; i < motors.length; i++) {
            if(Math.abs(motors[i].getCurrentPosition()) - target > threshold) {
                return false;
            }
        }
        return true;
    }

    public void setMaxPower(double maxPower) {
        this.maxPower = maxPower;
    }
    public void setMinPower(double minPower) {
        this.minPower = minPower;
    }

    @Override
    public void run() {
        try {
            while(!didReachTarget()) {
                for(int i = 0; i < motors.length; i++) {
                    int error = motors[i].getTargetPosition() - motors[i].getCurrentPosition();
                    accumError[i] += error;
                    PIDValue[i] = kP[i] * error + kI[i] * accumError[i];
                    if(Math.abs(PIDValue[i]) > threshold) {
                        motors[i].setPower(Range.clip(PIDValue[i], -1.0d, 1.0d));
                    } else {
                        motors[i].setPower(0.0d);
                    }
                }
                sleep(10);
            }
        } catch(InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}