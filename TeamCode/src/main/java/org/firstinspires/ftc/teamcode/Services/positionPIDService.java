package org.firstinspires.ftc.teamcode.Services;

import com.qualcomm.robotcore.hardware.PIDFCoefficients;
//this service is meant for storing flywheel pids for the ftc decode game will impliment a general pid solution.
public class positionPIDService {
    //after tuning make sure to transfer the kp values to the final variable
    private double KP = 0;

    private final double FinalKP = 0;
    private final double KI = 0;
    private double KD = 0;
    private double KF = 0;
    private final double FinalKF = 0;


    public PIDFCoefficients getFlywheelCoefficents(){
        PIDFCoefficients flyhwheelconts = new PIDFCoefficients(FinalKP, KI, KD,FinalKF);
        return flyhwheelconts;
    }
    public void setKP(double kp){
        KP = kp;

    }
    public void setKI(double ki){
        KP = ki;

    }
    public void setKD(double kd){
        KP = kd;

    }
    public void setKF(double kf){
        KF = kf;

    }
    public double getKP() {
        return KP;
    }

    public double getKI() {
        return KI;
    }

    public double getKF() {
        return KF;
    }

    public double getKD() {
        return KD;
    }
    public double getFinalKF() {
        return FinalKF;
    }
    public double getFinalKP() {
        return FinalKP;
    }
}
