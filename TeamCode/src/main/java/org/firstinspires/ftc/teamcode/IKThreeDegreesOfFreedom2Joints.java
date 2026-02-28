package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;


@Config
@TeleOp(name="IK 3 dof 2 joints")
public class IKThreeDegreesOfFreedom2Joints extends LinearOpMode {
    private Servo A = null;
    private Servo B = null;
    private Servo R = null;


    private double lengthA = 0;

    private double lengthB = 0;

    private double lengthC = 0;


    private double angleB = 0;

    private double angleC = 0;

    private double angleA = 0;

    private double Theta = 0;

    private double angleR = 0;


    private double targetX = 0;

    private double targetY = 0;

    private double targetZ = 0;



    @Override
    public void runOpMode() {
        A = hardwareMap.get(Servo.class,"A");
        B = hardwareMap.get(Servo.class,"B");
        R = hardwareMap.get(Servo.class,"R");
        A.setPosition(0.5);
        B.setPosition(0.5);
        R.setPosition(0.5);
        waitForStart();
        while (opModeIsActive()) {
            if(gamepad1.dpadUpWasPressed()){
                targetY+=0.5;
            }
            if(gamepad1.dpadDownWasPressed()){
                targetY-=0.5;
            }
            if(gamepad1.dpadLeftWasPressed()){
                targetX-=0.5;
            }
            if(gamepad1.dpadRightWasPressed()){
                targetX+=0.5;
            }
            if(gamepad1.rightBumperWasPressed()){
                targetZ+=0.5;
            }
            if(gamepad1.leftBumperWasPressed()){
                targetZ-=0.5;
            }
            if(gamepad1.aWasPressed()){
                Solve();
            }
        }
    }
    public void Solve(){
        lengthA = Math.sqrt((Math.pow((targetX),2) + Math.pow((targetY),2)));//gives net distance from origin to target point
        angleA = Math.acos((Math.pow(lengthA,2)-Math.pow(lengthB,2)-Math.pow(lengthC,2))/(-2*lengthB*lengthC));//law of cosines
        angleB = Math.asin((lengthB*Math.sin(angleA))/lengthA);//law of sines
        Theta = Math.atan(targetY/targetX);
        angleR = Math.atan(targetZ/targetX);
        A.setPosition(angleA/Math.PI);//puts it into 0 to 1 to mirror realife will needed to be changed based on mounting
        B.setPosition(((angleB+Theta)/Math.PI));
        R.setPosition(angleR/Math.PI);
    }




}
