package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;


@Config
@TeleOp(name="IK 3 dof 3 joints")
public class IKThreeDegreesOfFreedom3Joints extends LinearOpMode {
    private Servo A = null;
    private Servo B = null;

    private Servo C = null;
    private Servo R = null;


    //all messurments in inches
    private double originX = 0;
    private double originY = 0;
    private double originZ = 0;
    private double netLength = 0;//net length

    private double lengthB = 0;

    private double lengthC = 0;
    private double lengthD = 0;


    private double angleB = 0;

    private double angleC = 0;

    private double angleA = 0;

    private double Theta = 0;

    private double angleR = 0;


    private double targetX = 0;

    private double targetY = 0;

    private double targetZ = 0;
    private double clawAngle;


    @Override
    public void runOpMode() {
        A = hardwareMap.get(Servo.class,"A");
        B = hardwareMap.get(Servo.class,"B");
        C = hardwareMap.get(Servo.class,"C");
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
                Solve(-90);
            }
        }
    }
    public void Solve(double endAngle){
        netLength = Math.sqrt((Math.pow((originX-targetX),2) + Math.pow((originY-targetY),2))+Math.pow(originZ-targetZ,2));//gives net distance from origin to target point
        angleA = Math.acos((Math.pow(netLength,2)-Math.pow(lengthB,2)-Math.pow(lengthC,2))/(-2*lengthB*lengthC));//law of cosines
        angleB = Math.asin((lengthC*Math.sin(angleA))/ netLength);//law of sines
        Theta = Math.atan2(targetY-originY,Math.hypot(targetX-originX, targetZ-originZ));
        angleR = Math.atan2(targetZ-originZ,targetX-originX);
        double forearmAngle = angleB - (Math.PI - angleA);
        clawAngle = Math.toRadians(endAngle) - forearmAngle;
        A.setPosition(angleA/Math.PI);//puts it into 0 to 1 to mirror realife will needed to be changed based on mounting
        B.setPosition(((angleB+Theta)/Math.PI));
        R.setPosition(angleR/Math.PI + 0.5);
        C.setPosition(clawAngle / Math.PI + 0.5);

    }




}
