package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;


@Config
@TeleOp(name="testing op")
public class TestingOP extends LinearOpMode {
    private Servo A = null;
    private Servo B = null;

    private Servo C = null;
    private Servo R = null;
    private Servo claw = null;


    //all messurments in inches
    private double originX = 0;
    private double originY = 3; // in inches
    private double originZ = 0;
    private double netLength = 0;//net length

    private double lengthB = 5;//in inches

    private double lengthC = 3.25;//in inches need to check cad for more accurate value


    private double angleB = 0;

    private double angleC = 0;

    private double angleA = 0;

    private double Theta = 0;

    private double angleR = 0;


    private double targetX = 0;

    private double targetY = 0;

    private double targetZ = 0;
    private double clawAngle;
    private boolean toggle = false;
    private double servoAInput;
    private double servoBInput;
    private double servoCInput;

    private boolean Amove = false;

    private boolean Bmove = false;

    private boolean Cmove = false;


    @Override
    public void runOpMode() {
        A = hardwareMap.get(Servo.class,"A");
        B = hardwareMap.get(Servo.class,"B");
        C = hardwareMap.get(Servo.class,"C");
        R = hardwareMap.get(Servo.class,"R");
        claw = hardwareMap.get(Servo.class,"Claw");
        A.setDirection(Servo.Direction.REVERSE);
        B.setDirection(Servo.Direction.REVERSE);
        C.setDirection(Servo.Direction.FORWARD);
        R.setDirection(Servo.Direction.REVERSE);
        claw.setDirection(Servo.Direction.FORWARD);
        R.setPosition(Math.toRadians(90)/Math.PI -.25);
        claw.setPosition(0);
        A.setPosition(.5);
        B.setPosition(.5);
        C.setPosition(0);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        Thread threadA = new Thread(this::ThreadA);
        Thread threadB = new Thread(this::ThreadB);
        Thread threadC = new Thread(this::ThreadC);
        waitForStart();
        threadA.start();
        threadB.start();
        threadC.start();
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
            if(gamepad1.yWasPressed()){
                claw.setPosition(1);
            }
            if(gamepad1.bWasPressed()){
                claw.setPosition(0);
            }
            telemetry.addData("targetX", targetX);
            telemetry.addData("targetY", targetY);
            telemetry.addData("targetZ", targetZ);
            telemetry.addData("angleA", A.getPosition());
            telemetry.addData("angleB", B.getPosition());
            telemetry.addData("angleC", C.getPosition());
            telemetry.addData("angleR", R.getPosition());
            telemetry.addData("claw", claw.getPosition());
            telemetry.addData("AngleA", Math.toDegrees(angleA));
            telemetry.addData("AngleB", Math.toDegrees(angleB+Theta));
            telemetry.addData("Claw angle", Math.toDegrees(clawAngle));
            telemetry.update();
        }

    }
    public void Solve(double endAngle){
        double dx = targetX - originX;
        double dy = targetY - originY;
        double dz = targetZ - originZ;
        double groundDist = Math.sqrt(dx * dx + dz * dz);

        netLength = Math.sqrt(Math.pow(groundDist,2) + Math.pow(dy,2));//gives net distance from origin to target point
        angleA = Math.acos((Math.pow(netLength,2)-Math.pow(lengthB,2)-Math.pow(lengthC,2))/(-2*lengthB*lengthC));//law of cosines
        angleB = Math.asin((lengthC*Math.sin(angleA))/ netLength);//law of sines
        Theta = Math.atan2(dy, groundDist);
        angleR = Math.atan2(dx, dz);
        double forearmAngle = angleB - (Math.PI - angleA);
        clawAngle = Math.toRadians(endAngle) - forearmAngle;
        R.setPosition(angleR/Math.PI -.2);
        servoAInput = (angleA/Math.PI);
        servoBInput = (angleB+Theta)/Math.toRadians(270)  - .25;
        servoCInput = clawAngle / Math.PI;
        Amove = true;
        Bmove = true;
        Cmove = true;
        /*
        B.setPosition((angleB+Theta)/Math.toRadians(270)  - .02);
        sleep(1000);
        C.setPosition(clawAngle / Math.PI);
        sleep(500);
        A.setPosition(angleA/Math.PI - .52);//puts it into 0 to 1 to mirror realife will needed to be changed based on mounting

         */
    }
    public void ThreadA(){
        while(opModeIsActive()) {
            if (Amove) {
                while(!(A.getPosition()<servoAInput+.01 && A.getPosition()>servoAInput-.01)){
                    if(A.getPosition() < servoAInput){
                        A.setPosition(A.getPosition() + .01);
                    }
                    else{
                        A.setPosition(A.getPosition() - .01);
                    }
                    sleep(50);
                }
                Amove = false;
            }
            sleep(50);
        }
    }
    public void ThreadB(){
        while(opModeIsActive()) {
            if (Bmove) {
                while(!(B.getPosition()<servoBInput+.01 && B.getPosition()>servoBInput-.01)){
                    if(B.getPosition() < servoBInput){
                        B.setPosition(B.getPosition() + .01);
                    }
                    else{
                        B.setPosition(B.getPosition() - .01);
                    }
                    sleep(50);
                }
                Bmove = false;
            }
            sleep(50);
        }

    }
    public void ThreadC(){
        while(opModeIsActive()) {
            if (Cmove) {
                while(!(C.getPosition()<servoCInput+.01 && C.getPosition()>servoCInput-.01)){
                    if(C.getPosition() < servoCInput){
                        C.setPosition(C.getPosition() + .01);
                    }
                    else{
                        C.setPosition(C.getPosition() - .01);
                    }
                    sleep(50);
                }
                Cmove = false;
            }
            sleep(50);
        }

    }



}
