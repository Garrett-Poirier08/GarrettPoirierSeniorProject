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
    private Servo b = null;
    private Servo a = null;

    private Servo C = null;
    private Servo R = null;
    private Servo claw = null;


    //all messurments in inches
    private double originX = 0;
    private double originY = 2.876; // in inches
    private double originZ = 0;
    private double netLength = 0;//net length

    private double lengthC = 5;//in inches

    private double lengthA = 3.343;//in inches


    private double angleA = 0;

    private double angleB = 0;

    private double Theta = 0;

    private double angleR = 0;


    private double targetX = 0;

    private double targetY = 0;

    private double targetZ = 0;
    private double clawAngle;
    private boolean toggle = false;
    private double servoBInput;
    private double servoAInput;
    private double servoCInput;

    private boolean Amove = false;

    private boolean Bmove = false;

    private boolean Cmove = false;
    private double clawOffset = 2;


    @Override
    public void runOpMode() {
        b = hardwareMap.get(Servo.class,"A");
        a = hardwareMap.get(Servo.class,"B");
        C = hardwareMap.get(Servo.class,"C");
        R = hardwareMap.get(Servo.class,"R");
        claw = hardwareMap.get(Servo.class,"Claw");
        b.setDirection(Servo.Direction.REVERSE);
        a.setDirection(Servo.Direction.REVERSE);
        C.setDirection(Servo.Direction.REVERSE);
        R.setDirection(Servo.Direction.REVERSE);
        claw.setDirection(Servo.Direction.FORWARD);
        R.setPosition(Math.toRadians(90)/Math.PI -.25);
        claw.setPosition(0);
        b.setPosition(.5);
        a.setPosition(.5);
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
                Solve(-90,3);
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
            telemetry.addData("angleA", b.getPosition());
            telemetry.addData("angleB", a.getPosition());
            telemetry.addData("angleC", C.getPosition());
            telemetry.addData("angleR", R.getPosition());
            telemetry.addData("claw", claw.getPosition());
            telemetry.addData("AngleB", Math.toDegrees(angleB));
            telemetry.addData("AngleA", Math.toDegrees(angleA));
            telemetry.addData("net AngleA", Math.toDegrees(angleA +Theta));
            telemetry.addData("Claw angle", Math.toDegrees(clawAngle));
            telemetry.update();
        }

    }
    public void Solve(double endAngle, double lengthClaw) { // Added lengthClaw
        double dx = targetX - originX;
        double dy = targetY - originY;
        double dz = targetZ - originZ;

        // 1. Calculate base rotation
        angleR = Math.atan2(dx, dz);

        // 2. Initial ground distance to TARGET
        double targetGroundDist = Math.sqrt(dx * dx + dz * dz);
        double endAngleRad = Math.toRadians(endAngle);

        // 3. WRIST DECOUPLING: Step backward from target to find the wrist joint center
        double wristGroundDist = targetGroundDist - (lengthClaw * Math.cos(endAngleRad));
        double wristY = dy - (lengthClaw * Math.sin(endAngleRad)); // Using dy relative to origin

        // 4. Calculate IK based on the WRIST position, not the target position
        netLength = Math.sqrt(Math.pow(wristGroundDist, 2) + Math.pow(wristY, 2));

        // prevents NaN errors
        if (netLength > (lengthA + lengthC)) {
            netLength = lengthA + lengthC - 0.001;
        }

        // Law of Cosines
        angleB = Math.acos((Math.pow(netLength, 2) - Math.pow(lengthC, 2) - Math.pow(lengthA, 2)) / (-2 * lengthC * lengthA));
        angleA = Math.acos((Math.pow(lengthA, 2) - Math.pow(lengthC, 2) - Math.pow(netLength, 2)) / (-2 * lengthC * netLength));

        // Angle to the wrist from horizontal
        Theta = Math.atan2(wristY, wristGroundDist);

        // 5. Correct absolute forearm angle (subtracting PI)
        double forearmAngleAbs = (Theta + angleA + angleB) - Math.PI;

        // 6. Calculate required claw joint angle to maintain the desired endAngle
        // Wrist Joint Angle = Desired Absolute Angle - Current Forearm Absolute Angle
        clawAngle = endAngleRad - forearmAngleAbs;

        // --- Servo Mappings ---
        R.setPosition(angleR / Math.PI - 0.2);

        servoBInput = (angleB / Math.PI);
        servoAInput = (Theta + angleA) / Math.toRadians(270); // Keeping your arbitrary mounting numbers

        // You may need to tweak the sign or offset of servoCInput depending on how your wrist servo is physically mounted
        servoCInput = Math.abs(clawAngle)/ Math.toRadians(190); // +0.5 assumes 90 degrees is the straight/neutral position

        Amove = true;
        Bmove = true;
        Cmove = true;
    }
    public void ThreadA(){
        while(opModeIsActive()) {
            if (Amove) {
                while(!(b.getPosition()< servoBInput +.01 && b.getPosition()> servoBInput -.01)){
                    if(b.getPosition() < servoBInput){
                        b.setPosition(b.getPosition() + .01);
                    }
                    else{
                        b.setPosition(b.getPosition() - .01);
                    }
                    sleep(50);
                }
                b.setPosition(servoBInput);
                Amove = false;
            }
            sleep(50);
        }
    }
    public void ThreadB(){
        while(opModeIsActive()) {
            if (Bmove) {
                while(!(a.getPosition()< servoAInput +.01 && a.getPosition()> servoAInput -.01)){
                    if(a.getPosition() < servoAInput){
                        a.setPosition(a.getPosition() + .01);
                    }
                    else{
                        a.setPosition(a.getPosition() - .01);
                    }
                    sleep(50);
                }
                a.setPosition(servoAInput);
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
                C.setPosition(servoCInput);
                Cmove = false;
            }
            sleep(50);
        }

    }
}
