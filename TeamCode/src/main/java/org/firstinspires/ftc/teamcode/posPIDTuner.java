package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Services.positionPIDService;


@Config
@TeleOp(name="posPIDTuner")
public class posPIDTuner extends LinearOpMode {

    private DcMotorEx POSmotorExample;

    private positionPIDService PIDservice = new positionPIDService();
    private double amount = 1;
    private int motorTargetPos=0;

    @Override
    public void runOpMode() {
        POSmotorExample = hardwareMap.get(DcMotorEx.class, "RPMmotorExample");
        POSmotorExample.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        POSmotorExample.setTargetPosition(0);
        POSmotorExample.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        while (opModeIsActive()) { // Loop
            if(gamepad1.dpadUpWasPressed()){
                PIDservice.setKP(PIDservice.getKP() + amount);
            }
            if(gamepad1.dpadDownWasPressed()){
                PIDservice.setKP(PIDservice.getKP() - amount);
            }
            if(gamepad1.dpadRightWasPressed()){
                PIDservice.setKF(PIDservice.getKF() + amount);
            }
            if(gamepad1.dpadLeftWasPressed()){
                PIDservice.setKF(PIDservice.getKF() - amount);
            }
            if(gamepad1.right_stick_button){
                PIDservice.setKD(PIDservice.getKD() + amount);
            }
            if(gamepad1.left_stick_button){
                PIDservice.setKD(PIDservice.getKD() - amount);
            }
            if(gamepad1.right_trigger>.8){
                PIDservice.setKI(PIDservice.getKI() + amount);
            }
            if(gamepad1.left_trigger>.8){
                PIDservice.setKI(PIDservice.getKI() - amount);
            }
            if(gamepad1.rightBumperWasPressed()){
                motorTargetPos+=amount;
            }
            if(gamepad1.leftBumperWasPressed()){
                motorTargetPos-=amount;
            }
            if(gamepad1.bWasPressed()){
                amount*=10;
            }
            if(gamepad1.aWasPressed()){
                amount/=10;
            }
            POSmotorExample.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, PIDservice.getFlywheelCoefficents());
            POSmotorExample.setTargetPosition(motorTargetPos);



            // --------------------------- TELEMETRY --------------------------- //

            telemetry.addData("kp", PIDservice.getKP());
            telemetry.addData("ki", PIDservice.getKI());
            telemetry.addData("kd", PIDservice.getKD());
            telemetry.addData("kf", PIDservice.getKF());
            telemetry.addData("amount", amount);//distanceToTarget
            telemetry.addData("Target Position", motorTargetPos);//distanceToTarget
            telemetry.addData("Actual Position", POSmotorExample.getCurrentPosition());
            telemetry.update();
        }
    }}
