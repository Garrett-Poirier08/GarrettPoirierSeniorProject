package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Services.FlywheelPIDService;


@Config
@TeleOp(name="rpmPIDTuner")
public class rpmPIDTuner extends LinearOpMode {

    private DcMotorEx RPMmotorExample;

    private FlywheelPIDService PIDservice = new FlywheelPIDService();

    //turre
    private double amount = 1;
    private double flyWheelTargetRPM=1000;

    @Override
    public void runOpMode() {
        RPMmotorExample = hardwareMap.get(DcMotorEx.class, "RPMmotorExample");
        RPMmotorExample.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        while (opModeIsActive()) { // Loop
            if(gamepad1.dpadUpWasPressed()){
                PIDservice.setFlyhweelKP(PIDservice.getFlyhweelKP() + amount);
            }
            if(gamepad1.dpadDownWasPressed()){
                PIDservice.setFlyhweelKP(PIDservice.getFlyhweelKP() - amount);
            }
            if(gamepad1.dpadRightWasPressed()){
                PIDservice.setFlyhweelKF(PIDservice.getFlywheeKF() + amount);
            }
            if(gamepad1.dpadLeftWasPressed()){
                PIDservice.setFlyhweelKF(PIDservice.getFlywheeKF() - amount);
            }
            if(gamepad1.rightBumperWasPressed()){
                flyWheelTargetRPM+=amount;
            }
            if(gamepad1.leftBumperWasPressed()){
                flyWheelTargetRPM-=amount;
            }
            if(gamepad1.bWasPressed()){
                amount*=10;
            }
            if(gamepad1.aWasPressed()){
                amount/=10;
            }
            RPMmotorExample.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, PIDservice.getFlywheelCoefficents());
            RPMmotorExample.setVelocity(flyWheelTargetRPM);



            // --------------------------- TELEMETRY --------------------------- //
            //you can add ki and kd but with velcoity control ki and kd is not required/over complicates controls
            telemetry.addData("kp", PIDservice.getFlyhweelKP());
            telemetry.addData("kf", PIDservice.getFlywheeKF());
            telemetry.addData("amount", amount);//distanceToTarget
            telemetry.addData("Flywheel Target Velocity", flyWheelTargetRPM);//distanceToTarget
            telemetry.addData("Flywheel L Velocity", RPMmotorExample.getVelocity());
            telemetry.update();
        }
    }}
