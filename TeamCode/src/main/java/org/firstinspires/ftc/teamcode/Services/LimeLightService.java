package org.firstinspires.ftc.teamcode.Services;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
//this service is used to collect usable data from a limlight 3a which as of 2025-2026 decode is the most advance camera that is legal for ftc
public class LimeLightService {
    private Limelight3A limelight= null;


    private double tx;// How far left or right the target is (degrees)

    private double ty;// How far up or down the target is (degrees)

    private double ta; // How big the target looks (0%-100% of the image)


    private double x;//x position of the field(left and right)


    private double y;//y position of the field(up and down)


    private double heading;//heading of the robot is the

    public void initLimeLight(HardwareMap hw){
        limelight = hw.get(Limelight3A.class,"limeLight");
        limelight.setPollRateHz(90);//how fast the limlight collects data per second
        limelight.start();
    }
    //used for 3d localization most accurate should be used in conjuction with the odomitry pods and overide position only when odomity drifts.
    //look at limlight documentation for use case examples.
    //call this meathod in the main loop or preferably in a thread in order to keep data fresh
    public void UpdateLimeLightData(){
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            Pose3D botpose = result.getBotpose();
            if (botpose != null) {
                x = botpose.getPosition().x;
                y = botpose.getPosition().y;
                heading = botpose.getOrientation().getYaw(AngleUnit.DEGREES);
            }
            tx = result.getTx();
            ty = result.getTy();
            ta = result.getTa();
        }
    }
    //getter methods to access the data provided by the limelight
    public double getTx() {
        return tx;
    }
    public double getTy() {
        return ty;
    }
    public double getTa() {
        return ta;
    }
    public double getX() {
        return x;
    }
    public double getY() {
        return y;
    }
    public void setLimeLightFilter(int filter){
        limelight.pipelineSwitch(filter);
    }
    public double getHeading() {
        return heading;
    }

}
