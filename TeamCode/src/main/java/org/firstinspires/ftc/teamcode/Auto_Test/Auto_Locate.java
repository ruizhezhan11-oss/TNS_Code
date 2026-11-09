package org.firstinspires.ftc.teamcode.Auto_Test;

import android.util.Size;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;

import java.util.List;

@Autonomous
public class Auto_Locate extends LinearOpMode {
    DcMotor FL,FR,BL,BR;
    Limelight3A limelight = null;
    GoBildaPinpointDriver pinpointDriver;
    double AP_X = 0,AP_Y=0,AP_Z=0;
    double x=0,y=0,heading=0;
    int ID_Code = 0;
    private final Position cameraPosition = new Position(DistanceUnit.INCH, 0, 0, 0, 0);
    private final YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES, 0, -90, 0, 0);
    public void init_hardware (){
        BL=hardwareMap.get(DcMotor.class,"BL");
        BR=hardwareMap.get(DcMotor.class,"BR");
        FL=hardwareMap.get(DcMotor.class,"FL");
        FR=hardwareMap.get(DcMotor.class,"FR");
        BL.setDirection(DcMotor.Direction.REVERSE);
        BR.setDirection(DcMotor.Direction.FORWARD);
        FL.setDirection(DcMotor.Direction.REVERSE);
        FR.setDirection(DcMotor.Direction.FORWARD);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void reset(){pinpointDriver.resetPosAndIMU();
        sleep(300);}
    public void init_odo(){
        pinpointDriver=hardwareMap.get(GoBildaPinpointDriver.class,"pinpoint");
        pinpointDriver.setOffsets(0,0, DistanceUnit.INCH);
        pinpointDriver.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpointDriver.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );
        reset();
    }
    public void Math_Odo(){
        pinpointDriver.update();
        Pose2D pose2D = pinpointDriver.getPosition();
        x=pose2D.getX(DistanceUnit.INCH);
        y= pose2D.getY(DistanceUnit.INCH);
        heading=pose2D.getHeading(AngleUnit.DEGREES);
    }
    public void telemetry(){
        telemetry.addLine(String.format("XYH,%2.1f %2.1f %2.1f",x,y,heading));
    }
    private void init_Vision(){
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);   // 每秒更新次數
        limelight.pipelineSwitch(0);    // 網頁介面中的 pipeline 編號
        limelight.start();
    }

    enum state {
        STEP_1,
        STEP_2,
        Final
    }
    private void read_AprilTag(){
        limelight.updateRobotOrientation(heading);
        LLResult result = limelight.getLatestResult();
        if(result !=null && result.isValid()){
            Pose3D pose = result.getBotpose_MT2();
            if(pose !=null){
                Position p = pose.getPosition().toUnit(DistanceUnit.INCH);
                AP_X = p.x;
                AP_Y = p.y;
                AP_Z = pose.getOrientation().getYaw(AngleUnit.DEGREES);
            }
            for (LLResultTypes.FiducialResult f : result.getFiducialResults()) {
                ID_Code = f.getFiducialId();   // 看到的 Tag ID
            }
            telemetry.addLine(String.format("XYZ %3.1f %3.1f %3.1f",AP_X,AP_Y,AP_Z));
            telemetry.addData("ID",ID_Code);
            telemetry.update();
        }

    }
    private void Step_1(){
        double correct_Heading = Math.toRadians(target_h);
        double dx = (target_x - x), dy = (target_y - y), eh = (correct_Heading - heading);
        double dist = Math.hypot(dx, dy); //直角三角形定位
        double forward = dx * Math.cos(heading) + dy * Math.sin(heading);
        double strafe = dx * Math.sin(heading) + dy * Math.cos(heading);
        eh = Math.atan2(Math.sin(eh), Math.cos(eh));
        //*誤差調整
        double fwdPower = Range.clip(Kp_drive * forward, -0.5, 0.5);
        double strPower = Range.clip(Kp_drive * strafe, -0.5, 0.5);
        double turnPower = Range.clip(Kp_turn * eh, -0.3, 0.3);
        //*驅動程式
        double fl = fwdPower + strPower + turnPower;
        double fr = fwdPower - strPower - turnPower;
        double bl = fwdPower - strPower + turnPower;
        double br = fwdPower + strPower - turnPower;
        //*馬達
        FL.setPower(fl);
        FR.setPower(fr);
        BL.setPower(bl);
        BR.setPower(br);
        //*判別式
        if(dist < tolerance){
            telemetry.addLine("第一步結束");
            telemetry.update();
            stateFinal = true ;

        }
    }
    private void Step_2(){
        double mx = AP_X - x ,my = AP_Y - y,mz = AP_Z - heading ;
        double dist = Math.hypot(mx, my); //直角三角形定位
        double forward = mx * Math.cos(heading) + my * Math.sin(heading);
        double strafe = mx * Math.sin(heading) + my * Math.cos(heading);
        mz = Math.atan2(Math.sin(mz), Math.cos(mz));
        double fwdPower = Range.clip(Kp_drive * forward, -0.5, 0.5);
        double strPower = Range.clip(Kp_drive * strafe, -0.5, 0.5);
        double turnPower = Range.clip(Kp_turn * mz, -0.3, 0.3);
        double fl = fwdPower + strPower + turnPower;
        double fr = fwdPower - strPower - turnPower;
        double bl = fwdPower - strPower + turnPower;
        double br = fwdPower + strPower - turnPower;
        //*馬達
        FL.setPower(fl);
        FR.setPower(fr);
        BL.setPower(bl);
        BR.setPower(br);
        //*判別式
        if(dist < tolerance){
            telemetry.addLine("第二步結束");
            telemetry.update();
            stateFinal = true ;

        }
    }
    private void Final_Step (){
        stateFinal = false;
        double target_X = 0,target_Y = 0,target_H = 0;
        double dx = (target_X - x), dy = (target_Y - y), eh = (target_H - heading);
        double dist = Math.hypot(dx, dy); //直角三角形定位
        double forward = dx * Math.cos(heading) + dy * Math.sin(heading);
        double strafe = dx * Math.sin(heading) + dy * Math.cos(heading);
        eh = Math.atan2(Math.sin(eh), Math.cos(eh));
        double fwdPower = Range.clip(Kp_drive * forward, -0.5, 0.5);
        double strPower = Range.clip(Kp_drive * strafe, -0.5, 0.5);
        double turnPower = Range.clip(Kp_turn * eh, -0.3, 0.3);
        double fl = fwdPower + strPower + turnPower;
        double fr = fwdPower - strPower - turnPower;
        double bl = fwdPower - strPower + turnPower;
        double br = fwdPower + strPower - turnPower;
        FL.setPower(fl);
        FR.setPower(fr);
        BL.setPower(bl);
        BR.setPower(br);
        if(dist < tolerance){
            telemetry.addLine("Final");
            telemetry.update();
            stateFinal = true;
        }
    }

    double target_x = 72 , target_y = 48 , target_h = 90;
    double tolerance = 2;
    double Kp_drive = 0.01 , Kp_turn = 1.0;
    boolean stateFinal = false;

    @Override
    public void runOpMode(){
        init_hardware();
        init_Vision();
        init_odo();
        state state = Auto_Locate.state.STEP_1;
        waitForStart();
        while (opModeIsActive()){
            Math_Odo(); //擷取Pinpoint
            read_AprilTag(); //擷取AprilTag
            switch (state) {
                case STEP_1:
                Step_1();
                if(stateFinal){
                    state = Auto_Locate.state.STEP_2;
                }
                break;
                //*=======================*\\
                case STEP_2:
                    stateFinal=false;
                    Math_Odo(); //擷取Pinpoint
                    read_AprilTag(); //擷取AprilTag
                    Step_2(); //Step 2主程式
                    if(stateFinal)state = Auto_Locate.state.Final;
                    break;
                //*========================*\\
                case Final:
                    Math_Odo(); //擷取Pinpoint
                    read_AprilTag(); //擷取AprilTag
                    Final_Step(); //Final Step 主程式
                    if(stateFinal) break;
                default:
                    telemetry.addLine("This Autonomous is already finished");
            }
        }
    }
}
