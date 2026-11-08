package org.firstinspires.ftc.teamcode.Auto_Test;

import android.util.Size;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
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
    VisionPortal Camera1 =null;
    ColorBlobLocatorProcessor ColorSensor1 ;
    AprilTagProcessor AprilTagProcessor1 ;
    GoBildaPinpointDriver pinpointDriver;
    double AP_X = 0,AP_Y=0,AP_Z=0;
    double x=0,y=0,heading=0;
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
        AprilTagProcessor1 = new AprilTagProcessor.Builder()
                .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary()) // 載入官方賽事 Tag 庫
                .setDrawTagID(true)               // 顯示 ID
                .setCameraPose(cameraPosition, cameraOrientation) // 設定相機姿態
                .setDrawTagOutline(true)          // 繪製外框線
                .setDrawAxes(true)                // 繪製坐標軸
                .setDrawCubeProjection(true)      // 繪製 3D 虛擬立方體
                .build();
        Camera1 = new VisionPortal.Builder()
                .setCameraResolution(new Size(1280,720))
                .setCamera(hardwareMap.get(WebcamName.class,"Camera1"))
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .enableLiveView(true)
                .setAutoStopLiveView(true)
                .build();
    }

    enum state {
        STEP_1,
        STEP_2,
        Final
    }
    private void read_AprilTag(){
        List<AprilTagDetection> Current = AprilTagProcessor1.getDetections();
        for(AprilTagDetection Detection : Current){
            AP_X = Detection.robotPose.getPosition().x;
            AP_Y = Detection.robotPose.getPosition().y;
            AP_Z = Detection.robotPose.getPosition().z;
            int ID = Detection.id;
            telemetry.addLine(String.format("XYZ %3.1f %3.1f %3.1f",AP_X,AP_Y,AP_Z));
            telemetry.addData("ID",ID);
        }
        telemetry.update();
    }
    private void move (){
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
            Math_Odo();

            read_AprilTag();
            switch (state) {
                case STEP_1:
                move();
                if(stateFinal){
                    state = Auto_Locate.state.STEP_2;
                }
                break;
                //*=======================*\\
                case STEP_2:
                    stateFinal=false;
                    Math_Odo();
                    read_AprilTag();
                    Step_2();
                    if(stateFinal)state = Auto_Locate.state.Final;
                    break;
                default:
                    telemetry.addLine("This Autonomous is already finished");
            }
        }
    }
}
