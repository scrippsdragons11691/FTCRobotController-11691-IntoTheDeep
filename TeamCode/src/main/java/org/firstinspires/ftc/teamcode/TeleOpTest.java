package org.firstinspires.ftc.teamcode;

import android.graphics.Canvas;
import android.util.Size;
import android.graphics.Color;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.hardware.RobotControlLights;
import org.firstinspires.ftc.teamcode.hardware.RobotControlMechanum;
import org.firstinspires.ftc.teamcode.hardware.Light;
import org.firstinspires.ftc.teamcode.hardware.LightMode;
import org.firstinspires.ftc.teamcode.hardware.RobotCamera;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.opencv.core.Mat;
import org.opencv.core.RotatedRect;

import java.util.List;


@TeleOp
public class TeleOpTest extends LinearOpMode {

    static final String TAG = "TeleOp TEST";

    @Override
    public void runOpMode() throws InterruptedException {
        //Initialize hardware
        RobotHardwareMap theHardwareMap = new RobotHardwareMap(this.hardwareMap, this);
        theHardwareMap.initialize();

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        //RobotControlMechanum robotDrive = new RobotControlMechanum(theHardwareMap, this);
        //robotDrive.initialize();

        //RobotControlLights lights = new RobotControlLights(theHardwareMap, this);
        //AutonBase autonBase = new AutonBase();

        //RobotCamera camera = new RobotCamera(theHardwareMap, this);
        //camera.initialize();
        telemetry.addLine("here");
        telemetry.update();

/*
        ColorBlobLocatorProcessor colorLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.BLUE)
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.asUnityCenterCoordinates(-0.5,0.8,0.5,-0.1))
                .setDrawContours(true)
                .setBlurSize(5)
                .setBoxFitColor(Color.rgb(255, 120, 31))
                .setRoiColor(Color.rgb(255, 255, 255))
                .setContourColor(Color.rgb(3, 227, 252))
                .build();

        VisionPortal visionPortal = new VisionPortal.Builder()
                .addProcessor(colorLocator)
                .setCamera(theHardwareMap.sideCamera)
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .enableLiveView(false)
                //.setAutoStopLiveView(true)
                .build();
*/
        //List of detected blobs
        //List<ColorBlobLocatorProcessor.Blob> blobs = colorLocator.getBlobs();
        //telemetry.addLine(" Area Density Aspect  Center");
        /*
        for (ColorBlobLocatorProcessor.Blob b : blobs)
        {
            RotatedRect boxFit = b.getBoxFit();
            telemetry.addLine(String.format("%5d %4.2f %5.2f (%3d,%3d)",
                    b.getContourArea(),b.getDensity(),b.getAspectRatio(), (int) boxFit.center.x, (int) boxFit.center.y));
        }

         */
        //telemetry.addLine("Post Blob Display");
        //telemetry.update();


        //DistanceSensor distanceSensor = theHardwareMap.baseHMap.get(DistanceSensor.class, "distance");
        //final int DISTANCE_FROM_BACKBOARD = 7;

        //lights.switchLight(Light.ALL, LightMode.GREEN);

        //telemetry.addData("Robot", "Initialized successfully");
        //telemetry.update();

        waitForStart();

        // do something in init mode?
        while (opModeInInit()) {
            telemetry.addData("Robot", "Initialized successfully. Ready to run?");
            telemetry.update();
        }

        //Initialize remaining variables

        //Main Loop
        while (opModeIsActive()) {

        }

        telemetry.addData("Status", "Stopped");
        telemetry.update();
    }
}