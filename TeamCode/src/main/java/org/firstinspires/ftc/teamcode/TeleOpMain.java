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
public class TeleOpMain extends LinearOpMode {

    static final String TAG = "TeleOp Main";

    @Override
    public void runOpMode() throws InterruptedException {
        //Initialize hardware
        RobotHardwareMap theHardwareMap = new RobotHardwareMap(this.hardwareMap, this);
        theHardwareMap.initialize();

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        RobotControlMechanum robotDrive = new RobotControlMechanum(theHardwareMap, this);
        robotDrive.initialize();

        RobotControlLights lights = new RobotControlLights(theHardwareMap, this);
        AutonBase autonBase = new AutonBase();

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
            //telemetry.addData("Robot", "Initialized successfully. Ready to run?");
            //telemetry.update();
        }

        //telemetry.addData("Robot", "running teleop.. press (Y) For telemetry");
        //telemetry.update();

        lights.switchLight(Light.ALL, LightMode.OFF);

        //Initialize remaining variables
        double loopTimeStart = 0;
        boolean slowMode = true;
        lights.switchLight(Light.LED1, LightMode.OFF);
		lights.switchLight(Light.LED2, LightMode.GREEN);
        boolean showTelemetry = false;

        //create some gamepads to look at debouncing
        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();

        double currentClaw = 0.8;
        //Main Loop
        while (opModeIsActive()) {

            loopTimeStart = System.currentTimeMillis();

            //copy over the previous gamepads so we can compare what changed
            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);

            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            //mechanum drive - left stick y is negative because the up is negative
            double drive = -1 * gamepad1.left_stick_y;
            double strafe = -1 * gamepad1.left_trigger + gamepad1.right_trigger;
            //double strafe = gamepad1.left_stick_x;
            double twist = gamepad1.right_stick_x;

            //Speed values for slow mode
            if (slowMode) {
                drive *= 0.3;
                strafe *= 0.3;
                twist *= 0.3;

            } else { // non slow mode is only 75% power
                drive *= .7;
                strafe *= .7;
                twist *= .7;
            }

            robotDrive.teleOpMechanum(drive, strafe, twist);

            //slow mode
            if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper) {
                slowMode = true;
                lights.switchLight(Light.LED2, LightMode.GREEN);
            } else if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper) {
                slowMode = false;
                lights.switchLight(Light.LED2, LightMode.YELLOW);
            }


            telemetry.addData("Front Left Position:",theHardwareMap.frontLeftMotor.getCurrentPosition());
            telemetry.addData("Front Right Position:",theHardwareMap.frontRightMotor.getCurrentPosition());
            telemetry.addData("Back Left Position:",theHardwareMap.backLeftMotor.getCurrentPosition());
            telemetry.addData("Back Right Position:",theHardwareMap.backRightMotor.getCurrentPosition());
            telemetry.update();

            /***************
             * Gamepad 2
             */

        }

        //telemetry.addData("Status", "Stopped");
        //telemetry.update();

    }
}