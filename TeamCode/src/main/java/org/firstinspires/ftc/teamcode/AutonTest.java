package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.ArmPositions;
import org.firstinspires.ftc.teamcode.hardware.RobotControlIntake;
import org.firstinspires.ftc.teamcode.hardware.RobotControlSpecimenLifter;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.opencv.core.RotatedRect;

import java.util.List;

@Autonomous(name = "Auton Test", group = "Autons")
public class AutonTest extends AutonBase {

    @Override
    public void runOpMode() {

        initialize();

        RobotControlIntake intake = new RobotControlIntake(theHardwareMap, this);
        intake.initialize();

        ColorBlobLocatorProcessor colorLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.BLUE)
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.asUnityCenterCoordinates(-1,0.8,1,-0.1))
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
        telemetry.setMsTransmissionInterval(50);//debug only
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);

        int rightTarget = 0;
        int largestTarget = 0;
        int centerPixel = 320;
        double pixelsPerInch = 45.0;
        double adjustDistance = 0.0;

        // do something in init mode?
        while (opModeInInit()) {
            /*
            //List of detected blobs
            telemetry.addLine(" Area Density Aspect  Center");

            List<ColorBlobLocatorProcessor.Blob> blobs = colorLocator.getBlobs();
            ColorBlobLocatorProcessor.Util.filterByArea(50, 20000, blobs);  // filter out very small blobs.

            for (ColorBlobLocatorProcessor.Blob b : blobs)
            {
                RotatedRect boxFit = b.getBoxFit();
                telemetry.addLine(String.format("%5d %4.2f %5.2f (%3d,%3d)",
                        b.getContourArea(),b.getDensity(),b.getAspectRatio(), (int) boxFit.center.x, (int) boxFit.center.y));

                if (b.getContourArea() > largestTarget)
                {
                    largestTarget = b.getContourArea();
                    rightTarget = (int) boxFit.center.x;
                }
            }

            //drive to the biggest blob to the right
            //Calculate the distance by dividing the distance from the target to center by the number of pixels per inch
            //One thing to consider is that the pixels per inch is dependent on the distance of the camera to the wall
            if (rightTarget > centerPixel)
            {
                adjustDistance = (rightTarget - centerPixel) / pixelsPerInch;
            }
            else if (rightTarget < centerPixel)
            {
                adjustDistance = (centerPixel - rightTarget) / pixelsPerInch * -1;
            }

            telemetry.addData("Largest Target:",largestTarget);
            telemetry.addData("Right Target:",rightTarget);
            telemetry.addData("adjust Distance:",adjustDistance);
            telemetry.update();

             */
        }

        //Initialize remaining variables

        //Main Loop
        waitForStart();

        //Set the arm motor to the drive position
        intakeArm.moveArmEncoded(ArmPositions.DRIVE);

        sleep(2000);

        imuDrive(autonSlow,20,0);

        //imuDrive(autonSlow,adjustDistance,0);

        telemetry.addLine("I was running");
        telemetry.update();

    }
}
