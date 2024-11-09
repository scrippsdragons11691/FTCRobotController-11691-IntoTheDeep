package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;
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

        //RobotCamera camera = new RobotCamera(theHardwareMap, this);
        //camera.initialize();
        /*
        ColorBlobLocatorProcessor colorLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.RED)
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
        }


        //Initialize remaining variables
        */
        //Main Loop
        waitForStart();

        //imuDrive(autonSlow,adjustDistance,0);

        telemetry.addLine("I was running");
        telemetry.update();

        imuDrive(autonSlow,10,0);

        //robotCamera RobotCamera =
        /*
        imuDrive(0.5, 20, 0);
        encoderStrafe(0.25, -8, 5);
        imuDrive(0.25, 6, 0);
        imuTurn(0.5, 87);
        imuDrive(0.75, 87, 0);
        sleep(4000);
        encoderStrafe(.75, 27, 2000);
        imuDrive(0.5, 12, 0);*/
        //CODE

        //Deliver Specimen
        /*
        imuDrive(autonFast, 21, 0);
        encoderStrafe(autonFast, 14, 5);
        imuTurn(autonFast, 90);
        encoderStrafe(autonFast, -8, 5);
        sleep(1000);
        encoderStrafe(autonFast, 5, 5);
        imuTurn(autonFast, -90);
        imuDrive(autonFast, -3, 0);

        //get and deliver first sample
        imuTurn(autonFast, -90);
        imuDrive(autonFast, 43, 0);
        imuTurn(autonFast, 90);
        imuDrive(autonFast, 5, 0);
        imuDrive(autonFast, -5, 0);
        encoderStrafe(autonFast, -10.5, 5);
        imuDrive(autonFast, -17, 0);
        imuTurn(autonFast, 45); //square up with basket
        //encoderStrafe(.43,4,5);
        sleep(1000);
        imuTurn(autonFast, -45);


        //get and deliver second sample
        imuDrive(autonFast, 20, 0);
        imuDrive(autonFast, -17.25, 0);
        imuTurn(autonFast, 45); //square up with basket
        sleep(1000);
        imuTurn(autonFast, 45);

        //get and deliver third sample
        imuDrive(autonFast,22.25,0);
        imuTurn(autonFast,-90);
        imuDrive(autonFast,26.5,0);
        imuTurn(autonFast,-90);
        imuDrive(autonFast,24,0);
        imuTurn(autonFast,90);
        encoderStrafe(autonFast,5,5);
        imuDrive(autonFast,-25,0);
        imuTurn(autonFast,45);

        //touch low rung
        //imuDrive(autonFast,25,0);
        */



        //close to wall strafe
      /*encoderStrafe(autonFast, 13, 2);
        imuDrive(autonFast, 111, 0);*/

        //close to submersible strafe
        /*encoderStrafe(autonFast,-21,3);
        imuDrive(autonFast,111,0);
        encoderStrafe(autonFast,23,10);
        */
    }
}
