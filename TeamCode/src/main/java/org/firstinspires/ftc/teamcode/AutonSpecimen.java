package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.hardware.ArmPositions;
import org.firstinspires.ftc.teamcode.hardware.GripperPositions;
import org.firstinspires.ftc.teamcode.hardware.LifterPositions;
import org.firstinspires.ftc.teamcode.hardware.RobotCameraLight;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.opencv.core.RotatedRect;

import java.util.List;

@Autonomous(name = "Auton Specimen", group = "Autons")

public class AutonSpecimen extends AutonBase{
    //This is the auton to deliver samples to the observation zone and pickup specimens to deliver

    @Override
    public void runOpMode() {

        initialize();
        RobotCameraLight cameraLight = new RobotCameraLight(theHardwareMap, this);
        cameraLight.initialize();

        //Need some way to determine red vs blue

        //Initialize the camera
        boolean cameraInitialized = false;
        ColorBlobLocatorProcessor colorLocator;

        try
        {
            colorLocator = new ColorBlobLocatorProcessor.Builder()
                    .setTargetColorRange(ColorRange.BLUE)
                    .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                    .setRoi(ImageRegion.asUnityCenterCoordinates(-1, 0.8, 1, -0.1))
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
                    .build();

            cameraInitialized = true;

            //Turn the light white
            cameraLight.adjustLight(1.0);
        }
        catch (IllegalArgumentException iae){
            telemetry.addData("Camera Fail:", iae.getMessage());

            colorLocator = null;
            //Turn the light dim
            cameraLight.adjustLight(0.2);
        }

        int rightTarget = 0;
        int largestTarget = 0;
        int centerPixel = 320;
        double pixelsPerInch = 45.0;
        double adjustDistance = 0.0;

        waitForStart();

        //Set the arm motor to the drive position
        intakeArm.moveArmEncoded(ArmPositions.DRIVE);
        sleep(100);

        //push 1st sample to observation zone
        encoderStrafe(autonMedium,-24,5);

        specimenLifter.moveLifterEncoded(LifterPositions.TOP);
        encoderStrafe(autonMedium,-3,5);
        specimenLifter.moveLifterEncoded(LifterPositions.TOP_DELIVER);
        gripperServo.moveToPosition(GripperPositions.GRIPPER_OPEN);

        //Drive to push samples
        encoderStrafe(autonMedium,9,5);
        specimenLifter.moveLifterEncoded(LifterPositions.PICKUP);
        imuDrive(autonMedium, 24,0);
        encoderStrafe(autonMedium, -34, 5);
        imuDrive(autonMedium,13.5,0);
        imuTurn(autonMedium,90);
        imuDrive(autonMedium,48,0);

        //Push the second sample
        imuDrive(autonMedium,-48,0);
        encoderStrafe(autonMedium,-8.5,5);
        imuDrive(autonMedium, 49, 0);
        imuDrive(autonMedium,-12,0);
        imuTurn(autonSlow,90);

        //Find specimen
        if (cameraInitialized && colorLocator != null)
        {
            telemetry.addLine(" Area Density Aspect  Center");

            List<ColorBlobLocatorProcessor.Blob> blobs = colorLocator.getBlobs();
            ColorBlobLocatorProcessor.Util.filterByArea(50, 20000, blobs);  // filter out very small blobs.

            for (ColorBlobLocatorProcessor.Blob b : blobs) {
                RotatedRect boxFit = b.getBoxFit();
                telemetry.addLine(String.format("%5d %4.2f %5.2f (%3d,%3d)",
                        b.getContourArea(), b.getDensity(), b.getAspectRatio(), (int) boxFit.center.x, (int) boxFit.center.y));

                if (b.getContourArea() > largestTarget) {
                    largestTarget = b.getContourArea();
                    rightTarget = (int) boxFit.center.x;
                }
            }

            //drive to the biggest blob to the right
            //Calculate the distance by dividing the distance from the target to center by the number of pixels per inch
            //One thing to consider is that the pixels per inch is dependent on the distance of the camera to the wall
            if (rightTarget > centerPixel) {
                adjustDistance = (rightTarget - centerPixel) / pixelsPerInch;
            } else if (rightTarget < centerPixel) {
                adjustDistance = (centerPixel - rightTarget) / pixelsPerInch * -1;
            }

            //Drive back/forth to align with the specimen
            if (adjustDistance != 0) {
                imuDrive(autonSlow, adjustDistance, 0);
            }
        }

        //Move to the wall
        encoderStrafe(autonMedium,-12,5);

        //grab specimen
        gripperServo.moveToPosition(GripperPositions.GRIPPER_CLOSED);
        specimenLifter.moveLifterEncoded(LifterPositions.TOP_DELIVER);

        encoderStrafe(autonMedium,3,5);
        //specimenLifter.moveLifterEncoded(LifterPositions.BOTTOM);

        //Drive back to submersisble while accounting for the back/forth adjustment
        imuDrive(autonMedium,47 + adjustDistance,0);
        imuTurn(autonSlow,180);
        encoderStrafe(autonMedium,-17,5);

        //Deliver the specimen
        specimenLifter.moveLifterEncoded(LifterPositions.TOP);
        encoderStrafe(autonSlow,3,5);
        specimenLifter.moveLifterEncoded(LifterPositions.TOP_DELIVER);
        gripperServo.moveToPosition(GripperPositions.GRIPPER_OPEN);
        encoderStrafe(autonSlow,3,5);

        //Reset the drive motors to 0 before auton ends
        specimenLifter.moveLifterEncoded(LifterPositions.BOTTOM);
        intakeArm.moveArmEncoded(ArmPositions.PICKUP);

        //Turn off the light
        cameraLight.adjustLight(0);
    }

}