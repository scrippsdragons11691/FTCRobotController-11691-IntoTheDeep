package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.hardware.ArmPositions;
import org.firstinspires.ftc.teamcode.hardware.GripperPositions;
import org.firstinspires.ftc.teamcode.hardware.LifterPositions;
import org.firstinspires.ftc.teamcode.hardware.RobotCameraLight;
import org.firstinspires.ftc.teamcode.hardware.RobotControlIntake;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.opencv.core.RotatedRect;

import java.util.List;

@Autonomous(name = "Auton Specimen Safe", group = "Autons")

public class AutonSpecimenSafe extends AutonBase{
    //This is the auton to deliver samples to the observation zone and pickup specimens to deliver

    @Override
    public void runOpMode() {

        initialize();
        RobotCameraLight cameraLight = new RobotCameraLight(theHardwareMap, this);
        cameraLight.initialize();

        RobotControlIntake intake = new RobotControlIntake(theHardwareMap, this);
        intake.initialize();

        //Need some way to determine red vs blue

        //Initialize the camera

        waitForStart();

        //Set the arm motor to the drive position
        intakeArm.moveArmEncoded(ArmPositions.DRIVE);

        firstSpecimenDeliver();

        //Drive to push samples
        encoderStrafe(autonMedium,10,5);
        specimenLifter.moveLifterEncoded(LifterPositions.PICKUP);
        imuDrive(autonMedium, 25,0);
        encoderStrafe(autonMedium, -34, 5);
        imuDrive(autonMedium,12,0);


        //we spin around to use the plow on the left side of the robot
        //imuTurn(autonSlow,90);
        //imuTurn(autonSlow,90);
        encoderStrafe(autonMedium,45,5);

        //Push the second sample

        encoderStrafe(autonMedium,-45,5);
        imuDrive(autonMedium,8,0);
        encoderStrafe(autonMedium, 45, 5);


        //Reset the drive motors to 0 before auton ends
        specimenLifter.moveLifterEncoded(LifterPositions.BOTTOM);
        intakeArm.moveArmEncoded(ArmPositions.PICKUP);

        //Give the robot any remaining time to lower the arms to their starting position
        sleep(5000);
    }

}