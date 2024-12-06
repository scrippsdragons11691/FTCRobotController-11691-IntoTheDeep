package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.hardware.ArmPositions;
import org.firstinspires.ftc.teamcode.hardware.GripperPositions;
import org.firstinspires.ftc.teamcode.hardware.LifterPositions;
import org.firstinspires.ftc.teamcode.hardware.RobotCameraLight;
import org.firstinspires.ftc.teamcode.hardware.RobotControlIntake;

@Autonomous(name = "Auton Sample", group = "Autons")
public class AutonSample extends AutonBase {

    @Override
    public void runOpMode() {

        initialize();
        RobotCameraLight cameraLight = new RobotCameraLight(theHardwareMap, this);
        cameraLight.initialize();

        RobotControlIntake intake = new RobotControlIntake(theHardwareMap, this);
        intake.initialize();

        initialize();
        waitForStart();

        //Set the arm motor to the drive position
        intakeArm.moveArmEncoded(ArmPositions.DRIVE);

        firstSpecimenDeliver();

        //drive to ascent zone

        encoderStrafe(autonMedium,10,5);
        specimenLifter.moveLifterEncoded(LifterPositions.PICKUP);
        imuDrive(autonMedium, -25,0);
        encoderStrafe(autonMedium, -34, 5);
        imuDrive(autonMedium,8,0);
    }
}
