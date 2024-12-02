package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.hardware.ArmPositions;
import org.firstinspires.ftc.teamcode.hardware.GripperPositions;
import org.firstinspires.ftc.teamcode.hardware.LifterPositions;

@Autonomous(name = "Auton Sample", group = "Autons")
public class AutonSample extends AutonBase {

    @Override
    public void runOpMode() {

        initialize();
        waitForStart();

        //Set the arm motor to the drive position
        intakeArm.moveArmEncoded(ArmPositions.DRIVE);

        //close the gripper
        gripperServo.moveToPosition(GripperPositions.GRIPPER_CLOSED);
        specimenLifter.moveLifterEncoded(LifterPositions.TOP);
        sleep(1000);

        //hang initial specimen
        encoderStrafe(autonMedium,-20,5);
        sleep(500);
        encoderStrafe(autonSlow,-5.5,5);
        sleep(1000);
        specimenLifter.moveLifterEncoded(LifterPositions.TOP_DELIVER);
        sleep(1000);
        gripperServo.moveToPosition(GripperPositions.GRIPPER_OPEN);

        //drive to ascent zone

        encoderStrafe(autonMedium,10,5);
        specimenLifter.moveLifterEncoded(LifterPositions.PICKUP);
        imuDrive(autonMedium, -25,0);
        encoderStrafe(autonMedium, 34, 5);
        imuDrive(autonMedium,8,0);
    }
}
