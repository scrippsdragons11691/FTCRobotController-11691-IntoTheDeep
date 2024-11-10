package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.hardware.ArmPositions;
import org.firstinspires.ftc.teamcode.hardware.GripperPositions;
import org.firstinspires.ftc.teamcode.hardware.LifterPositions;

@Autonomous(name = "Auton Specimen", group = "Autons")

public class AutonSpecimen extends AutonBase{
    //This is the auton to deliver samples to the observation zone

    @Override
    public void runOpMode() {

        initialize();

        waitForStart();

        //for testing
        autonMedium = .4;
        autonSlow=0.15;

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

        imuDrive(autonMedium,-48,0);
        encoderStrafe(autonMedium,-8.5,5);
        imuDrive(autonMedium, 49, 0);
        imuDrive(autonMedium,-12,0);
        imuTurn(autonSlow,90);
        sleep(1000);
        //Find specimen
        encoderStrafe(autonMedium,-12,5);

        //grab specimen
        gripperServo.moveToPosition(GripperPositions.GRIPPER_CLOSED);
        specimenLifter.moveLifterEncoded(LifterPositions.TOP_DELIVER);

        encoderStrafe(autonMedium,3,5);
        //specimenLifter.moveLifterEncoded(LifterPositions.BOTTOM);

        //Drive back to submersisble
        imuDrive(autonMedium,47,0);
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

    }

}