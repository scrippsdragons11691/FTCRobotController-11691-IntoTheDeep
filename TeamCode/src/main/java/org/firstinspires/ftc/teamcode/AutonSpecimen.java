package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
@Autonomous(name = "Auton Specimen", group = "Autons")

public class AutonSpecimen extends AutonBase{
    //This is the auton to deliver samples to the observation zone

    @Override
    public void runOpMode() {

        initialize();
        waitForStart();

        imuDrive(autonMedium, 27, 0);
        sleep(1000);
        imuDrive(autonMedium, -3,0);
        imuTurn(autonMedium,90);
        imuDrive(autonMedium, 24,0);
        encoderStrafe(autonMedium, -31, 5);
        imuDrive(autonMedium,12,0);
        imuTurn(autonMedium,90);
        imuDrive(autonMedium,44,0);
        imuDrive(autonMedium,-44,0);
        encoderStrafe(autonMedium,-12,5);
        imuDrive(autonMedium, 44, 0);

        /*
        imuTurn(autonMedium,180);
        imuDrive(autonMedium,13.5,0);
        imuDrive(autonMedium,-10, 0);
        encoderStrafe(autonMedium,-8,5);
        imuTurn(autonMedium,180);
        imuTurn(autonMedium,180);
        imuDrive(autonMedium,13.5,0);
        */
    }

}