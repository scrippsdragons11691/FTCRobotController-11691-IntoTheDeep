package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
@Autonomous(name = "Auton Specimen", group = "Autons")

public class AutonSpecimen extends AutonBase{
    //This is the auton to deliver samples to the observation zone

    @Override
    public void runOpMode() {

        initialize();
        waitForStart();

        //push 1st sample to observation zone
        encoderStrafe(autonMedium,-36,5);
        sleep(1000);
        encoderStrafe(autonMedium,9,5);
        imuDrive(autonMedium, 24,0);
        encoderStrafe(autonMedium, -34, 5);
        imuDrive(autonMedium,13.5,0);
        imuTurn(autonMedium,90);
        imuDrive(autonMedium,48,0);

        imuDrive(autonMedium,-48,0);
        encoderStrafe(autonMedium,-18,5);
        imuDrive(autonMedium, 49, 0);
        imuDrive(autonMedium,-35,0);
        encoderStrafe(autonMedium,7,5);
        imuTurn(autonMedium,-90);
        imuDrive(autonMedium,9,0);
        imuTurn(autonMedium,90);
        encoderStrafe(autonMedium,4,5);
        imuDrive(autonMedium,37,0);
        imuDrive(autonMedium,-15,0);
        sleep(3000);
        imuDrive(autonMedium,15,0);


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