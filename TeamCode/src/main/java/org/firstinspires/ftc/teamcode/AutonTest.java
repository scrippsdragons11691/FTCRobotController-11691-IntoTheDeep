package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Auton Test", group = "Autons")
public class AutonTest extends AutonBase {

    @Override
    public void runOpMode() {

        initialize();
        waitForStart();
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
        imuDrive(autonMedium, 22, 0);
        encoderStrafe(autonMedium, 12, 5);
        imuTurn(autonMedium,90);
        encoderStrafe(autonMedium,-8,5);
        encoderStrafe(autonMedium,5,5);
        imuTurn(autonMedium,-90);
        imuDrive(autonMedium,-3,0);
        //imuDrive(0.43,45,0);
        sleep(2000);

        //get and deliver first sample
        //imuTurn(0.43,-5);
        //imuDrive(0.43, -5, 0);
        imuTurn(autonMedium, -90);
        imuDrive(autonMedium, 43, 0);
        imuTurn(autonMedium, 90);
        imuDrive(autonMedium,5,0);
        imuDrive(autonMedium,-5,0);
        sleep(500);
        encoderStrafe(autonMedium, -10.5, 5);
        imuDrive(autonMedium, -17, 0);
        imuTurn(autonMedium, 45); //square up with basket
        //encoderStrafe(.43,4,5);
        sleep(1500);
        imuTurn(autonMedium, -45);


        //get and deliver second sample
        imuDrive(autonMedium, 20, 0);
        encoderStrafe(autonMedium,-5,5);
        imuDrive(autonMedium, -17.25, 0);
        imuTurn(autonMedium, 45); //square up with basket
        sleep(1500);
        imuTurn(autonMedium, -45);

        //get and deliver third sample
        encoderStrafe(autonMedium,15,3);
        imuDrive(autonMedium,29,0);
        imuTurn(autonMedium,-90);
        imuDrive(autonMedium,15,0);
        imuDrive(autonMedium,-5,0);
        imuTurn(autonMedium,90);
        imuDrive(autonMedium,-24,0);
        encoderStrafe(autonMedium,-5,3);
        imuTurn(autonMedium,45);
        //park close to wall
        /*imuTurn(.43,45);
        imuDrive(.7,111,0);
        encoderStrafe(.5,15,3);*/
        //park closer to submersible







    }

}
