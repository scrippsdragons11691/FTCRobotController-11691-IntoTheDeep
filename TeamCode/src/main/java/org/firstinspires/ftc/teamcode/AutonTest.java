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
        imuDrive(0.43, 22, 0);
        encoderStrafe(0.43, 12, 5);
        imuTurn(0.43,90);
        encoderStrafe(0.43,-8,5);
        encoderStrafe(0.34,5,5);
        imuTurn(0.43,-90);
        imuDrive(0.43,-3,0);
        //imuDrive(0.43,45,0);
        sleep(2000);

        //get and deliver first sample
        //imuTurn(0.43,-5);
        //imuDrive(0.43, -5, 0);
        imuTurn(0.43, -90);
        imuDrive(0.43, 43, 0);
        imuTurn(0.43, 90);
        imuDrive(0.43,5,0);
        imuDrive(0.43,-5,0);
        sleep(500);
        encoderStrafe(.43, -10.5, 5);
        imuDrive(0.43, -17, 0);
        imuTurn(.4, 45); //square up with basket
        //encoderStrafe(.43,4,5);
        sleep(1500);
        imuTurn(.4, -45);


        //get and deliver second sample
        imuDrive(.43, 20, 0);
        encoderStrafe(0.43,5,5);
        imuDrive(.43, -16.75, 0);
        imuTurn(.4, 45); //square up with basket
        sleep(1500);
        imuTurn(.4, -45);

        //park close to wall
        /*imuTurn(.43,45);
        imuDrive(.7,111,0);
        encoderStrafe(.5,15,3);

         */




    }

}
