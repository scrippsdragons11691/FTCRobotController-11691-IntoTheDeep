package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Auton Test", group = "Autons")
public class AutonTest extends AutonBase {

    @Override
    public void runOpMode(){

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
        imuDrive(0.25,23, 0);
        encoderStrafe (0.25,8, 5);
        imuDrive(0.25,5, 0);
        //deliver specimen
        imuDrive(0.25,-5, 0);
        imuTurn (0.25,90);
        imuDrive (0.25, -21, 0);
        imuTurn (0.25,-90);
        imuDrive (0.25,-18,0);



    }

}
