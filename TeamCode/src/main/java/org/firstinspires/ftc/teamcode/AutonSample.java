package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Auton Sample", group = "Autons")
public class AutonSample extends AutonBase {

    @Override
    public void runOpMode() {

        initialize();
        waitForStart();

        //Deliver Specimen
        imuDrive(autonFast, 21, 0);
        encoderStrafe(autonFast, 14, 5);
        imuTurn(autonFast, 90);
        encoderStrafe(autonFast, -8, 5);
        sleep(1000);
        encoderStrafe(autonFast, 5, 5);
        imuTurn(autonFast, -90);
        imuDrive(autonFast, -3, 0);

        //get and deliver first sample
        imuTurn(autonFast, -90);
        imuDrive(autonFast, 43, 0);
        imuTurn(autonFast, 90);
        imuDrive(autonFast, 5, 0);
        imuDrive(autonFast, -5, 0);
        encoderStrafe(autonFast, -10.5, 5);
        imuDrive(autonFast, -17, 0);
        imuTurn(autonFast, 45); //square up with basket
        //encoderStrafe(.43,4,5);
        sleep(1000);
        imuTurn(autonFast, -45);


        //get and deliver second sample
        imuDrive(autonFast, 20, 0);
        imuDrive(autonFast, -17.25, 0);
        imuTurn(autonFast, 45); //square up with basket
        sleep(1000);
        imuTurn(autonFast, 45);

        //get and deliver third sample
        imuDrive(autonFast,22.25,0);
        imuTurn(autonFast,-90);
        imuDrive(autonFast,26.5,0);
        imuTurn(autonFast,-90);
        imuDrive(autonFast,24,0);
        imuTurn(autonFast,90);
        encoderStrafe(autonFast,5,5);
        imuDrive(autonFast,-25,0);
        imuTurn(autonFast,45);

        //touch low rung
        //imuDrive(autonFast,25,0);

        //close to wall strafe
        /*
        encoderStrafe(autonFast, 13, 2);
        imuDrive(autonFast, 111, 0);
        */

        //close to submersible strafe
        /*
        encoderStrafe(autonFast,-21,3);
        imuDrive(autonFast,111,0);
        encoderStrafe(autonFast,23,10);
        */
    }
}
