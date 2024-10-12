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
        imuDrive(autonFast, 22, 0);
        encoderStrafe(autonFast, 14, 5);
        imuTurn(autonMedium,90);
        encoderStrafe(autonMedium,-8,5);
        sleep(1000);
        encoderStrafe(autonMedium,5,5);
        imuTurn(autonMedium,-90);
        imuDrive(autonMedium,-3,0);

        //get and deliver first sample
        imuTurn(autonMedium, -90);
        imuDrive(autonFast, 43, 0);
        imuTurn(autonMedium, 90);
        imuDrive(autonFast,5,0);
        imuDrive(autonFast,-5,0);
        encoderStrafe(autonMedium, -10.5, 5);
        imuDrive(autonFast, -17, 0);
        imuTurn(autonMedium, 45); //square up with basket
        //encoderStrafe(.43,4,5);
        sleep(1000);
        imuTurn(autonMedium, -45);


        //get and deliver second sample
        imuDrive(autonFast, 20, 0);
        imuDrive(autonFast, -17.25, 0);
        imuTurn(autonMedium, 45); //square up with basket
        sleep(1000);
        imuTurn(autonMedium, 45);

        //get and deliver third sample9
       /* imuDrive(autonFast,29,0);
        imuTurn(autonMedium,-90);
        imuDrive(autonFast,15,0);
        imuDrive(autonFast,-5,0);
        imuTurn(autonMedium,90);
        imuDrive(autonFast,-24,0);
        encoderStrafe(autonFast,-5,3);
        imuTurn(autonFast,45);*/

        /*close to wall strafe
        encoderStrafe(autonFast,9.25,2);
        imuDrive(autonFast,111,0);*/

        //close to submersible strafe
        encoderStrafe(autonFast,-21.5,3);
        imuDrive(autonFast,111,0);
        encoderStrafe(autonFast,21.5,3);





    }

}
