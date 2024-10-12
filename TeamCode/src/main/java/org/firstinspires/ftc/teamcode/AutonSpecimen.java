package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
@Autonomous(name = "Auton Specimen", group = "Autons")

public class AutonSpecimen extends AutonBase{
    //This is the auton to deliver samples to the observation zone

    @Override
    public void runOpMode() {

        initialize();
        waitForStart();

        imuDrive(autonFast, 24, 0);
        //HANG THE SPECIMEN
        sleep (2000);
        imuDrive(autonFast,-20,0);
        encoderStrafe (autonFast,51,2000);

    }

}