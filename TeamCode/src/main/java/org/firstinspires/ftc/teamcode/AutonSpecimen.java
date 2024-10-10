package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
@Autonomous(name = "Auton Test", group = "Autons")

public class AutonSpecimen extends AutonBase{
    //This is the auton to deliver samples to the observation zone

    @Override
    public void runOpMode() {

        initialize();
        waitForStart();

        imuDrive(autonFast, 20, 0);
    }

}