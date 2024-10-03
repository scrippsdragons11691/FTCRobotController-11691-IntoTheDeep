package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotHardwareMap;

public class RobotControlLight {

    static final String TAG = "RobotControlLight";
    RobotHardwareMap robotHardwareMap;
    LinearOpMode opMode;
    String lightLocation = "leftRed";
    DigitalChannel led;
    boolean ledInit = false;

    public RobotControlLight(RobotHardwareMap robotHardwareMap, LinearOpMode opMode, String lightLocation){
        this.opMode = opMode;
        this.robotHardwareMap = robotHardwareMap;
        this.lightLocation = lightLocation;
        initialize();
    }

    public void initialize(){
        try {
            led = robotHardwareMap.baseHMap.get(DigitalChannel.class, lightLocation);
            led.setMode(DigitalChannel.Mode.OUTPUT);
            ledInit = true;
        } catch (IllegalArgumentException iae){
            //don't output error for light initialization
            //opMode.telemetry.addData(lightLocation, iae.getMessage());
        }
    }

    public void setState(boolean state){
        if (ledInit){
            led.setState(state);
        }
    }
}
