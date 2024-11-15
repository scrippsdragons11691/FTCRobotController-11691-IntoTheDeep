package org.firstinspires.ftc.teamcode.hardware;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotHardwareMap;

public class RobotCameraLight {
    static final String TAG = "RobotCameraLight";
    RobotHardwareMap robotHardwareMap;
    LinearOpMode opMode;
    private Telemetry telemetry;

    Servo light;
    private boolean lightInitialized = false;
    double currentBrightness = 0;

    public RobotCameraLight(RobotHardwareMap robotHardwareMap, LinearOpMode opMode)
    {
        this.opMode = opMode;
        this.robotHardwareMap = robotHardwareMap;
        initialize();
    }

    public void initialize(){
        try {
            light = robotHardwareMap.baseHMap.servo.get("Light");
            lightInitialized = true;
            light.setPosition(0);
            opMode.telemetry.addData("Light:", "Initialized");
        }
        catch (IllegalArgumentException iae){
            opMode.telemetry.addData("Light:", iae.getMessage());
        }
    }

    public void adjustLight(double brightness){
        if (currentBrightness != brightness) {
            Log.d(TAG, "Moving from " + currentBrightness + " to " + brightness);
            currentBrightness = brightness;
            if (lightInitialized) {
                light.setPosition(brightness);
            }
        }
    }

    public double getBrightness(){
        return light.getPosition();
    }
}

