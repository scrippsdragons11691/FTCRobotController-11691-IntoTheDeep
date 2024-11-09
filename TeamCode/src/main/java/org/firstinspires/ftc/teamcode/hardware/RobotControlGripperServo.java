package org.firstinspires.ftc.teamcode.hardware;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotHardwareMap;

public class RobotControlGripperServo {
    static final String TAG = "RobotControlLowerGripperServo";
    RobotHardwareMap robotHardwareMap;
    LinearOpMode opMode;
    private Telemetry telemetry;
    private boolean dashLogging = true;

    Servo servo;
    private boolean servoInitialized = false;
    GripperPositions targetPosition = GripperPositions.UNKNOWN;
    GripperPositions currentPosition = GripperPositions.UNKNOWN;

    // ServoGripper
    public RobotControlGripperServo(RobotHardwareMap robotHardwareMap, LinearOpMode opMode)
    {
        this.opMode = opMode;
        this.robotHardwareMap = robotHardwareMap;
        initialize();
    }

    public void initialize(){
        try {
            servo = robotHardwareMap.baseHMap.servo.get("Gripper Servo");
            servoInitialized = true;
            opMode.telemetry.addData("Gripper Servo:", "Initialized");
            servo.setDirection(Servo.Direction.FORWARD);
        }
        catch (IllegalArgumentException iae){
            opMode.telemetry.addData("Gripper Servo:", iae.getMessage());
        }

        if (dashLogging) {
            telemetry = opMode.telemetry;

            FtcDashboard dashboard = FtcDashboard.getInstance();
            telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        }
    }

    public void moveToPosition(GripperPositions targetPosition){
        if (currentPosition != targetPosition) {
            Log.d(TAG, "Moving from " + currentPosition + " to " + targetPosition);
            currentPosition = targetPosition;
            if (servoInitialized) {
                servo.setPosition(targetPosition.getServoPos());
            }
        }
    }

    public GripperPositions getCurrentPosition(){
        return currentPosition;
    }

    public double getActualPosition(){
        return servo.getPosition();
    }
}

