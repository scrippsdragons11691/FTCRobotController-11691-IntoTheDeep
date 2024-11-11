package org.firstinspires.ftc.teamcode.hardware;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotHardwareMap;

public class RobotControlArm {


    static final String TAG = "RobotControlArm";
    RobotHardwareMap robotHardwareMap;
    LinearOpMode opMode;
    private Telemetry telemetry;
    private boolean dashLogging = true;

    DcMotorEx armMotor;
    private boolean armInitialized = false;
    ControlModes mode = ControlModes.MANUAL;
    ArmPositions armTargetPosition = ArmPositions.UNKNOWN;
    ArmPositions armCurrentPosition = ArmPositions.UNKNOWN;


    public RobotControlArm(RobotHardwareMap robotHardwareMap, LinearOpMode opMode){
        this.opMode = opMode;
        this.robotHardwareMap = robotHardwareMap;
        initialize();
    }

    public void initialize(){
        try {
            armMotor = robotHardwareMap.baseHMap.get(DcMotorEx.class, "Arm Motor");
            armMotor.setPower(0);
            armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            armMotor.setDirection(DcMotorEx.Direction.REVERSE);
            armMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            armMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            armInitialized = true;
            opMode.telemetry.addData("Arm Motor", "initialized");
        } catch (IllegalArgumentException iae){
            opMode.telemetry.addData("Arm Motor", iae.getMessage());
        }

        if (dashLogging) {
            telemetry = opMode.telemetry;
            FtcDashboard dashboard = FtcDashboard.getInstance();
            telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        }
    }

    public void moveArmPower(double power ){
        mode = ControlModes.MANUAL;
        armCurrentPosition = ArmPositions.UNKNOWN;

        //only try moving the arm if initilized
        if (armInitialized) {
            armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            double currentPosition = armMotor.getCurrentPosition();

            double lowerPowerFloat = 0.01;

            //Based on the position we adjust the power automatically
            //Going up, we want full power until it is past the vertical position then we want to float down so we don't slam the robot
            if (power > 0)
            {
                if (currentPosition > 1450)
                {
                    armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    power = lowerPowerFloat;
                }
                else
                {
                    armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                }

            }
            //Going down, we want to have power until it is across the top then we float down
            else if (power < 0)
            {
                if(currentPosition < 1400)
                {
                    armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    power = lowerPowerFloat;
                }
                else
                {
                    armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                }
            }

            armMotor.setPower(power);
        }

    }

    public void moveArmEncoded(ArmPositions armTargetPosition){
        if (armInitialized){
            mode = ControlModes.AUTO;
            int currentPosition = armMotor.getCurrentPosition();
            armMotor.setTargetPosition(armTargetPosition.getEncodedPos());
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armMotor.setPower(0.5);
        }
    }

    public void moveArmReset(double power){
        if (armInitialized) {
            armMotor.setPower(power);
            armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            opMode.sleep(500);
            armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            armMotor.setDirection(DcMotorEx.Direction.REVERSE);
            armMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            armMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    public void stopArmWithHold(){
        if (armInitialized) {
            if (mode == ControlModes.MANUAL) {
                //hold code
                double currentPosition = armMotor.getCurrentPosition();
                double holdPower = 0.3;
                armMotor.setPower(holdPower);
            }
        }
    }

    public void stopArm(){
        if (armInitialized){
            armMotor.setPower(0);
        }
    }

    public void addArmTelemetry(){
        if (armInitialized){
            telemetry.addData("armEncoder:", armMotor.getCurrentPosition());
        } else {
            telemetry.addData("armEncoder:", "Uninitialized!");
        }
    }

    public double getArmEncodedPosition(){
        return armMotor.getCurrentPosition();
    }
}