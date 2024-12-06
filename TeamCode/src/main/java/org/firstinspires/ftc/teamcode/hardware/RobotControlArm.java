package org.firstinspires.ftc.teamcode.hardware;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotHardwareMap;

public class RobotControlArm {

    static final String TAG = "RobotControlArm";
    RobotHardwareMap robotHardwareMap;
    LinearOpMode opMode;
    private Telemetry telemetry;
    private boolean dashLogging = true;

    String potentiometerLocation = "Arm Pot";
    AnalogInput potentiometer;

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

            potentiometer = robotHardwareMap.baseHMap.get(AnalogInput.class, potentiometerLocation);

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
                if(currentPosition < 1300)
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

    public void moveArmPot(ArmPositionsPotentiometer armPositionsPotentiometer, RobotControlArm armMotor){
        double power = .75;
        double positionTolerance = 0.1;
        int timeout = 3;

        ElapsedTime runtime = new ElapsedTime();

        runtime.reset();
        while (
                (Math.abs(getCurrentPotPosition() - armPositionsPotentiometer.getVoltagePos()) > positionTolerance)
                && (runtime.seconds() < timeout)
        )
        {
            telemetry.addData("Potentiometer Position: ", getCurrentPotPosition());
            telemetry.update();
            if (getCurrentPotPosition() > armPositionsPotentiometer.getVoltagePos())
            {
                power = -power;
            }
            else if (getCurrentPotPosition() == armPositionsPotentiometer.getVoltagePos())
            {
                power = 0.2;
            }
            else
            {
                power = power;
            }

            //If it is close in position, we want to reduce power to keep it from fighting itself
            if (Math.abs(getCurrentPotPosition() - armPositionsPotentiometer.getVoltagePos()) < .2)
            {
                power = power * .25;
            }

            armMotor.moveArmPower(power);
        }

        armMotor.stopArmWithHold();
        telemetry.addData("Final Position: ", this.getCurrentPotPosition());
        telemetry.update();
    }

    public void stopArmWithHold(){
        if (armInitialized)
        {
            double currentPosition = getCurrentPotPosition();
            double holdPower = 0.0;

            if (currentPosition < 1)
            {
                holdPower = 0.5;
            }
            else if (currentPosition > 1 && currentPosition < ArmPositionsPotentiometer.ARM_TOP_DELIVER.getVoltagePos())
            {
                holdPower = 0;
            }

            armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            armMotor.setPower(holdPower);
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
            telemetry.addData("armPotentiometer:", this.getCurrentPotPosition());
        } else {
            telemetry.addData("armEncoder:", "Uninitialized!");
        }
    }

    public double getArmEncodedPosition(){
        return armMotor.getCurrentPosition();
    }

    public double getCurrentPotPosition(){
        return potentiometer.getVoltage();
    }

}