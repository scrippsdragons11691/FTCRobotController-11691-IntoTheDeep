package org.firstinspires.ftc.teamcode.hardware;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotHardwareMap;

public class RobotControlIntake {
    static final String TAG = "RobotControlIntake";
    RobotHardwareMap robotHardwareMap;
    LinearOpMode opMode;
    private Telemetry telemetry;
    DcMotorEx intakeMotor;
    CRServo intakeServo;
    private boolean intakeInitialized = false;
    ControlModes mode = ControlModes.MANUAL;


    public RobotControlIntake(RobotHardwareMap robotHardwareMap, LinearOpMode opMode){
        this.opMode = opMode;
        this.robotHardwareMap = robotHardwareMap;
        initialize();
    }

    public void initialize(){
        try {
            intakeMotor = robotHardwareMap.baseHMap.get(DcMotorEx.class, "Intake Motor");
            intakeMotor.setPower(0);
            intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            intakeMotor.setDirection(DcMotorEx.Direction.REVERSE);
            intakeMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            intakeMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            intakeInitialized = true;
            opMode.telemetry.addData("Intake Motor", "initialized");
        } catch (IllegalArgumentException iae){
            opMode.telemetry.addData("Intake Motor", iae.getMessage());
        }

        try {
            intakeServo = robotHardwareMap.baseHMap.crservo.get("Intake Servo");
            opMode.telemetry.addData("Intake Servo:", "Initialized");
            intakeServo.setDirection(CRServo.Direction.REVERSE);
            intakeServo.setPower(0);

        } catch (IllegalArgumentException iae){
            opMode.telemetry.addData("Intake Servo:", iae.getMessage());
            intakeInitialized = false;
        }
    }

    public void moveIntake(double power ){
        mode = ControlModes.MANUAL;

        //only try moving the arm if initialized
        if (intakeInitialized) {
            intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            double currentPosition = intakeMotor.getCurrentPosition();
            intakeMotor.setPower(power);
        }
    }

    public double getIntakeEncodedPosition(){
        return intakeMotor.getCurrentPosition();
    }

    public void setIntakePower(double power){
        if(intakeInitialized)
        {
            intakeServo.setPower(power);
            opMode.telemetry.addData("Intake Power:",intakeServo.getPower());
        }
    }

    public double getIntakePower() {return intakeServo.getPower();}

}