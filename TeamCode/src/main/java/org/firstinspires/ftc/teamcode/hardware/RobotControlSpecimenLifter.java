package org.firstinspires.ftc.teamcode.hardware;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotHardwareMap;

public class RobotControlSpecimenLifter {
    static final String TAG = "RobotControlSpecimenLifter";
    RobotHardwareMap robotHardwareMap;
    LinearOpMode opMode;
    private Telemetry telemetry;
    private boolean dashLogging = true;

    DcMotorEx specimenMotor;
    private boolean specimenLifterInitialized = false;

    public RobotControlSpecimenLifter(RobotHardwareMap robotHardwareMap, LinearOpMode opMode){
        this.opMode = opMode;
        this.robotHardwareMap = robotHardwareMap;
        initialize();
    }
    public void initialize(){
        try {
            specimenMotor = robotHardwareMap.baseHMap.get(DcMotorEx.class, "Specimen");
            specimenMotor.setPower(0);
            specimenMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            specimenMotor.setDirection(DcMotorEx.Direction.REVERSE);
            specimenMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            specimenMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            //specimenMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            specimenLifterInitialized = true;
            opMode.telemetry.addData("Specimen Motor", "initialized");
        } catch (IllegalArgumentException iae){
            opMode.telemetry.addData("Specimen Motor", iae.getMessage());
        }

        if (dashLogging) {
            telemetry = opMode.telemetry;
            FtcDashboard dashboard = FtcDashboard.getInstance();
            telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        }
    }

    public void moveLifterPower(double power ){
        //mode = ControlModes.MANUAL;

        //only try moving the arm if initilized
        if (specimenLifterInitialized)
        {
            specimenMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            double currentPosition = specimenMotor.getCurrentPosition();
            specimenMotor.setPower(power);
            telemetry.addData("Lifter Position",currentPosition);
            telemetry.addData("Lifter Power",power);
        }
    }

    public void stopLifterWithHold()
    {
        if (specimenLifterInitialized)
        {
            if (robotHardwareMap.mode == ControlModes.MANUAL) {
                //hold code
                double currentPosition = specimenMotor.getCurrentPosition();
                double holdPower = 0;
                specimenMotor.setPower(holdPower);
            }
        }
    }

    public void stopLifter(){
        if (specimenLifterInitialized){
            specimenMotor.setPower(0);
        }
    }

    public void addLifterTelemetry()
    {
        if (specimenLifterInitialized){
            telemetry.addData("Specimen Lifter Encoder:", specimenMotor.getCurrentPosition());
        } else {
            telemetry.addData("Specimen Lifter Encoder:", "Uninitialized!");
        }
    }
}