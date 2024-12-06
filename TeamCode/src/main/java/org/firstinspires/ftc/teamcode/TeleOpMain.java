package org.firstinspires.ftc.teamcode;

import android.graphics.Canvas;
import android.util.Size;
import android.graphics.Color;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.hardware.ArmPositions;
import org.firstinspires.ftc.teamcode.hardware.ArmPositionsPotentiometer;
import org.firstinspires.ftc.teamcode.hardware.ControlModes;
import org.firstinspires.ftc.teamcode.hardware.GripperPositions;
import org.firstinspires.ftc.teamcode.hardware.RobotCameraLight;
import org.firstinspires.ftc.teamcode.hardware.RobotControlArm;
import org.firstinspires.ftc.teamcode.hardware.RobotControlIntake;
import org.firstinspires.ftc.teamcode.hardware.RobotControlLights;
import org.firstinspires.ftc.teamcode.hardware.RobotControlMechanum;
import org.firstinspires.ftc.teamcode.hardware.Light;
import org.firstinspires.ftc.teamcode.hardware.LightMode;
import org.firstinspires.ftc.teamcode.hardware.RobotCamera;
import org.firstinspires.ftc.teamcode.hardware.RobotControlSpecimenLifter;
import org.firstinspires.ftc.teamcode.hardware.RobotControlGripperServo;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.opencv.core.Mat;
import org.opencv.core.RotatedRect;

import java.util.List;


@TeleOp
public class TeleOpMain extends LinearOpMode {

    static final String TAG = "TeleOp Main";

    @Override
    public void runOpMode() throws InterruptedException {
        //Initialize hardware
        RobotHardwareMap theHardwareMap = new RobotHardwareMap(this.hardwareMap, this);
        theHardwareMap.initialize();
        theHardwareMap.mode = ControlModes.MANUAL;

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        telemetry.clearAll();

        RobotControlMechanum robotDrive = new RobotControlMechanum(theHardwareMap, this);
        robotDrive.initialize();

        RobotControlSpecimenLifter specimenLifter = new RobotControlSpecimenLifter(theHardwareMap,this);
        specimenLifter.initialize();

        RobotControlGripperServo gripperServo = new RobotControlGripperServo(theHardwareMap,this);
        gripperServo.initialize();

        RobotControlArm intakeArm = new RobotControlArm(theHardwareMap,this);
        intakeArm.initialize();

        RobotControlIntake intake = new RobotControlIntake(theHardwareMap, this);
        intake.initialize();

        RobotCameraLight cameraLight = new RobotCameraLight(theHardwareMap, this);
        cameraLight.initialize();

        //RobotControlLights lights = new RobotControlLights(theHardwareMap, this);

        //AutonBase autonBase = new AutonBase();  //Do we need this?

        gripperServo.moveToPosition(GripperPositions.GRIPPER_CLOSED);

        waitForStart();

        telemetry.addData("Robot", "Initialized successfully");
        telemetry.update();

        // do something in init mode?
        while (opModeInInit()) {
            telemetry.addData("Robot", "Initialized successfully. Ready to run?");
            telemetry.update();
        }

        //lights.switchLight(Light.ALL, LightMode.OFF);

        //Initialize remaining variables
        double loopTimeStart = 0;
        boolean slowMode = true;
        boolean showTelemetry = true;

        //create some gamepads to look at debouncing
        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();

        double currentClaw = 0.8;
        //Main Loop
        while (opModeIsActive()) {

            loopTimeStart = System.currentTimeMillis();

            //copy over the previous gamepads so we can compare what changed
            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);

            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            //mechanum drive - left stick y is negative because the up is negative
            double drive = -1 * gamepad1.left_stick_y;
            double strafe = -1 * gamepad1.left_trigger + gamepad1.right_trigger;
            //double strafe = gamepad1.left_stick_x;
            double twist = gamepad1.right_stick_x;

            //Speed values for slow mode
            if (slowMode) {
                drive *= 0.45;
                strafe *= 0.45;
                twist *= 0.3;

            } else { // non slow mode is only 75% power
                drive *= 0.7;
                strafe *= 0.7;
                twist *= 0.7;
            }

            robotDrive.teleOpMechanum(drive, strafe, twist);

            //slow mode
            if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper) {
                slowMode = true;
                cameraLight.adjustLight(0);
            } else if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper) {
                slowMode = false;
                cameraLight.adjustLight(0.25);
            }

            //Allow the driver to lower the arm regardless of position and reset the encoder
            if (gamepad1.x)
            {
                intakeArm.moveArmReset(-0.4);
            }


            /*
            //Used to debug wiring/config issues with drive motors
            if (currentGamepad1.y)
            {
                theHardwareMap.backLeftMotor.setPower(.2);
            }
            else
            {
                theHardwareMap.backLeftMotor.setPower(0);
            }
            */

            /***************
             * Gamepad 2
             */


            //Open and close the gripper
            if (currentGamepad2.a)
            {
                gripperServo.moveToPosition(GripperPositions.GRIPPER_OPEN);
            }
            else if (previousGamepad2.a && !currentGamepad2.a)
            {
                gripperServo.moveToPosition(GripperPositions.GRIPPER_CLOSED);
            }

            //Raise/lower the specimen lifter
            if (currentGamepad2.dpad_up)
            {
                specimenLifter.moveLifterPower(0.6);
            }
            else if(previousGamepad2.dpad_up && !currentGamepad2.dpad_up)
            {
                specimenLifter.moveLifterPower(0);
            }
            if (currentGamepad2.dpad_down)
            {
                specimenLifter.moveLifterPower(-0.6);
            }
            else if(previousGamepad2.dpad_down && !currentGamepad2.dpad_down)
            {
                specimenLifter.moveLifterPower(0);
            }

            //arm drive posistion
            if (currentGamepad2.b && !previousGamepad2.b)
            {
                //intakeArm.moveArmEncoded(ArmPositions.DRIVE);
                intakeArm.moveArmPot(ArmPositionsPotentiometer.ARM_DRIVE, intakeArm);
            }

            //Arm up to deliver samples
            if (currentGamepad2.y && !previousGamepad2.y)
            {
                intakeArm.moveArmPot(ArmPositionsPotentiometer.ARM_TOP_DELIVER, intakeArm);
            }

            //Change the intake arm position
            if (currentGamepad2.right_stick_y != 0)
            {
                intakeArm.moveArmPower(-1 * currentGamepad2.right_stick_y);
                telemetry.addData("Arm Encoded Position:",intakeArm.getArmEncodedPosition());
                telemetry.addData("Arm Pot Position:",intakeArm.getCurrentPotPosition());
            }

            //Extend/retract the intake
            if (currentGamepad2.left_stick_y !=0)
            {
                intake.moveIntake(-1 * currentGamepad2.left_stick_y);
                telemetry.addData("Arm Extension:",intake.getIntakeEncodedPosition());
            }

            //Turn on/reverse the intake
            if (currentGamepad2.right_bumper && !previousGamepad2.right_bumper)
            {
                intake.setIntakePower(1.0);
            }

            if(currentGamepad2.left_bumper && !previousGamepad2.left_bumper)
            {
                intake.setIntakePower(-1.0);
            }

            if(currentGamepad2.b && !previousGamepad2.b)
            {
                intake.setIntakePower(0);
            }

            //Drive motor encoder debugging
            if ( 1==0 )
            {
                telemetry.addData("Front Left:", theHardwareMap.frontLeftMotor.getCurrentPosition());
                telemetry.addData("Front Right:", theHardwareMap.frontRightMotor.getCurrentPosition());
                telemetry.addData("Back Left:", theHardwareMap.backLeftMotor.getCurrentPosition());
                telemetry.addData("Back Right:", theHardwareMap.backRightMotor.getCurrentPosition());
            }

            if (showTelemetry = true)
            {
                //telemetry.addData("Lifter Position", specimenLifter.);
                telemetry.update();
            }

        }

        telemetry.addData("Status", "Stopped");
        telemetry.update();
    }
}