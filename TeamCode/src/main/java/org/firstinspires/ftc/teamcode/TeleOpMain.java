package org.firstinspires.ftc.teamcode;

import android.util.Log;
import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.hardware.ArmPositions;
import org.firstinspires.ftc.teamcode.hardware.RobotControlFlipperPotentiometer;
import org.firstinspires.ftc.teamcode.hardware.FlipperPotentiometerPositions;
import org.firstinspires.ftc.teamcode.hardware.GripperPositions;
import org.firstinspires.ftc.teamcode.hardware.RobotControlArm;
import org.firstinspires.ftc.teamcode.hardware.RobotControlFlipperMotor;
import org.firstinspires.ftc.teamcode.hardware.RobotControlGripperServos;
import org.firstinspires.ftc.teamcode.hardware.RobotControlLifter;
import org.firstinspires.ftc.teamcode.hardware.RobotControlLights;
import org.firstinspires.ftc.teamcode.hardware.RobotControlMechanum;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.teamcode.hardware.Light;
import org.firstinspires.ftc.teamcode.hardware.LightMode;

import java.util.List;

@TeleOp
public class TeleOpMain extends LinearOpMode {

    static final String TAG = "TeleOp Main";

    @Override
    public void runOpMode() throws InterruptedException {
        //Initialize hardware
        RobotHardwareMap theHardwareMap = new RobotHardwareMap(this.hardwareMap, this);
        theHardwareMap.initialize();

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        RobotControlMechanum robotDrive = new RobotControlMechanum(theHardwareMap, this);
        robotDrive.initialize();

        RobotControlLights lights = new RobotControlLights(theHardwareMap, this);
        RobotControlLifter liftMotor = new RobotControlLifter(theHardwareMap,this);
        RobotControlArm armMotor = new RobotControlArm(theHardwareMap,this);
        RobotControlFlipperMotor flipperMotor = new RobotControlFlipperMotor(theHardwareMap, this);
        RobotControlGripperServos clawServo1 = new RobotControlGripperServos(theHardwareMap, this, "ServoClaw1");
        RobotControlGripperServos clawServo2 = new RobotControlGripperServos(theHardwareMap, this, "ServoClaw2");
        RobotControlGripperServos servoLauncher = new RobotControlGripperServos(theHardwareMap,this,"ServoLauncher");
        RobotControlFlipperPotentiometer robotControlFlipperPotentiometer = new RobotControlFlipperPotentiometer(theHardwareMap, this, "potentiometer");
        AutonBase autonBase = new AutonBase();

        DistanceSensor distanceSensor = theHardwareMap.baseHMap.get(DistanceSensor.class, "distance");
        final int DISTANCE_FROM_BACKBOARD = 7;

        //Set the initial value for the Drone Launcher servo
        servoLauncher.moveToPosition(GripperPositions.DRONE_READY);

        lights.switchLight(Light.ALL, LightMode.GREEN);

        /*FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());*/

        telemetry.addData("Robot", "Initialized successfully");
        telemetry.update();

        // waitForStart();
        AprilTagProcessor aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawTagID(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .build();
        VisionPortal visionPortal = new VisionPortal.Builder()
                .addProcessor(aprilTagProcessor)
                .setCamera(theHardwareMap.frontCamera)
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .enableLiveView(true)
                .setAutoStopLiveView(true)
                .build();


        // do something in init mode?
        while (opModeInInit()) {
            //telemetry.addData("Robot", "Initialized successfully. Ready to run?");
            //telemetry.update();
        }

        telemetry.addData("Robot", "running teleop.. press (Y) For telemetry");
        telemetry.update();

        lights.switchLight(Light.ALL, LightMode.OFF);

        //Initialize remaining variables
        double loopTimeStart = 0;
        boolean slowMode = true;
        lights.switchLight(Light.LED1, LightMode.OFF);
		lights.switchLight(Light.LED2, LightMode.GREEN);
        boolean showTelemetry = false;

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
                drive *= 0.6;
                strafe *= 0.6;
                twist *= 0.6;

            } else { // non slow mode is only 75% power
                drive *= 1;
                strafe *= 1;
                twist *= 1;
            }

            robotDrive.teleOpMechanum(drive, strafe, twist);

            //slow mode
            if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper) {
                slowMode = true;
                lights.switchLight(Light.LED2, LightMode.GREEN);
            } else if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper) {
                slowMode = false;
                lights.switchLight(Light.LED2, LightMode.YELLOW);
            }

            //Drone launch
            if (currentGamepad1.x && !previousGamepad1.x)
            {
                servoLauncher.moveToPosition(GripperPositions.DRONE_LAUNCH);
                telemetry.addData ("Drone Launch",servoLauncher.getCurrentPosition());
            } else if (!currentGamepad1.x && previousGamepad1.x) {
                servoLauncher.moveToPosition(GripperPositions.DRONE_READY);
                telemetry.addData("Drone Reset",servoLauncher.getCurrentPosition());
            }

            //Distance Sensor Alignment
            //TODO: Add functionality with April Tags
            //TODO: Make sure you can run auton functionality in TeleOp
            if (currentGamepad1.a){
                double currentDistance = distanceSensor.getDistance(DistanceUnit.INCH);
                if (currentDistance > DISTANCE_FROM_BACKBOARD){
                    autonBase.imuDrive(0.5, -(currentDistance - DISTANCE_FROM_BACKBOARD), 0);
                }
                else if (currentDistance < DISTANCE_FROM_BACKBOARD){
                    autonBase.imuDrive(0.5, currentDistance - DISTANCE_FROM_BACKBOARD, 0);
                }
                else {
                    autonBase.imuDrive(0, 0, 0);
                }
            }

            //Lifter motor
            if (currentGamepad1.y)
            {
                liftMotor.moveLifterPower(1.0);
            }
            if (currentGamepad1.b)
            {
                liftMotor.moveLifterPower(-1.0);
            }
            if ((!currentGamepad1.y && previousGamepad1.y)
                || (!currentGamepad1.b && previousGamepad1.b))
            {
                liftMotor.stopLifterWithHold();
            }

            //parking brake
            if (currentGamepad1.a && !previousGamepad1.a)
            {
                liftMotor.moveLifterPower(-0.4);
            }

            /***************
             * Gamepad 2
             */

            //Open/close claw1
            if (currentGamepad2.left_bumper)
            {
                clawServo1.moveToPosition(GripperPositions.GRIPPER1_OPEN);
                telemetry.addData("Claw1 Open",clawServo1.getCurrentPosition());
            }
            else if (!currentGamepad2.left_bumper & previousGamepad2.left_bumper)
            {
                clawServo1.moveToPosition(GripperPositions.GRIPPER1_CLOSED);
                telemetry.addData("Claw1 Close",clawServo1.getCurrentPosition());
            }

            //claw 2 controls
            if (currentGamepad2.right_bumper)
            {
                clawServo2.moveToPosition(GripperPositions.GRIPPER2_OPEN);
                telemetry.addData("Claw2 Open",clawServo2.getCurrentPosition());
            }
            else if (!currentGamepad2.right_bumper & previousGamepad2.right_bumper)
            {
                clawServo2.moveToPosition(GripperPositions.GRIPPER2_CLOSED);
                telemetry.addData("Claw2 Close",clawServo2.getCurrentPosition());
            }

            /*if (currentGamepad2.y && previousGamepad2.y){
                currentClaw += 0.05;
                theHardwareMap.servoClaw2.setPosition(currentClaw);
            } else if (currentGamepad2.x && previousGamepad2.x){
                currentClaw -= 0.05;
                theHardwareMap.servoClaw2.setPosition(currentClaw);
            }*/

            //Arm Up/Down
            if (currentGamepad2.left_stick_y != 0)
            {
                armMotor.moveArmPower(-currentGamepad2.left_stick_y);
            } else {
                armMotor.stopArmWithHold();
            }


            if (currentGamepad2.dpad_up && !previousGamepad2.dpad_up){
                armMotor.moveArmEncoded(ArmPositions.FRONT_ARC_ZERO);

            } else if (currentGamepad2.dpad_down && !previousGamepad2.dpad_down){
                armMotor.moveArmEncoded(ArmPositions.FRONT_ARC_MIN);

            }

            //Flipper
            if (currentGamepad2.right_stick_y !=0)
            {
                flipperMotor.moveFlipperPower(-currentGamepad2.right_stick_y);
            } else {
                flipperMotor.stopFlipper();
            }

            //Move the flipper down to pick up 2
            if (currentGamepad2.b && !previousGamepad2.b && (Math.abs((robotControlFlipperPotentiometer.getCurrentPotPosition() - FlipperPotentiometerPositions.CLAW2_DOWN.getVoltagePos())) > 0.1))
            {
//                flipperMotor.moveFlipperEncoded(FlipperMotorPositions.CLAW2_DOWN);
                robotControlFlipperPotentiometer.moveToPosition(FlipperPotentiometerPositions.CLAW2_DOWN, flipperMotor, 0.5);
            }
            if (currentGamepad2.x && !previousGamepad2.x){
//                flipperMotor.moveFlipperEncoded(FlipperMotorPositions.CLAW2_UP);
                robotControlFlipperPotentiometer.moveToPosition(FlipperPotentiometerPositions.CLAW2_PLACE, flipperMotor, 0.5);
            }
            //Move the flipper down to pick up 1
            if (currentGamepad2.a && !previousGamepad2.a  && (Math.abs((robotControlFlipperPotentiometer.getCurrentPotPosition() - FlipperPotentiometerPositions.CLAW1_DOWN.getVoltagePos())) > 0.1))
            {
//                flipperMotor.moveFlipperEncoded(FlipperMotorPositions.CLAW1_DOWN);
                robotControlFlipperPotentiometer.moveToPosition(FlipperPotentiometerPositions.CLAW1_DOWN, flipperMotor, 0.5);
            }
            if (currentGamepad2.y && !previousGamepad2.y){
//                flipperMotor.moveFlipperEncoded(FlipperMotorPositions.CLAW1_UP);
                robotControlFlipperPotentiometer.moveToPosition(FlipperPotentiometerPositions.CLAW1_PLACE, flipperMotor, 0.5);
            }

            double currentArmPosition = armMotor.getArmEncodedPosition();

            if (currentArmPosition <= ArmPositions.FRONT_ARC_TOP.getEncodedPos()){

                if (currentGamepad2.b){
//                flipperMotor.moveFlipperEncoded(FlipperMotorPositions.CLAW2_DOWN);
                    robotControlFlipperPotentiometer.moveToPosition(FlipperPotentiometerPositions.CLAW2_DOWN, flipperMotor, 1);
                }
                else if (currentGamepad2.a){
//                flipperMotor.moveFlipperEncoded(FlipperMotorPositions.CLAW1_DOWN);
                    robotControlFlipperPotentiometer.moveToPosition(FlipperPotentiometerPositions.CLAW1_DOWN, flipperMotor, 1);
                }
            }
            else {
                if (currentGamepad2.b){
//                flipperMotor.moveFlipperEncoded(FlipperMotorPositions.CLAW2_UP);
                    robotControlFlipperPotentiometer.moveToPosition(FlipperPotentiometerPositions.CLAW2_PLACE, flipperMotor, 1);
                }
                else if (currentGamepad2.a){
//                flipperMotor.moveFlipperEncoded(FlipperMotorPositions.CLAW1_UP);
                    robotControlFlipperPotentiometer.moveToPosition(FlipperPotentiometerPositions.CLAW1_PLACE, flipperMotor, 1);
                }

            }


            //Check for detections
            lights.switchLight(Light.LED1, LightMode.OFF);
            //telemetry.clear();

            if (1 == 0) {

                List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();

                //loop through the AprilTag detections to see what we found
                for (AprilTagDetection detection : currentDetections) {
                    //update the lights that we found one
                    lights.switchLight(Light.LED1, LightMode.GREEN);

                    //If we found something, display the data for it
                    if (detection.ftcPose != null) {
                        telemetry.addData("Tag Bearing", detection.ftcPose.bearing);
                        telemetry.addData("Tag Range", detection.ftcPose.range);
                        telemetry.addData("Tag Yaw", detection.ftcPose.yaw);
                        telemetry.addData("ID", detection.id);

                    }
                    //If we detect a specific apriltag and they are pressing X, then we are twisting to that angle
                    if (detection.id == 5) {
                        telemetry.addData("Driveauto", detection.id);
                        double twistAmount = 0.25;
                        //adjust the twist based on the amount of yaw
                        //tweak the color for 5 and or 2
                        //add support for finding 2
                        //test other buttons for ease of use

                        if (detection.ftcPose.yaw >= 0) {
                            twistAmount = twistAmount * -1;
                        }
                        robotDrive.teleOpMechanum(0, 0, twistAmount);
                    }
                }
            }


            armMotor.addArmTelemetry();
            flipperMotor.addFlipperTelemetry();
            telemetry.addData("Pot Position", robotControlFlipperPotentiometer.getCurrentPotPosition());
            telemetry.addData("looptime", System.currentTimeMillis() - loopTimeStart);
            telemetry.addData("Servo 1: ", clawServo1.getCurrentPosition().getServoPos());
            telemetry.addData("Servo 2: ", clawServo2.getCurrentPosition().getServoPos());
            telemetry.addData("Drone Launcher: ", servoLauncher.getCurrentPosition().getServoPos());
            telemetry.update();
        }

        telemetry.addData("Status", "Stopped");
        telemetry.update();

    }
}