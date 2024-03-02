package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;

//@TeleOp(name = "One Gamepad", group = "CenterStage")
@Disabled
public class TeleOp_OneGamepad extends Base {

    // Declare OpMode members for each of the 4 motors.
    boolean wasY = false;
    boolean wasX = false;
    boolean wasLT = false;
    double axial = 0.0;
    double lateral = 0.0;
    double yaw = 0.0;
    boolean TS = false;
    boolean wasTS = false;
    double leftFrontPower = 0.0;
    double leftBackPower = 0.0;
    double rightFrontPower = 0.0;
    double rightBackPower = 0.0;
    double max = 0.0;
    static final double SPEED_MULTIPLIER = 0.75;
    static final double BASE_TURN_SPEED = 2.5;
    double slowdownMultiplier = 0.0;
    static final double CAR_WASH_POWER = 1.0;
    static final double[] BACK_BOUNDS = {0.3, 0.6};

    @Override
    public void runOpMode() {
        setup();
        if (trayTiltingServo != null) {
            trayTiltingServo.setPosition(1);
        }

        while (opModeIsActive()) {
            slowdownMultiplier = (1.0 - gamepad1.right_trigger);

            axial = ((-gamepad1.left_stick_y * SPEED_MULTIPLIER) * slowdownMultiplier);
            lateral = ((gamepad1.left_stick_x * SPEED_MULTIPLIER) * slowdownMultiplier);
            yaw = ((gamepad1.right_stick_x * BASE_TURN_SPEED) * slowdownMultiplier);

            leftFrontPower = axial + lateral + yaw;
            rightFrontPower = axial - lateral - yaw;
            leftBackPower = axial - lateral + yaw;
            rightBackPower = axial + lateral - yaw;

            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }

            // Send calculated power to wheels
            lf.setPower(leftFrontPower);
            rf.setPower(rightFrontPower);
            lb.setPower(leftBackPower);
            rb.setPower(rightBackPower);

            if (pixelLiftingMotor != null) {
                if (!gamepad2.dpad_up && !gamepad2.dpad_down || gamepad2.dpad_down && gamepad2.dpad_up) {
                    pixelLiftingMotor.setPower(0);
                } else {
                    if (touchSensor != null) {
                        TS = touchSensor.isPressed();
                    } else {
                        TS = false;
                        addTelemetry("Touch sensor not connected");
                    }
                    if (gamepad2.dpad_up && !gamepad2.dpad_down) {
                        if (pixelLiftingMotor.getCurrentPosition() < 2450) {
                            pixelLiftingMotor.setPower(0.75);
                            addTelemetry("pixelLiftingMotor now moving");
                        } else {
                            pixelLiftingMotor.setPower(0);
                            addTelemetry("pixelLiftingMotor no longer moving");
                        }
                    }
                    if (gamepad2.dpad_down && !gamepad2.dpad_up && !TS) {
                        if (pixelLiftingMotor.getCurrentPosition() > 0) {
                            pixelLiftingMotor.setPower(-0.75);
                            addTelemetry("pixelLiftingMotor now moving");
                        } else {
                            pixelLiftingMotor.setPower(0);
                            addTelemetry("pixelLiftingMotor no longer moving");
                        }
                    }
                }
            }

            if (carWashMotor != null) {
                if (!gamepad1.a && !gamepad1.b) {
                    carWashMotor.setPower(0);
                } else {
                    if (gamepad1.a) {
                        carWashMotor.setPower(CAR_WASH_POWER);
                        addTelemetry("carWashMotor now moving forward");
                    } else if (gamepad1.b) {
                        carWashMotor.setPower(-CAR_WASH_POWER);
                        addTelemetry("carWashMotor now moving backward");
                    }
                }
            }

            if (trayTiltingServo != null) {
                boolean isLT = (gamepad2.left_trigger > 0.25);
                if (isLT && !wasLT) {
                    if (trayTiltingServo.getPosition() != 0) {
                        trayTiltingServo.setPosition(0);
                        addTelemetry("Set trayTiltingServo to 0");
                    } else {
                        trayTiltingServo.setPosition(1);
                        addTelemetry("Set trayTiltingServo to 1");
                    }
                }
                wasLT = isLT;
            }

            if (pixelBackServo != null) {
                if (gamepad2.y && !wasY) {
                    if (pixelBackServo.getPosition() > BACK_BOUNDS[1] - 0.05 && pixelBackServo.getPosition() < BACK_BOUNDS[1] + 0.05) {
                        pixelBackServo.setPosition(BACK_BOUNDS[0]);
                        addTelemetry("Set pixelBackServo to 0");
                    } else {
                        pixelBackServo.setPosition(BACK_BOUNDS[1]);
                        addTelemetry("Set pixelBackServo to 0.6");
                    }
                }
                wasY = gamepad2.y;
            }

            if (pixelLockingServo != null) {
                if (gamepad2.x && !wasX) {
                    if (pixelLockingServo.getPosition() > 0.83 - 0.05 && pixelLockingServo.getPosition() < 0.83 + 0.05) {
                        pixelLockingServo.setPosition(0.5);
                        addTelemetry("Set pixelFrontServo to 0.5");
                    } else {
                        pixelLockingServo.setPosition(0.83);
                        addTelemetry("Set pixelFrontServo to 0.83");
                    }
                }
                wasX = gamepad2.x;
            }

            if (droneServo != null) {
                if (gamepad1.left_bumper && gamepad1.right_bumper) {
                    droneServo.setPosition(0);
                }
            }
        }

        if (touchSensor != null) {
            if (!wasTS) {
                if (touchSensor.isPressed()) {
                    pixelLiftingMotor.setPower(0);
                    pixelLiftingMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    wasTS = true;
                } else {
                    wasTS = false;
                }
            } else if (!touchSensor.isPressed()) {
                wasTS = false;
            }
        }
    }
    public void addTelemetry(String message){
        telemetry.addData("Last Action",message); // Last Action: message
        telemetry.addData("Pixel Lifting Motor Position", pixelLiftingMotor.getCurrentPosition());
        if(trayTiltingServo == null) {
            telemetry.addData("Tray Tilting Servo", "Disconnected");
        }
        if (pixelLockingServo == null) {
            telemetry.addData("Pixel Front Servo", "Disconnected");
        }
        if (touchSensor == null) {
            telemetry.addData("Touch Sensor", "Disconnected");
        }
        telemetry.update();
    }
}