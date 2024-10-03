package org.firstinspires.ftc.teamcode.hardware;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotHardwareMap;

public class RobotControlLights {

    static final String TAG = "RobotControlLights";
    RobotHardwareMap theHardwareMap;
    LinearOpMode opMode;

    int sequenceCounter = 0;
    LightSequence currentSequence = LightSequence.NONE;

    Light currentLights = Light.ALL;
    LightMode currentMode = LightMode.OFF;
    LightMode currentLightStep = LightMode.OFF;

    RobotControlLight leftRedLED;
    RobotControlLight leftGreenLED;
    RobotControlLight rightRedLED;
    RobotControlLight rightGreenLED;
    RobotControlLight frontLeftRedLED;
    RobotControlLight frontLeftGreenLED;
    RobotControlLight frontRightRedLED;
    RobotControlLight frontRightGreenLED;

    RobotControlLight LED1Red;
    RobotControlLight LED1Green;
    RobotControlLight LED2Red;
    RobotControlLight LED2Green;

    public enum LightSequence {
        NONE, ALTERNATE_GREEN, ALTERNATE_RED
    }

    public RobotControlLights(RobotHardwareMap theHardwareMap, LinearOpMode opMode) {
        this.opMode = opMode;
        this.theHardwareMap = theHardwareMap;
        initialize();
    }

    //LED1green, LED1red, LED2green, LED2red
    public void initialize(){
        Log.d(TAG, "Lights init");
        leftRedLED = new RobotControlLight(theHardwareMap, opMode, "leftred");
        leftGreenLED = new RobotControlLight(theHardwareMap, opMode, "leftgreen");
        rightRedLED = new RobotControlLight(theHardwareMap, opMode, "rightred");
        rightGreenLED = new RobotControlLight(theHardwareMap, opMode, "rightgreen");
        frontLeftRedLED = new RobotControlLight(theHardwareMap, opMode, "flRed");
        frontLeftGreenLED = new RobotControlLight(theHardwareMap, opMode, "flGreen");
        frontRightRedLED = new RobotControlLight(theHardwareMap, opMode, "frRed");
        frontRightGreenLED = new RobotControlLight(theHardwareMap, opMode, "frGreen");

        LED1Red = new RobotControlLight(theHardwareMap, opMode, "LED1red");
        LED1Green = new RobotControlLight(theHardwareMap, opMode, "LED1green");
        LED2Red = new RobotControlLight(theHardwareMap, opMode, "LED2red");
        LED2Green = new RobotControlLight(theHardwareMap, opMode, "LED2green");

    }

    /***
     * Used to switch on and off lights
     * @param light light to change mode
     * @param lightMode whether to turn on or off
     */
    public void switchLight( Light light, LightMode lightMode){
        boolean lightBoolGreen = true;
        boolean lightBoolRed = true;

        switch (lightMode){
            case OFF:
                lightBoolGreen = true;
                lightBoolRed = true;
                break;

            case RED:
                lightBoolGreen = false;
                lightBoolRed = true;
                break;

            case GREEN:
                lightBoolGreen = true;
                lightBoolRed = false;
                break;

            case YELLOW:
                lightBoolGreen = false;
                lightBoolRed = false;
                break;
        }

        switch (light){

            case LED1:
                LED1Red.setState(lightBoolRed);
                LED1Green.setState(lightBoolGreen);
                break;

            case LED2:
                LED2Red.setState(lightBoolRed);
                LED2Green.setState(lightBoolGreen);
                break;

            case BACKLEFT:
                leftRedLED.setState(lightBoolRed);
                leftGreenLED.setState(lightBoolGreen);
                break;

            case BACKRIGHT:
                rightRedLED.setState(lightBoolRed);
                rightGreenLED.setState(lightBoolGreen);
                break;

            case FRONTLEFT:
                frontLeftGreenLED.setState(lightBoolGreen);
                frontLeftRedLED.setState(lightBoolRed);
                break;

            case FRONTRIGHT:
                frontRightGreenLED.setState(lightBoolGreen);
                frontRightRedLED.setState(lightBoolRed);
                break;

            case ALL_RIGHT:
                rightGreenLED.setState(lightBoolGreen);
                rightRedLED.setState(lightBoolRed);
                frontRightGreenLED.setState(lightBoolGreen);
                frontRightRedLED.setState(lightBoolRed);
                break;

            case ALL_LEFT:
                leftGreenLED.setState(lightBoolGreen);
                leftRedLED.setState(lightBoolRed);
                frontLeftGreenLED.setState(lightBoolGreen);
                frontLeftRedLED.setState(lightBoolRed);
                break;

            case ALL_BACK:
                rightGreenLED.setState(lightBoolGreen);
                rightRedLED.setState(lightBoolRed);
                leftGreenLED.setState(lightBoolGreen);
                leftRedLED.setState(lightBoolRed);
                break;

            case ALL_FRONT:
                frontRightGreenLED.setState(lightBoolGreen);
                frontRightRedLED.setState(lightBoolRed);
                frontLeftGreenLED.setState(lightBoolGreen);
                frontLeftRedLED.setState(lightBoolRed);
                break;

            case ALL:
                frontRightGreenLED.setState(lightBoolGreen);
                frontRightRedLED.setState(lightBoolRed);
                frontLeftGreenLED.setState(lightBoolGreen);
                frontLeftRedLED.setState(lightBoolRed);
                rightGreenLED.setState(lightBoolGreen);
                rightRedLED.setState(lightBoolRed);
                leftGreenLED.setState(lightBoolGreen);
                leftRedLED.setState(lightBoolRed);
                LED1Green.setState(lightBoolGreen);
                LED1Red.setState(lightBoolRed);
                LED2Green.setState(lightBoolGreen);
                LED2Red.setState(lightBoolRed);
                break;

        }

    }

    /**
     * rotate between off, red, yellow, green, off...
     * @param changedLight
     */
    public void doLightSteps(Light changedLight) {
        LightMode newLightStep = LightMode.RED;

        if (currentLightStep == LightMode.OFF) {
            newLightStep = LightMode.RED;
        } else if (currentLightStep == LightMode.RED) {
            newLightStep = LightMode.YELLOW;
        } else if (currentLightStep == LightMode.YELLOW){
            newLightStep = LightMode.GREEN;
        } else if (currentLightStep == LightMode.GREEN){
            newLightStep = LightMode.OFF;
        }

        switchLight(changedLight, newLightStep);
        currentLightStep = newLightStep;
    }

    public void runLightSeq(LightSequence lightSeq, int times){
        currentSequence = lightSeq;
        sequenceCounter = times;
    }

    private void mainLoop(){
        if (sequenceCounter > 0){
            Log.d(TAG, "light seq:" + sequenceCounter);
            if (currentSequence == LightSequence.ALTERNATE_GREEN){
                if (sequenceCounter % 2 == 0) {

                } else if (sequenceCounter == 1){
                    leftGreenLED.setState(true);
                    rightGreenLED.setState(true);
                } else {
                    rightGreenLED.setState(true);
                    leftGreenLED.setState(false);
                }
            } else if (currentSequence == LightSequence.ALTERNATE_RED){
                if (sequenceCounter % 2 == 0) {
                    rightRedLED.setState(false);
                    leftRedLED.setState(true);
                } else if (sequenceCounter == 1){
                    rightRedLED.setState(true);
                    leftRedLED.setState(true);
                } else {
                    rightRedLED.setState(true);
                    leftRedLED.setState(false);
                }
            }
            sequenceCounter--;

        }

    }

}
