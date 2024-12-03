package org.firstinspires.ftc.teamcode.hardware;

public enum GripperPositions {
    UNKNOWN("UNKNOWN", 0),
    GRIPPER_OPEN("GRIPPER_OPEN", 0.9),
    GRIPPER_CLOSED("GRIPPER_CLOSED", .45),
    GRIPPER_WIDE_OPEN("GRIPPER_WIDE_OPEN",0.75);

    private final String position;
    private final double servoPos;

    GripperPositions(String position, double servoPos){
        this.position = position;
        this.servoPos = servoPos;
    }

    public String getPosition(){
        return position;
    }

    public double getServoPos(){
        return servoPos;
    }
}
