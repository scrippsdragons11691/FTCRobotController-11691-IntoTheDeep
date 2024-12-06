package org.firstinspires.ftc.teamcode.hardware;

public enum ArmPositionsPotentiometer {

    MIN_VOLTAGE("MIN_VOLTAGE", 0),
    ARM_DRIVE("ARM_DRIVE", .45),
    ARM_TOP_DELIVER("ARM_TOP_DELIVER", 1.45),
    MAX_VOLTAGE("MAX_VOLTAGE", 4);

    private final String position;
    private final double voltagePos;

    ArmPositionsPotentiometer(String position, double voltagePos){
        this.position = position;
        this.voltagePos = voltagePos;
    }

    public String getPosition(){
        return position;
    }

    public double getVoltagePos(){
        return voltagePos;
    }
}
