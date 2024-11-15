
package org.firstinspires.ftc.teamcode.hardware;

public enum ArmPositions {
    UNKNOWN("UNKNOWN", -1),
    DRIVE("DRIVE",250),
    PICKUP("PICKUP",70),
    DELIVER("DELIVER",100);

    private final String position;
    private final int encodedPos;

    ArmPositions(String position, int encodedPos){
        this.position = position;
        this.encodedPos = encodedPos;
    }

    public String getPosition(){
        return position;
    }

    public int getEncodedPos(){
        return encodedPos;
    }
}
