
package org.firstinspires.ftc.teamcode.hardware;

public enum LifterPositions {
    UNKNOWN("UNKNOWN", -1),
    TOP("TOP",7300),
    TOP_DELIVER("TOP_DELIVER",5325),
    BOTTOM ("BOTTOM",0),
    PICKUP("PICKUP",525);

    private final String position;
    private final int encodedPos;

    LifterPositions(String position, int encodedPos){
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
