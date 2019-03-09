package frc.utility;

public class VisionTarget {

    public float x;
    public float y;
    public float connectorMag;
    public float distance;

    public VisionTarget(float x, float y, float connectorMag, float distance) {
        this.x = x;
        this.y = y;
        this.connectorMag = connectorMag;
        this.distance = distance;
    }

    public float getX() {
        return this.x;
    }
    
    public float getY() {
        return this.y;
    }

    public float getMag() {
        return this.connectorMag;
    }

    public float getDistance() {
        return this.distance;
    }

}