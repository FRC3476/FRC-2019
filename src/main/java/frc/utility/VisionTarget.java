package frc.utility;

public class VisionTarget {

    public float x;
    public float y;
    public float connectorMag;

    public VisionTarget(float x, float y, float connectorMag) {
        this.x = x;
        this.y = y;
        this.connectorMag = connectorMag;
    }

    float getX() {
        return this.x;
    }
    
    float getY() {
        return this.y;
    }

    float getMag() {
        return this.connectorMag;
    }

}