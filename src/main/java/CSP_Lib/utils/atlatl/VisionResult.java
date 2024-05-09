package CSP_Lib.utils.atlatl;

/** Class to represent the limelight's vision measurement for a specified AprilTag */
public class VisionResult {
    public final double tx, ty, stdDev;
    public final int id;

    /**
     * Constructs a new {@link VisionResult} object.
     * @param tx The limelight tx value of the tag.
     * @param ty The limelight ty value of the tag.
     * @param id The ID of the tag being seen.
     */
    public VisionResult(double tx, double ty, double stdDev, int id) {
        this.id = id;
        this.tx = tx;
        this.ty = ty;
        this.stdDev = stdDev;
    }
}