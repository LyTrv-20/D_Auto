package CSP_Lib.utils.atlatl;

import edu.wpi.first.math.geometry.Translation2d;

/** Class to represent the result of this algorithm */
public class LocationResult {
    public final Translation2d estimation, stdDevs;

    public LocationResult(Translation2d estimation, Translation2d stdDevs) {
        this.estimation = estimation;
        this.stdDevs = stdDevs;
    }
}