package CSP_Lib.utils.atlatl;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;

public class CameraLocation {
    public final Pose3d cameraMounting;
    public final Rotation2d gyroAngle;

    public CameraLocation(Pose3d cameraMounting, Rotation2d gyroAngle) {
        this.cameraMounting = cameraMounting;
        this.gyroAngle = gyroAngle;
    }
}