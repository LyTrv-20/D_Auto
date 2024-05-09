package CSP_Lib.utils.atlatl.apriltags;

import java.io.IOException;
import java.nio.file.Path;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.wpilibj.Filesystem;

public enum AprilTagLayoutType {
    OFFICIAL("2024-official"),
    SPEAKERS_ONLY("2024-speakers"),
    AMPS_ONLY("2024-amps"),
    WPI("2024-wpi");

    public final AprilTagFieldLayout layout;
    public final String layoutString;

    private AprilTagLayoutType(String name) {
        try {
            layout =
                new AprilTagFieldLayout(
                    Path.of(Filesystem.getDeployDirectory().getPath(), "apriltags", name + ".json"));
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
        if (layout == null) {
            layoutString = "";
        } else {
            try {
                layoutString = new ObjectMapper().writeValueAsString(layout);
            } catch (JsonProcessingException e) {
                throw new RuntimeException(
                    "Failed to serialize AprilTag layout JSON " + toString() + "for Northstar");
            }
        }
    }
}