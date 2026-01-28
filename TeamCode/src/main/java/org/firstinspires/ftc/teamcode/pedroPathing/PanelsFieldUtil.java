package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.field.Style;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;

/**
 * Public utility for drawing paths and robot pose on Panels Field.
 * Use from any OpMode to visualize planned paths and current robot pose.
 */
public class PanelsFieldUtil {
    private static final FieldManager field = PanelsField.INSTANCE.getField();

    private static final Style pathStyle = new Style("", "#FF9800", 0.9); // orange
    private static final Style robotStyle = new Style("", "#3F51B5", 0.9); // indigo

    public static void initPedroOffsets() {
        field.setOffsets(PanelsField.INSTANCE.getPresets().getPEDRO_PATHING());
    }

    public static void drawRobot(Follower follower) {
        if (follower == null) return;
        drawRobot(follower.getPose());
    }

    public static void drawRobot(Pose pose) {
        if (pose == null) return;
        field.setStyle(robotStyle);
        field.moveCursor(pose.getX(), pose.getY());
        field.circle(9);

        // heading line
        double hx = Math.cos(pose.getHeading());
        double hy = Math.sin(pose.getHeading());
        double x1 = pose.getX() + hx * 4.5;
        double y1 = pose.getY() + hy * 4.5;
        double x2 = pose.getX() + hx * 9.0;
        double y2 = pose.getY() + hy * 9.0;
        field.setStyle(robotStyle);
        field.moveCursor(x1, y1);
        field.line(x2, y2);
    }

    public static void drawPath(Path path) {
        if (path == null) return;
        double[][] points = path.getPanelsDrawingPoints();
        sanitize(points);
        field.setStyle(pathStyle);
        field.moveCursor(points[0][0], points[0][1]);
        field.line(points[1][0], points[1][1]);
    }

    public static void drawPathChain(PathChain chain) {
        if (chain == null) return;
        for (int i = 0; i < chain.size(); i++) {
            drawPath(chain.getPath(i));
        }
    }

    public static void update() {
        field.update();
    }

    private static void sanitize(double[][] points) {
        if (points == null) return;
        for (int i = 0; i < points[0].length; i++) {
            for (int j = 0; j < points.length; j++) {
                if (Double.isNaN(points[j][i])) points[j][i] = 0;
            }
        }
    }
}
