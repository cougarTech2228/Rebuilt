package frc.robot.commands;

import static frc.robot.subsystems.vision.VisionConstants.aprilTagLayout;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;

/**
 *  We have to declare these in a class if we want to use them in the constructor of Destination objects
 *  Thanks Java....
 */
class DestConsts {
    static double field_length = aprilTagLayout.getFieldLength();
    static double field_height = aprilTagLayout.getFieldWidth();
    static double alliance_zone_length = 3.963924;
}

public enum Zone {
    // All defined here relative to Blue. All functions below will mirror as needed.

    HOME_ALLIANCE_ZONE(
        0, 0,
        DestConsts.alliance_zone_length, 0,
        DestConsts.alliance_zone_length, DestConsts.field_height,
        0, DestConsts.field_height,
        false),

    NEUTRAL_ZONE_SOUTH(
        DestConsts.alliance_zone_length, 0,
        DestConsts.field_length, 0,
        DestConsts.field_length, DestConsts.field_height/2,
        DestConsts.alliance_zone_length, DestConsts.field_height/2,
        false),

    NEUTRAL_ZONE_NORTH(
        DestConsts.alliance_zone_length, DestConsts.field_height/2,
        DestConsts.field_length, DestConsts.field_height/2,
        DestConsts.field_length, DestConsts.field_height,
        DestConsts.alliance_zone_length, DestConsts.field_height,
        false);

    // vertices of the quad in which the destination is possible
    private final ArrayList<Translation2d> zone;
    private final boolean invert_height_when_red;

    private Zone(double p1x, double p1y, double p2x, double p2y, double p3x,
            double p3y, double p4x, double p4y, boolean invert_height_when_red) {

        zone = new ArrayList<>();
        zone.add(new Translation2d(p1x, p1y));
        zone.add(new Translation2d(p2x, p2y));
        zone.add(new Translation2d(p3x, p3y));
        zone.add(new Translation2d(p4x, p4y));
        this.invert_height_when_red = invert_height_when_red;
    }

    public boolean inZone(Pose2d pos, DriverStation.Alliance alliance) {
        int pve = 0;
        int neg = 0;

        double x = pos.getX();
        double y = pos.getY();

        for (int i = 0; i < 4; i++) {
            // Form a segment between the i'th point
            double x1 = zone.get(i).getX();
            if (alliance == DriverStation.Alliance.Red)
                x1 = DestConsts.field_length - x1;
            double y1 = zone.get(i).getY();

            if (invert_height_when_red && alliance == DriverStation.Alliance.Red)
                y1 = DestConsts.field_height - y1;

            // And the i+1'th, or if i is the last, with the first point
            int i2 = (i + 1) % 4;

            double x2 = zone.get(i2).getX();
            if (alliance == DriverStation.Alliance.Red)
                x2 = DestConsts.field_length - x2;
            double y2 = zone.get(i2).getY();
            if (invert_height_when_red && alliance == DriverStation.Alliance.Red)
                y2 = DestConsts.field_height - y2;

            // Compute the cross product
            double d = (x - x1) * (y2 - y1) - (y - y1) * (x2 - x1);

            if (d > 0)
                pve++;
            if (d < 0)
                neg++;

            // If the sign changes, then point is outside
            if (pve > 0 && neg > 0)
                return false;
        }

        // If no change in direction, then on same side of all segments, and thus inside
        return true;
    }
}