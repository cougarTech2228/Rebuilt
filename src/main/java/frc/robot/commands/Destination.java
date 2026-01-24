package frc.robot.commands;

import static frc.robot.subsystems.vision.VisionConstants.aprilTagLayout;
import frc.robot.Constants;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;

/**
 *  We have to declare these in a class if we want to use them in the constructor of Destination objects
 *  Thanks Java....
 */
class DestConsts {
    static double loaderCenterOffset = -0.16;
    static double loaderRightOffset = loaderCenterOffset + 0.6096;
    static double loaderLeftOffset = loaderCenterOffset - 0.6096;
    static double loaderXOffset = -0.07;

    static double reefLeftOffset = -0.33;

    static double angleTolerance = 60; // how close to destination angle to be in zone (degrees)
}

/**
 * Destinations for short autonomous paths
 */
public enum Destination {
    // All defined here relative to Blue. All functions below will mirror as needed.
    // Currently approximations (most destinations cause clipping with DuckBot in sim)

    TOWER(31,0,1.144, 0,0, 100,0, 100,100, 0,100),
    DEPOT(31, 2.217,0.7,0,0,100,0,100,100,0,100),
    OUTPOST(29, 0,0,0,0,0,100,100,100,100,0),
    PROCESSOR(25,0,0,0,0,0,100,100,100,100,0);

    // x, y, angle of the destination
    private final Pose2d pose;
    private final Pose2d tagPose;

    // vertices of the quad in which the destination is possible (???)
    private final ArrayList<Translation2d> zone;
    private final double yShift;
    private final double xShift;

    private Destination(double x, double y, double angle, double p1x, double p1y, double p2x, double p2y, double p3x,
            double p3y, double p4x, double p4y) {
        this.pose = new Pose2d(x, y, Rotation2d.fromDegrees(angle));
        this.tagPose = pose;
        this.zone = new ArrayList<>(); // What is this zone for?
        zone.add(new Translation2d(p1x, p1y));
        zone.add(new Translation2d(p2x, p2y));
        zone.add(new Translation2d(p3x, p3y));
        zone.add(new Translation2d(p4x, p4y));
        yShift = 0;
        xShift = 0;
    }

    private Destination(int tagID, double yShift, double xShift, double p1x, double p1y, double p2x, double p2y, double p3x,
    double p3y, double p4x, double p4y) {
        tagPose = aprilTagLayout.getTagPose(tagID).get().toPose2d();
        this.pose = tagPose.transformBy(new Transform2d(Constants.robotLength/2 + xShift, yShift, new Rotation2d(Math.PI)));
        this.zone = new ArrayList<>();
        zone.add(new Translation2d(p1x, p1y));
        zone.add(new Translation2d(p2x, p2y));
        zone.add(new Translation2d(p3x, p3y));
        zone.add(new Translation2d(p4x, p4y));
        this.yShift = yShift;
        this.xShift = xShift;
    }

    public Rotation2d getAngle(DriverStation.Alliance alliance) {
        return alliance == DriverStation.Alliance.Blue ? pose.getRotation()
                : pose.getRotation().plus(Rotation2d.k180deg);
    }

    public Pose2d getPose() {
        return pose;
    }

    public Pose2d getApproachPose() {
        // return pose.transformBy(new Transform2d(-Constants.robotLength/2, xShift, new Rotation2d()));
        return tagPose.transformBy(new Transform2d(Constants.robotLength + xShift, yShift, new Rotation2d(Math.PI)));
    }

    public boolean inZone(Pose2d pos, DriverStation.Alliance alliance) {
        // if the robot is more than 90 degrees away from the right angle, it's not in the zone.
        double destAngle = getAngle(alliance).getDegrees();
        double robAngle = pos.getRotation().getDegrees();
        double angleDiff = (robAngle-destAngle+720) % 360; //sorry
        if (angleDiff > DestConsts.angleTolerance && angleDiff < 360-DestConsts.angleTolerance)
            return false;
        
        // for the given pose, is it in the trapezoid?
        // n>2 Keep track of cross product sign changes
        int pve = 0;
        int neg = 0;

        double x = pos.getX();
        double y = pos.getY();

        for (int i = 0; i < 4; i++) {
            // Form a segment between the i'th point
            double x1 = zone.get(i).getX();
            if (alliance == DriverStation.Alliance.Red)
                x1 = 17.55 - x1;
            double y1 = zone.get(i).getY();
            if (alliance == DriverStation.Alliance.Red)
                y1 = 8.05 - y1;

            // And the i+1'th, or if i is the last, with the first point
            int i2 = (i + 1) % 4;

            double x2 = zone.get(i2).getX();
            if (alliance == DriverStation.Alliance.Red)
                x2 = 17.55 - x2;
            double y2 = zone.get(i2).getY();
            if (alliance == DriverStation.Alliance.Red)
                y2 = 8.05 - y2;

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