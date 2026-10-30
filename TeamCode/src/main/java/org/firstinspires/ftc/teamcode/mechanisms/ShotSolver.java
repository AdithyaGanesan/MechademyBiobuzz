package org.firstinspires.ftc.teamcode.mechanisms;

/**
 * Pure aiming math, no hardware: robot pose + target position in, turret angle and shot settings out.
 * Uses the same field frame as HiveState (inches, origin at field center, +X toward blue, +Y away from
 * the audience). Heading is degrees counterclockwise from +X, same as the turret's angle convention.
 * @author: AdithyaGanesan
 */
public final class ShotSolver {
    // CAD: turret axis sits 20 mm ahead of the drive center, centered side to side.
    private static final double TURRET_FORWARD_OFFSET_IN = 20 / 25.4;

    // Shot tables: horizontal distance (in) from turret to target -> flywheel RPM / hood angle.
    // PLACEHOLDERS, replace with test-shot results (add rows in any order).
    private static final LookupTable RPM_TABLE = new LookupTable()
            .put(24, 2800)
            .put(48, 3200)
            .put(72, 3600)
            .put(96, 4100)
            .put(120, 4600);
    private static final LookupTable HOOD_DEG_TABLE = new LookupTable()
            .put(24, 15)
            .put(48, 15)
            .put(72, 15)
            .put(96, 15)
            .put(120, 15);

    private ShotSolver() {}

    /** Everything the turret and shooter need for one shot. */
    public static final class Shot {
        /** Robot-relative turret angle, degrees, 0 = robot front, counterclockwise positive. */
        public final double turretAngleDeg;
        /** Horizontal distance from turret to target, inches. */
        public final double distanceIn;
        public final double rpm;
        public final double hoodDeg;

        Shot(double turretAngleDeg, double distanceIn, double rpm, double hoodDeg) {
            this.turretAngleDeg = turretAngleDeg;
            this.distanceIn = distanceIn;
            this.rpm = rpm;
            this.hoodDeg = hoodDeg;
        }
    }

    /**
     * @param robotX         robot center X, inches
     * @param robotY         robot center Y, inches
     * @param robotHeadingDeg robot heading, degrees counterclockwise from +X
     * @param target         {x, y, z} target in inches, e.g. HiveState.getTargetPosition()
     */
    public static Shot solve(double robotX, double robotY, double robotHeadingDeg, double[] target) {
        double h = Math.toRadians(robotHeadingDeg);
        // The turret isn't at the robot's center, so aim from where the turret actually is.
        double turretX = robotX + TURRET_FORWARD_OFFSET_IN * Math.cos(h);
        double turretY = robotY + TURRET_FORWARD_OFFSET_IN * Math.sin(h);

        double dx = target[0] - turretX;
        double dy = target[1] - turretY;
        double fieldAngleDeg = Math.toDegrees(Math.atan2(dy, dx));
        // Field direction minus where the robot faces = direction relative to the robot.
        // Turret.setTargetAngle() handles wrapping past ±180.
        double turretAngleDeg = fieldAngleDeg - robotHeadingDeg;
        double distanceIn = Math.hypot(dx, dy);

        return new Shot(turretAngleDeg, distanceIn, RPM_TABLE.get(distanceIn), HOOD_DEG_TABLE.get(distanceIn));
    }
}
