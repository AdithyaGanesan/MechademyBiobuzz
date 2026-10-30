package org.firstinspires.ftc.teamcode.mechanisms;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Tracks which cell of OUR hive is facing up, since the aim point jumps to the other cell after every tip.
 * Starts from the match setup, can be flipped by a driver button, and later gets set by the hive camera.
 *
 * Field frame used here (inches, from the official field CAD): origin at field center,
 * +X toward the blue alliance wall, +Y away from the audience, Z up.
 * @author: AdithyaGanesan
 */
public class HiveState implements Subsystem {
    public enum Alliance {RED, BLUE}
    /** Which end of the hive: the cell nearest the audience, or the one nearest the far wall. */
    public enum Cell {AUDIENCE, FAR}
    /** Who last decided which cell is up, shown on telemetry so drivers know how much to trust it. */
    public enum Source {MATCH_START, DRIVER, CAMERA}

    // Hive pivots sit at X = ±12.75. The up cell's opening is ~61" high and ~8.7" from the field centerline.
    private static final double HIVE_X = 12.75;
    private static final double CELL_OPENING_Y = 8.7;
    private static final double CELL_OPENING_Z = 61.1;

    private final Alliance alliance;
    private Cell upCell;
    private Source source = Source.MATCH_START;

    public HiveState(Alliance alliance) {
        this.alliance = alliance;
        // Match setup (field CAD): blue starts with its far cell up, red with its audience cell up.
        upCell = (alliance == Alliance.BLUE) ? Cell.FAR : Cell.AUDIENCE;
    }

    public Cell getUpCell() {return upCell;}

    public Source getSource() {return source;}

    /** Driver button: the hive tipped, so the other cell is now up. */
    public void flip() {
        upCell = (upCell == Cell.FAR) ? Cell.AUDIENCE : Cell.FAR;
        source = Source.DRIVER;
    }

    /** Hive camera saw which cell is up. */
    public void setFromCamera(Cell cell) {
        upCell = cell;
        source = Source.CAMERA;
    }

    /** @return {x, y, z} of the up cell's opening in the field frame above, in inches */
    public double[] getTargetPosition() {
        double x = (alliance == Alliance.BLUE) ? HIVE_X : -HIVE_X;
        double y = (upCell == Cell.FAR) ? CELL_OPENING_Y : -CELL_OPENING_Y;
        return new double[]{x, y, CELL_OPENING_Z};
    }

    /** Nothing to do each loop yet; the hive camera will feed setFromCamera() later. */
    @Override
    public void update() {}

    @Override
    public void addTelemetry(Telemetry telemetry) {
        double[] t = getTargetPosition();
        telemetry.addData("Hive up cell", "%s %s (from %s)", alliance, upCell, source);
        telemetry.addData("Hive target (in)", "x %.1f  y %.1f  z %.1f", t[0], t[1], t[2]);
    }
}
