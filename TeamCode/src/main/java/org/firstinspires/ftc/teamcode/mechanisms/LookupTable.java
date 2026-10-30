package org.firstinspires.ftc.teamcode.mechanisms;

import java.util.Map;
import java.util.TreeMap;

/**
 * Interpolating lookup table, same idea as WPILib's InterpolatingDoubleTreeMap in FRC.
 * put() measured points (e.g. shot distance -> RPM); get() returns a straight-line blend between the two
 * nearest points, and holds the end value outside the measured range.
 * @author: AdithyaGanesan
 */
public class LookupTable {
    private final TreeMap<Double, Double> points = new TreeMap<>();

    public LookupTable put(double key, double value) {
        points.put(key, value);
        return this;
    }

    public double get(double key) {
        if (points.isEmpty()) throw new IllegalStateException("LookupTable has no points");
        Map.Entry<Double, Double> below = points.floorEntry(key);
        Map.Entry<Double, Double> above = points.ceilingEntry(key);
        if (below == null) return above.getValue();
        if (above == null) return below.getValue();
        if (below.getKey().equals(above.getKey())) return below.getValue();
        double t = (key - below.getKey()) / (above.getKey() - below.getKey());
        return below.getValue() + t * (above.getValue() - below.getValue());
    }
}
