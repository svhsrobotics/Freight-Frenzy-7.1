package org.firstinspires.ftc.teamcode.vision;

import org.opencv.core.Scalar;

public class HSVColor {
    public Double hue;
    public Double saturation;
    public Double value;

    public HSVColor(Scalar scalar) {
        this.hue = scalar.val[0];
        this.saturation = scalar.val[1];
        this.value = scalar.val[2];
    }

    public HSVColor(Double hue, Double saturation, Double value) {
        this.hue = hue;
        this.saturation = saturation;
        this.value = value;
    }

    public Scalar toScalar() {
        return new Scalar(hue, saturation, value);
    }
}
