package org.firstinspires.ftc.teamcode.vision

import org.opencv.core.Core.mean
import org.opencv.core.Mat
import org.opencv.core.Point
import org.opencv.core.Rect
import org.opencv.core.Scalar
import org.opencv.imgproc.Imgproc

class Region(private val rect: Rect) {
    constructor(pointA: Point, pointB: Point) :
            this(Rect(pointA, pointB))

    constructor(pointA: Point, width: Double, height: Double) :
            this(pointA, Point(pointA.x + width, pointA.y + height))

    private lateinit var submat: Mat

    fun submatOf(parent: Mat) {
        submat = parent.submat(rect)
    }

    private fun draw(frame: Mat, color: Scalar, thickness: Int) {
        Imgproc.rectangle(frame, rect, color, thickness)
    }

    fun outline(frame: Mat) {
        draw(frame, Scalar(0.0, 0.0, 255.0), 2)
    }

    fun highlight(frame: Mat) {
        draw(frame, Scalar(0.0, 255.0, 0.0), -1)
    }

    fun getAverage(channel: Int): Double? {
        return mean(submat).`val`.getOrNull(channel)
    }
}