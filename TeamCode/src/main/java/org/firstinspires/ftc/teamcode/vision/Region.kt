package org.firstinspires.ftc.teamcode.vision

import org.opencv.core.*
import org.opencv.imgproc.Imgproc

class Region(private val pointA: Point, private val pointB: Point) {
    constructor(pointA: Point, width: Double, height: Double) :
            this(pointA, Point(pointA.x + width, pointA.y + height))

    private val blue = Scalar(0.0, 0.0, 255.0)
    private val green = Scalar(0.0, 255.0, 0.0)

    private lateinit var submat: Mat

    val average: Double
        get() = Core.mean(submat).`val`[0]

    fun init(parent: Mat) {
        submat = parent.submat(Rect(pointA, pointB))
    }

    private fun draw(frame: Mat, color: Scalar, thickness: Int) {
        Imgproc.rectangle(frame, pointA, pointB, color, thickness)
    }

    fun outline(frame: Mat) {
        draw(frame, blue, 2)
    }

    fun highlight(frame: Mat) {
        draw(frame, green, -1)
    }
}