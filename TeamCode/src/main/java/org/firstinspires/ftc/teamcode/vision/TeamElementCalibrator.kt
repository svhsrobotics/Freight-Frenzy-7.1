package org.firstinspires.ftc.teamcode.vision

import org.opencv.core.Mat
import org.opencv.core.Point
import org.opencv.core.Scalar
import org.opencv.imgproc.Imgproc
import org.openftc.easyopencv.OpenCvPipeline

class TeamElementCalibrator : OpenCvPipeline() {
    private val region: Region = Region(Point(320 / 3 * 1.0, 0.0), 320 / 3.0, 240.0)

    // Because regions are internally linked to their parent mat, it must be initialized here.
    private var parent: Mat = Mat()

    // Volatile since accessed by OpMode thread w/o synchronization
    @Volatile
    private lateinit var color: Scalar;

    // Converts frame to HSV
    private fun toHSV(input: Mat, output: Mat) {
        Imgproc.cvtColor(input, output, Imgproc.COLOR_RGB2HSV)
    }

    override fun init(firstFrame: Mat) {
        // Convert to HSV
        toHSV(firstFrame, parent)

        // Initialize the internal submat in the region
        region.submatOf(parent);
    }

    override fun processFrame(input: Mat): Mat {
        // Convert frame to HSV
        toHSV(input, parent)

        // Outline the region
        region.outline(input)

        color = region.getMean()

        println("DEBUG: Region is $color")

        // Render input to the viewport, with annotations.
        return input
    }

    // Call this from the OpMode thread to obtain the latest analysis
    fun getAnalysis(): Scalar {
        return color
    }
}