package org.firstinspires.ftc.teamcode.vision

import org.opencv.core.Core
import org.opencv.core.Mat
import org.opencv.core.Point
import org.opencv.core.Scalar
import org.opencv.imgproc.Imgproc
import org.openftc.easyopencv.OpenCvPipeline
import kotlin.math.abs

class TeamElementDetector : OpenCvPipeline() {
    enum class TeamElementPosition {
        LEFT, CENTER, RIGHT
    }

    private val region1: Region = Region(Point(320 / 3 * 0.0, 0.0), 320 / 3.0, 240.0)
    private val region2: Region = Region(Point(320 / 3 * 1.0, 0.0), 320 / 3.0, 240.0)
    private val region3: Region = Region(Point(320 / 3 * 2.0, 0.0), 320 / 3.0, 240.0)

    private val regions: Map<TeamElementPosition, Region> = mapOf(
        TeamElementPosition.LEFT to region1,
        TeamElementPosition.CENTER to region2,
        TeamElementPosition.RIGHT to region3,
    )

    // Because regions are internally linked to their parent mat, it must be initialized here.
    private var parent: Mat = Mat()

    // Volatile since accessed by OpMode thread w/o synchronization
    @Volatile
    private var position = TeamElementPosition.LEFT

    // Converts frame to YCrCb and extracts Cb channel
    private fun toCb(input: Mat, output: Mat) {
        Imgproc.cvtColor(input, output, Imgproc.COLOR_RGB2YCrCb)
        Core.extractChannel(output, output, 2)
    }

    // Converts frame to HSV
    private fun toHSV(input: Mat, output: Mat) {
        Imgproc.cvtColor(input, output, Imgproc.COLOR_RGB2HSV)
    }

    // HSV scoring weights
    private val hsvweights: Array<Double> = arrayOf(1.0, 1.0, 1.0)

    // Score the value based on the target
    private fun score(weights: Array<Double>, value: Scalar, target: Scalar): Double {
        // TODO: See if we can optimize this...
        // Create empty array
        var scores: Array<Double> = Array(weights.size) { 0.0 }
        // For each weight..
        for (i in weights.indices) {
            // Get the difference between the value and the target
            val difference = target.`val`[i] - value.`val`[i]
            // Get the absolute value of the difference and then weight it
            scores[i] = abs(difference) * weights[i]
        }
        // Average the scores together
        return scores.average()
    }

    override fun init(firstFrame: Mat) {
        // Convert to HSV
        toHSV(firstFrame, parent)

        // Initialize the internal submats in the regions
        regions.forEach { it.value.submatOf(parent) }
    }

    override fun processFrame(input: Mat): Mat {
        // Convert frame to HSV
        toHSV(input, parent)

        // Outline all the regions
        regions.forEach { it.value.outline(input) }

        regions.forEach {
            val score = score(hsvweights, it.value.getMean(), Scalar(1.1,0.1,0.0))
            println(score)
        }
        // Get the entry with the highest average
        //val entry = regions.maxByOrNull { it.value.getAverage(0)!! }!!
        //entry.value.highlight(input)

        // Set the position


        // Render input to the viewport, with annotations.
        return input
    }

    // Call this from the OpMode thread to obtain the latest analysis
    fun getAnalysis(): TeamElementPosition {
        return position
    }
}