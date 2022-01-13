package org.firstinspires.ftc.teamcode.vision

import org.firstinspires.ftc.teamcode.util.Configuration
import org.opencv.core.Core
import org.opencv.core.Mat
import org.opencv.core.Point
import org.opencv.core.Scalar
import org.opencv.imgproc.Imgproc
import org.openftc.easyopencv.OpenCvPipeline
import kotlin.math.abs
import kotlin.math.round

class TeamElementDetector(config: Configuration) : OpenCvPipeline() {
    private val target: Scalar = config.target.toScalar();
    enum class TeamElementPosition {
        LEFT, CENTER, RIGHT
    }

    /*private val region1: Region = Region(Point(320 / 3 * 0.0, 0.0), 320 / 3.0, 240.0)
    private val region2: Region = Region(Point(320 / 3 * 1.0, 0.0), 320 / 3.0, 240.0)
    private val region3: Region = Region(Point(320 / 3 * 2.0, 0.0), 320 / 3.0, 240.0)*/

    /*private val regions: HashMap<TeamElementPosition, Region> = hashMapOf(
        //TeamElementPosition.LEFT to region1,
        TeamElementPosition.CENTER to region2,
        TeamElementPosition.RIGHT to region3,
    )*/

    private val regions = config.regions;

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
    private val weights: Array<Double> = arrayOf(1.0, 1.0, 1.0)

    // Target color
    //private val target: Scalar = Scalar(27.0,222.0,172.0)

    // Threshold: if it is higher than this value, assume the element is off-screen (on the left dot)
    private val threshold = config.threshold;

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

        // Map to store the scores
        val scores: MutableMap<TeamElementPosition, Double> = mutableMapOf(
            //TeamElementPosition.LEFT to 0.0,
            TeamElementPosition.CENTER to 0.0,
            TeamElementPosition.RIGHT to 0.0,
        )

        regions.forEach {
            // TODO: Calibrate this value
            if (it.key != TeamElementPosition.LEFT) {
                val score = score(weights, it.value.getMean(), target)
                scores[it.key] = round(score)
                println("DEBUG: ${it.key} is ${it.value.getMean()} giving it a score of $score")
            }
        }

        // TODO: This may throw a Null pointer exception if it cannot find a minimum.
        val min = scores.minByOrNull { it.value }!!
        println("DEBUG: ${min.key} has the lowest score of ${min.value}")

        // Check if it is on the left dot
        position = if (min.value >= threshold) {
            // If it's not one of these 2, it must be the leftmost one which is offscreen
            TeamElementPosition.LEFT
        } else {
            min.key
        }

        // Highlight the winning position on the camera stream
        regions[position]?.highlight(input)

        // Render input to the viewport, with annotations.
        return input
    }

    // Call this from the OpMode thread to obtain the latest analysis
    fun getAnalysis(): TeamElementPosition {
        return position
    }
}