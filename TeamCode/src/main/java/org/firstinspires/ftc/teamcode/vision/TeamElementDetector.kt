package org.firstinspires.ftc.teamcode.vision

import org.opencv.core.Core
import org.opencv.core.Mat
import org.opencv.core.Point
import org.opencv.imgproc.Imgproc
import org.openftc.easyopencv.OpenCvPipeline

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

    override fun init(firstFrame: Mat) {
        // Get the Cb channel from the frame
        toCb(firstFrame, parent)

        // Initialize the internal submats in the regions
        regions.forEach { it.value.submatOf(parent) }
    }

    override fun processFrame(input: Mat): Mat {
        // Get the Cb channel of the input frame after conversion to YCrCb
        toCb(input, parent)

        // Outline all the regions
        regions.forEach { it.value.outline(input) }

        // Get the entry with the highest average
        val entry = regions.maxByOrNull { it.value.getAverage(0)!! }!!
        entry.value.highlight(input)

        // Set the position
        position = entry.key

        // Render input to the viewport, with annotations.
        return input
    }

    // Call this from the OpMode thread to obtain the latest analysis
    fun getAnalysis(): TeamElementPosition {
        return position
    }
}