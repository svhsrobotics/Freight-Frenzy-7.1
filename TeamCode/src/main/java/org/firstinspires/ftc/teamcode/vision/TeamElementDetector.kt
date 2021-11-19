package org.firstinspires.ftc.teamcode.vision

import org.opencv.core.*
import org.opencv.imgproc.Imgproc
import org.openftc.easyopencv.OpenCvPipeline

class TeamElementDetector : OpenCvPipeline() {
    enum class TeamElementPosition {
        LEFT, CENTER, RIGHT
    }

    private val region1: Region = Region(Point(320 / 3 * 0.0, 0.0), 320 / 3.0, 240.0)
    private val region2: Region = Region(Point(320 / 3 * 1.0, 0.0), 320 / 3.0, 240.0)
    private val region3: Region = Region(Point(320 / 3 * 2.0, 0.0), 320 / 3.0, 240.0)

    private var Cb: Mat = Mat()

    // Volatile since accessed by OpMode thread w/o synchronization
    @Volatile
    private var position = TeamElementPosition.LEFT

    // Converts frame to YCrCb and extracts Cb channel
    private fun Mat.toCb(): Mat {
        var YCrCb: Mat = Mat()
        var Cb: Mat = Mat()
        Imgproc.cvtColor(this, YCrCb, Imgproc.COLOR_RGB2YCrCb)
        Core.extractChannel(YCrCb, Cb, 2)
        return Cb
    }

    override fun init(firstFrame: Mat) {
        // Get the Cb channel from the frame
        Cb = firstFrame.toCb()

        // Initialize the internal submats in the regions
        region1.init(Cb)
        region2.init(Cb)
        region3.init(Cb)
    }

    override fun processFrame(input: Mat): Mat {
        // Get the Cb channel of the input frame after conversion to YCrCb
        Cb = input.toCb()

        region1.outline(input)
        region2.outline(input)
        region3.outline(input)

        // Find the max of the 3 averages
        when (maxOf(region1.average, region2.average, region3.average)) {
            region1.average -> {
                position = TeamElementPosition.LEFT
                region1.highlight(input)
            }
            region2.average -> {
                position = TeamElementPosition.CENTER
                region2.highlight(input)
            }
            region3.average -> {
                position = TeamElementPosition.RIGHT
                region3.highlight(input)
            }
        }

        // Render input to the viewport, with annotations.
        return input
    }

    // Call this from the OpMode thread to obtain the latest analysis
    fun getAnalysis(): TeamElementPosition {
        return position
    }
}