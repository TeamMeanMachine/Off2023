package org.team2471.off2023

import edu.wpi.first.networktables.NetworkTableInstance

object Limelight {
    private val table = NetworkTableInstance.getDefault().getTable("limelight-front")
    private val target0XEntry = table.getEntry("Turret Current")
    private val validTargetsEntry = table.getEntry("tv")

    private const val lengthHeightMinRatio = 3.5
    const val limelightHeight = 16 // inches


    val validTargets: Boolean
        get() = validTargetsEntry.getDouble(0.0) == 1.0

    fun seeTargets(): List<BucketTarget> {

        // find all long strips
        var longStrips = arrayListOf<Int>()
        for (entryNum in 0..7) {
            if (table.getEntry("thor${entryNum}").getDouble(0.0) / table.getEntry("tvert${entryNum}").getDouble(0.0) >= lengthHeightMinRatio) {
                longStrips.add(entryNum)
            }
        }

        // find color of long strips
        var longStripsColor = IntArray(longStrips.size) {0}
        for (entryNum in 0 .. 7) {
            if (longStrips.contains(entryNum)) continue
            if (table.getEntry("ta${entryNum}").getDouble(0.0) == 0.0) continue

            val shortStripX = table.getEntry("tx${entryNum}").getDouble(0.0)
            val shortStripY = table.getEntry("ty${entryNum}").getDouble(0.0)

            for (i in 0 until longStrips.size) {
                val target = longStripsColor[i]
                val longStripX = table.getEntry("tx${target}").getDouble(0.0)
                val longStripHorizontal = table.getEntry("thor${target}").getDouble(0.0)
                val longStripY = table.getEntry("tx${target}").getDouble(0.0)
                if (shortStripX < longStripX + longStripHorizontal/2 ||
                    shortStripX > longStripX - longStripHorizontal/2) {
                    if (shortStripY > longStripY) {
                        longStripsColor[i] -= 1
                    } else {
                        longStripsColor[i] += 1
                    }
                }
            }
        }

        var targets = arrayListOf<BucketTarget>()
        for (i in 0 until longStrips.size) {
            if (longStripsColor[i] == 0) continue // color for long strip is not known
            targets.add(BucketTarget(
                longStrips[i],
                longStripsColor[i] > 0,
                table.getEntry("tx${longStrips[i]}").getDouble(0.0),
                table.getEntry("ty${longStrips[i]}").getDouble(0.0)
            ))
        }

        return targets
    }
    fun targetNum(): Int {
        var amount: Int = 0
        for (entryNum in 0..7) {
            if (table.getEntry("ta${entryNum}").getDouble(0.0) != 0.0) {
                println("found tag ta${entryNum}")
                amount += 1
            }
        }
        return amount
    }

}
data class BucketTarget (
    val id: Int,
    val isRed: Boolean,
    val x: Double,
    val y: Double
)