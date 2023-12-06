package org.firstinspires.ftc.teamcode.AutoStages;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Utils.TeamElementFinder;
import org.firstinspires.ftc.teamcode.Utils.Vector2D;

public class AutoStageConstantsTable {
    public final Robot.Side allianceSide;
    public final double startingRobotFacing;
    public final Vector2D scanTeamElementPosition, teamElementLinePositionLeft, teamElementLinePositionRight, teamElementLinePositionCenter,
            lowestHorizontalWalkWayAndOutMostVerticalWalkWayCross, lowestHorizontalWalkWayAndInnerVerticalWalkWayCross,
            aimWallSweetSpot,
            pixelStashOuter, pixelStashMiddle, pixelStashInner;

    public AutoStageConstantsTable(
            Robot.Side allianceSide,
            double startingRobotFacing,
            Vector2D scanTeamElementPosition, Vector2D teamElementLinePositionLeft, Vector2D teamElementLinePositionRight, Vector2D teamElementLinePositionCenter,
            Vector2D lowestHorizontalWalkWayAndOutMostVerticalWalkWayCross, Vector2D lowestHorizontalWalkWayAndInnerVerticalWalkWayCross,
            Vector2D aimWallSweetSpot, Vector2D pixelStashOuter, Vector2D pixelStashMiddle, Vector2D pixelStashInner) {
        this.allianceSide = allianceSide;
        this.startingRobotFacing = startingRobotFacing;
        this.scanTeamElementPosition = scanTeamElementPosition;
        this.teamElementLinePositionLeft = teamElementLinePositionLeft;
        this.teamElementLinePositionRight = teamElementLinePositionRight;
        this.teamElementLinePositionCenter = teamElementLinePositionCenter;
        this.lowestHorizontalWalkWayAndOutMostVerticalWalkWayCross = lowestHorizontalWalkWayAndOutMostVerticalWalkWayCross;
        this.lowestHorizontalWalkWayAndInnerVerticalWalkWayCross = lowestHorizontalWalkWayAndInnerVerticalWalkWayCross;
        this.aimWallSweetSpot = aimWallSweetSpot;
        this.pixelStashOuter = pixelStashOuter;
        this.pixelStashMiddle = pixelStashMiddle;
        this.pixelStashInner = pixelStashInner;
    }

    public Vector2D getFeedingPosition(TeamElementFinder.TeamElementPosition teamElementPosition) {
        switch (teamElementPosition) {
            case LEFT:
                return teamElementLinePositionLeft;
            case RIGHT:
                return teamElementLinePositionRight;
            case CENTER:
                return teamElementLinePositionCenter;
        }
        return null;
    }
}

