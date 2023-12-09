package org.firstinspires.ftc.teamcode.AutoStages;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Utils.TeamElementFinder;
import org.firstinspires.ftc.teamcode.Utils.Vector2D;

public class AutoStageConstantsTable {
    public final Robot.Side allianceSide;
    public final boolean backField;
    public final double startingRobotFacing;
    public final double centerTeamElementRotation;
    public final double releasePixelRotationToCenterLineAngle;
    public final double centerLineYPosition; // field center line, the horizontal line that separates the front and back field
    public final Vector2D scanTeamLeftRightElementPosition, scanTeamCenterElementPosition, teamElementLinePositionLeft, teamElementLinePositionRight, teamElementLinePositionCenter,
            lowestHorizontalWalkWayAndOutMostVerticalWalkWayCross, lowestHorizontalWalkWayAndInnerVerticalWalkWayCross,
            aimWallSweetSpot,
            pixelStashOuter, pixelStashMiddle, pixelStashInner,
            parkingPosition;

    public AutoStageConstantsTable(
            Robot.Side allianceSide,
            boolean backField,
            double startingRobotFacing,
            double centerTeamElementRotation,
            double releasePixelRotationToCenterLineAngle, // angle between the center line of team element and the rotation in which robot release the pixel
            double centerLineYPosition,
            Vector2D scanTeamLeftRightElementPosition, Vector2D scanTeamCenterElementPosition, Vector2D teamElementLinePositionLeft, Vector2D teamElementLinePositionCenter, Vector2D teamElementLinePositionRight,
            Vector2D lowestHorizontalWalkWayAndOutMostVerticalWalkWayCross, Vector2D lowestHorizontalWalkWayAndInnerVerticalWalkWayCross,
            Vector2D aimWallSweetSpot, Vector2D pixelStashOuter, Vector2D pixelStashMiddle, Vector2D pixelStashInner, Vector2D parkingPosition) {
        this.allianceSide = allianceSide;
        this.backField = backField;
        this.startingRobotFacing = startingRobotFacing;
        this.centerTeamElementRotation = centerTeamElementRotation;
        this.releasePixelRotationToCenterLineAngle = releasePixelRotationToCenterLineAngle;
        this.centerLineYPosition = centerLineYPosition;
        this.scanTeamLeftRightElementPosition = scanTeamLeftRightElementPosition;
        this.scanTeamCenterElementPosition = scanTeamCenterElementPosition;
        this.teamElementLinePositionLeft = teamElementLinePositionLeft;
        this.teamElementLinePositionRight = teamElementLinePositionRight;
        this.teamElementLinePositionCenter = teamElementLinePositionCenter;
        this.lowestHorizontalWalkWayAndOutMostVerticalWalkWayCross = lowestHorizontalWalkWayAndOutMostVerticalWalkWayCross;
        this.lowestHorizontalWalkWayAndInnerVerticalWalkWayCross = lowestHorizontalWalkWayAndInnerVerticalWalkWayCross;
        this.aimWallSweetSpot = aimWallSweetSpot;
        this.pixelStashOuter = pixelStashOuter;
        this.pixelStashMiddle = pixelStashMiddle;
        this.pixelStashInner = pixelStashInner;
        this.parkingPosition = parkingPosition;
    }

    public Vector2D getReleasePixelLinePosition(TeamElementFinder.TeamElementPosition teamElementPosition) {
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

    public double getReleasePixelRotation() {
        return centerTeamElementRotation + releasePixelRotationToCenterLineAngle;
    }

}
