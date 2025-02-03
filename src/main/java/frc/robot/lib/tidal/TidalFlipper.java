package frc.robot.lib.tidal;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class TidalFlipper {
    /**
     * Flips a coordinate to a RED Alliance
     * @param coordinate the coordinate to flip
     * @return the coordinate flipped
     */
    public static Coordinate coordinate(Coordinate coordinate){

        Translation2d fixedXY =
         new Translation2d(TidalUtil.getField2025().getLenght() - coordinate.inX(), TidalUtil.getField2025().getWidth() - coordinate.inY());
        Rotation2d fixedAngle = coordinate.toPose2d().getRotation().rotateBy(Rotation2d.kPi);

        Coordinate fixedCoordinate = new Coordinate(fixedXY.getX(), fixedXY.getY(), fixedAngle.getDegrees());

        return fixedCoordinate;
    }

    /**
     * Flips the coordinate inside the grid
     * @param grid the grid to flip
     * @return the grid flipped 
     */
    public static Grid grid(Grid grid){
        return new Grid(coordinate(grid.asCoordinate()), grid.getTolerance());
    }

}
