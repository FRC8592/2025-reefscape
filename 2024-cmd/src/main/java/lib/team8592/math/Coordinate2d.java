package lib.team8592.math;

import edu.wpi.first.math.geometry.Translation2d;

public class Coordinate2d {
    private Translation2d translationMeters;

    public Coordinate2d(Distance2d x, Distance2d y) {
        this.translationMeters = new Translation2d(x.toMeters(), y.toMeters());
    }

    public Coordinate2d(Translation2d translation) {
        this.translationMeters = translation;
    }

    public Coordinate2d mirrorFromX(double x) {
        double newX = 2*x - translationMeters.getX();
        translationMeters = new Translation2d(newX, translationMeters.getY());
        return this;
    }

    public Coordinate2d mirrorFromY(double y) {
        double newY = 2*y - translationMeters.getY();
        translationMeters = new Translation2d(translationMeters.getX(), newY);
        return this;
    }

    public Coordinate2d mirrorFromCoordinate(Coordinate2d coordinate) {
        double newX = 2*coordinate.getX().toMeters() - translationMeters.getX();
        double newY = 2*coordinate.getY().toMeters() - translationMeters.getY();
        translationMeters = new Translation2d(newX, newY);
        return this;
    }

    public Distance2d getX() {
        return Distance2d.fromMeters(translationMeters.getX());
    }

    public Distance2d getY() {
        return Distance2d.fromMeters(translationMeters.getX());
    }

    public Translation2d getTranslation() {
        return translationMeters;
    }

    public Distance2d getDistanceFromCoordinate(Coordinate2d other) {
        return Distance2d.fromMeters(translationMeters.getDistance(other.getTranslation()));
    }

    public Vector2d vectorBetweenCoordinates(Coordinate2d other) {
        return new Vector2d(
            other.getX().minus(this.getX()),
            other.getY().minus(this.getY())
        );
    }

    public String toString() {
        return "(" + translationMeters.getX() + ", " + translationMeters.getY() + ")";
    }
}