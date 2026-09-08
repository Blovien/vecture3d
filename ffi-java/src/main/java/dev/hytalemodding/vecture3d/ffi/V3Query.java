package dev.hytalemodding.vecture3d.ffi;

import java.util.List;
import java.util.Objects;

/** One closest world query. Geometry points are relative to the origin. */
public record V3Query(
    long queryId,
    double originX,
    double originY,
    double originZ,
    float halfExtentX,
    float halfExtentY,
    float halfExtentZ,
    float translationX,
    float translationY,
    float translationZ,
    Kind kind,
    long categoryBits,
    long maskBits,
    float radius,
    List<V3HullPoint> points
) {
    private static final long ALL_BODY_CATEGORIES = 7L;

    /**
     * Creates a legacy box query against static obstacles.
     */
    public V3Query(
        long queryId,
        double originX,
        double originY,
        double originZ,
        float halfExtentX,
        float halfExtentY,
        float halfExtentZ,
        float translationX,
        float translationY,
        float translationZ
    ) {
        this(
            queryId, originX, originY, originZ, halfExtentX, halfExtentY, halfExtentZ,
            translationX, translationY, translationZ, Kind.LEGACY_BOX, 0L, 0L, 0.0f, List.of()
        );
    }

    public V3Query {
        Objects.requireNonNull(kind, "kind");
        points = List.copyOf(Objects.requireNonNull(points, "points"));
        if (queryId == 0) {
            throw new IllegalArgumentException("queryId cannot be zero");
        }
        if (!Double.isFinite(originX) || !Double.isFinite(originY) || !Double.isFinite(originZ)
            || !V3BoxBodyCommand.allFinite(
                halfExtentX, halfExtentY, halfExtentZ, translationX, translationY, translationZ, radius
            )) {
            throw new IllegalArgumentException("query values must be finite");
        }
        if ((kind == Kind.LEGACY_BOX || kind == Kind.BOX)
            && (halfExtentX <= 0.0f || halfExtentY <= 0.0f || halfExtentZ <= 0.0f)) {
            throw new IllegalArgumentException("box query half extents must be positive");
        }
        if (radius < 0.0f || points.size() > 64) {
            throw new IllegalArgumentException("query radius or point count is invalid");
        }
        int requiredPoints = switch (kind) {
            case LEGACY_BOX, RAY -> 0;
            case SPHERE, BOX -> 1;
            case CAPSULE -> 2;
            case HULL -> -1;
        };
        if ((requiredPoints >= 0 && points.size() != requiredPoints) || (kind == Kind.HULL && points.size() < 4)) {
            throw new IllegalArgumentException("query geometry has the wrong point count");
        }
        if ((kind == Kind.SPHERE || kind == Kind.CAPSULE) && radius <= 0.0f) {
            throw new IllegalArgumentException("sphere and capsule radii must be positive");
        }
        if ((kind == Kind.LEGACY_BOX || kind == Kind.RAY || kind == Kind.BOX) && radius != 0.0f) {
            throw new IllegalArgumentException("this query kind requires zero radius");
        }
    }

    public static V3Query ray(
        long queryId, double originX, double originY, double originZ,
        float translationX, float translationY, float translationZ
    ) {
        return cast(queryId, originX, originY, originZ, translationX, translationY, translationZ, Kind.RAY, 0.0f, List.of());
    }

    public static V3Query sphere(
        long queryId, double originX, double originY, double originZ, V3HullPoint center, float radius,
        float translationX, float translationY, float translationZ
    ) {
        return cast(
            queryId, originX, originY, originZ, translationX, translationY, translationZ,
            Kind.SPHERE, radius, List.of(center)
        );
    }

    public static V3Query capsule(
        long queryId, double originX, double originY, double originZ, V3HullPoint center1, V3HullPoint center2,
        float radius, float translationX, float translationY, float translationZ
    ) {
        return cast(
            queryId, originX, originY, originZ, translationX, translationY, translationZ,
            Kind.CAPSULE, radius, List.of(center1, center2)
        );
    }

    public static V3Query box(
        long queryId, double originX, double originY, double originZ, V3HullPoint center,
        float halfExtentX, float halfExtentY, float halfExtentZ,
        float translationX, float translationY, float translationZ
    ) {
        return new V3Query(
            queryId, originX, originY, originZ, halfExtentX, halfExtentY, halfExtentZ,
            translationX, translationY, translationZ, Kind.BOX,
            ALL_BODY_CATEGORIES, ALL_BODY_CATEGORIES, 0.0f, List.of(center)
        );
    }

    public static V3Query hull(
        long queryId, double originX, double originY, double originZ, List<V3HullPoint> points,
        float translationX, float translationY, float translationZ
    ) {
        return cast(queryId, originX, originY, originZ, translationX, translationY, translationZ, Kind.HULL, 0.0f, points);
    }

    public V3Query withFilter(long queryCategoryBits, long queryMaskBits) {
        return new V3Query(
            queryId, originX, originY, originZ, halfExtentX, halfExtentY, halfExtentZ,
            translationX, translationY, translationZ, kind,
            queryCategoryBits, queryMaskBits, radius, points
        );
    }

    private static V3Query cast(
        long queryId, double originX, double originY, double originZ,
        float translationX, float translationY, float translationZ, Kind kind,
        float radius, List<V3HullPoint> points
    ) {
        return new V3Query(
            queryId, originX, originY, originZ, 0.0f, 0.0f, 0.0f,
            translationX, translationY, translationZ, kind,
            ALL_BODY_CATEGORIES, ALL_BODY_CATEGORIES, radius, points
        );
    }

    public enum Kind {
        LEGACY_BOX(0), RAY(1), SPHERE(2), CAPSULE(3), BOX(4), HULL(5);

        private final int nativeValue;

        Kind(int nativeValue) {
            this.nativeValue = nativeValue;
        }

        int nativeValue() {
            return nativeValue;
        }
    }
}
