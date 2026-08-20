package dev.hytalemodding.vecture3d.ffi;

import java.nio.file.Path;

final class V3TestSupport {
    private V3TestSupport() {
    }

    static V3NativeLibrary library() {
        return V3NativeLibrary.load(Path.of(System.getProperty("v3.native.library")));
    }

    static V3BoxBodyCommand box(
        long logicalId,
        int generation,
        V3BoxBodyCommand.Kind kind,
        double x,
        double y,
        double z
    ) {
        return box(logicalId, generation, kind, x, y, z, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
            V3BoxBodyCommand.INITIAL_AWAKE_FLAG);
    }

    static V3BoxBodyCommand box(
        long logicalId,
        int generation,
        V3BoxBodyCommand.Kind kind,
        double x,
        double y,
        double z,
        float linearVelocityX,
        float linearVelocityY,
        float linearVelocityZ,
        float angularVelocityX,
        float angularVelocityY,
        float angularVelocityZ,
        int flags
    ) {
        return new V3BoxBodyCommand(
            new V3BodyHandle(logicalId, generation),
            kind,
            x,
            y,
            z,
            0.0f,
            0.0f,
            0.0f,
            1.0f,
            linearVelocityX,
            linearVelocityY,
            linearVelocityZ,
            angularVelocityX,
            angularVelocityY,
            angularVelocityZ,
            0.5f,
            0.5f,
            0.5f,
            kind == V3BoxBodyCommand.Kind.DYNAMIC ? 1.0f : 0.0f,
            0.5f,
            0.0f,
            0.0f,
            flags
        );
    }

    static V3TerrainBox terrainBox(long featureId, float x, float y, float z) {
        return new V3TerrainBox(featureId, x, y, z, 0.5f, 0.5f, 0.5f, 0.5f);
    }

    static V3Transform transform(V3StepResult result, long logicalId) {
        return result.transforms().stream()
            .filter(value -> value.handle().logicalId() == logicalId)
            .findFirst()
            .orElseThrow(() -> new AssertionError("missing transform for logical body " + logicalId));
    }

}
