package dev.hytalemodding.vecture3d.ffi;

import java.io.IOException;
import java.lang.foreign.MemorySegment;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Objects;

import dev.hytalemodding.vecture3d.ffi.generated.V3Abi;

public final class V3NativeLibrary {
    private static final Object LOAD_LOCK = new Object();

    private static Path loadedPath;
    private static V3NativeLibrary loadedLibrary;

    private V3NativeLibrary() {
    }

    /**
     * Loads one global Vecture3D library process before generated bindings are initialized
     */
    public static V3NativeLibrary load(Path path) {
        Path realPath = validatePath(path);
        synchronized (LOAD_LOCK) {
            if (loadedLibrary != null) {
                if (!loadedPath.equals(realPath)) {
                    throw new IllegalStateException("Vecture3D is already loaded from " + loadedPath);
                }
                return loadedLibrary;
            }

            // NOTE: Jextract resolves loader symbols when its generated classes initialize
            System.load(realPath.toString());
            int nativeVersion = V3Abi.v3_abi_version();
            long nativeHash = V3Abi.v3_abi_schema_hash();
            if (nativeVersion != V3Abi.V3_ABI_VERSION() || nativeHash != V3Abi.V3_ABI_SCHEMA_HASH()) {
                throw new V3Exception(V3Exception.Kind.ABI_MISMATCH, "load", nativeVersion, nativeHash);
            }

            loadedPath = realPath;
            loadedLibrary = new V3NativeLibrary();
            return loadedLibrary;
        }
    }

    public V3World createWorld(double gravityX, double gravityY, double gravityZ) {
        requireGravity(gravityX, gravityY, gravityZ);
        MemorySegment nativeWorld = V3Abi.v3_world_create(gravityX, gravityY, gravityZ);
        if (nativeWorld.equals(MemorySegment.NULL)) {
            throw new V3Exception(V3Exception.Kind.NATIVE_STATUS, "createWorld", V3Abi.V3_NATIVE_FAILURE(), 0);
        }
        try {
            return V3World.create(nativeWorld);
        } catch (RuntimeException | Error failure) {
            V3Abi.v3_world_destroy(nativeWorld);
            throw failure;
        }
    }

    int activeWorldCount() {
        return V3Abi.v3_active_world_count();
    }

    static void requireSuccess(String operation, int status, long detail) {
        if (status != V3Abi.V3_OK()) {
            throw new V3Exception(V3Exception.Kind.NATIVE_STATUS, operation, status, detail);
        }
    }

    private static Path validatePath(Path path) {
        Objects.requireNonNull(path, "path");
        if (!path.isAbsolute()) {
            throw new IllegalArgumentException("path must be absolute");
        }

        final Path realPath;
        try {
            realPath = path.toRealPath();
        } catch (IOException failure) {
            throw new IllegalArgumentException("path must resolve to a real native library", failure);
        }
        if (!Files.isRegularFile(realPath)) {
            throw new IllegalArgumentException("path must resolve to a regular file");
        }
        if (!Files.isReadable(realPath)) {
            throw new IllegalArgumentException("native library must be readable");
        }
        return realPath;
    }

    private static void requireGravity(double gravityX, double gravityY, double gravityZ) {
        if (!Double.isFinite(gravityX) || !Double.isFinite(gravityY) || !Double.isFinite(gravityZ)
            || Math.abs(gravityX) > Float.MAX_VALUE
            || Math.abs(gravityY) > Float.MAX_VALUE
            || Math.abs(gravityZ) > Float.MAX_VALUE) {
            throw new IllegalArgumentException("gravity must be finite and representable as native floats");
        }
    }
}
