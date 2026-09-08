package dev.hytalemodding.vecture3d.ffi;

import java.io.IOException;
import java.lang.foreign.Arena;
import java.lang.foreign.MemorySegment;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;
import java.util.Objects;
import java.util.Properties;

import dev.hytalemodding.vecture3d.ffi.generated.V3Abi;
import dev.hytalemodding.vecture3d.ffi.generated.v3_block_box;
import dev.hytalemodding.vecture3d.ffi.generated.v3_block_cell;
import dev.hytalemodding.vecture3d.ffi.generated.v3_block_material;
import dev.hytalemodding.vecture3d.ffi.generated.v3_world_limits;

import static java.lang.foreign.ValueLayout.ADDRESS;

public final class V3NativeLibrary {
    private static final String PROCESS_PATH_KEY = "dev.hytalemodding.vecture3d.ffi.native.path";
    private static final int MAX_BLOCK_GRID_ITEMS = 65_534;

    private static V3NativeLibrary loadedLibrary;

    private V3NativeLibrary() {
    }

    /**
     * Loads one global Vecture3D library process before generated bindings are initialized
     */
    public static V3NativeLibrary load(Path path) {
        Path realPath = validatePath(path);
        Properties processState = System.getProperties();
        synchronized (processState) {
            String admittedPath = processState.getProperty(PROCESS_PATH_KEY);
            if (admittedPath != null) {
                if (loadedLibrary == null) {
                    throw new IllegalStateException(
                        "Vecture3D must be loaded once from a shared parent classloader"
                    );
                }
                if (!admittedPath.equals(realPath.toString())) {
                    throw new IllegalStateException("Vecture3D is already loaded from " + admittedPath);
                }
                return loadedLibrary;
            }
            if (loadedLibrary != null) {
                throw new IllegalStateException("Vecture3D process admission state is invalid");
            }

            // NOTE: Jextract resolves loader symbols when its generated classes initialize
            System.load(realPath.toString());
            long nativeVersion = Integer.toUnsignedLong(V3Abi.v3_abi_version());
            long nativeHash = V3Abi.v3_abi_schema_hash();
            long expectedVersion = Integer.toUnsignedLong(V3Abi.V3_ABI_VERSION());
            if (nativeVersion != expectedVersion || nativeHash != V3Abi.V3_ABI_SCHEMA_HASH()) {
                throw new V3Exception(V3Exception.Kind.ABI_MISMATCH, "load", nativeVersion, nativeHash);
            }

            loadedLibrary = new V3NativeLibrary();
            processState.setProperty(PROCESS_PATH_KEY, realPath.toString());
            return loadedLibrary;
        }
    }

    public V3World createWorld(double gravityX, double gravityY, double gravityZ) {
        return createWorld(gravityX, gravityY, gravityZ, V3WorldLimits.DEFAULTS);
    }

    public V3World createWorld(double gravityX, double gravityY, double gravityZ, V3WorldLimits limits) {
        requireGravity(gravityX, gravityY, gravityZ);
        Objects.requireNonNull(limits, "limits");
        MemorySegment nativeWorld;
        // The record is read during the call only; the world copies it into its own configuration.
        try (Arena creationArena = Arena.ofConfined()) {
            MemorySegment nativeLimits = v3_world_limits.allocate(creationArena);
            V3World.writeLimits(nativeLimits, limits);
            nativeWorld = V3Abi.v3_world_create(gravityX, gravityY, gravityZ, nativeLimits);
        }
        if (nativeWorld.equals(MemorySegment.NULL)) {
            long status = Integer.toUnsignedLong(V3Abi.V3_NATIVE_FAILURE());
            throw new V3Exception(V3Exception.Kind.NATIVE_STATUS, "createWorld", status, 0);
        }
        try {
            return V3World.create(nativeWorld);
        } catch (RuntimeException | Error failure) {
            V3Abi.v3_world_destroy(nativeWorld);
            throw failure;
        }
    }

    /**
     * Cooks immutable BlockGrid geometry without touching a world.
     *
     * <p>Every box names the cell that owns it and the boxes may be listed in any order. The
     * returned handle owns one native reference; close it once, after the last attachment that
     * needs it, because each attachment takes a reference of its own.
     */
    public V3CookedGrid cookBlockGrid(
        List<V3BlockMaterial> materials,
        List<V3BlockCell> cells,
        List<V3BlockBox> boxes
    ) {
        List<V3BlockMaterial> materialValues = List.copyOf(Objects.requireNonNull(materials, "materials"));
        List<V3BlockCell> cellValues = List.copyOf(Objects.requireNonNull(cells, "cells"));
        List<V3BlockBox> boxValues = List.copyOf(Objects.requireNonNull(boxes, "boxes"));
        requireCookSize(materialValues.size(), "materials");
        requireCookSize(cellValues.size(), "cells");
        requireCookSize(boxValues.size(), "boxes");
        for (V3BlockCell cell : cellValues) {
            if (cell.materialIndex() >= materialValues.size()) {
                throw new IllegalArgumentException("cell material index is out of range");
            }
        }
        for (V3BlockBox box : boxValues) {
            if (box.ownerCellIndex() >= cellValues.size()) {
                throw new IllegalArgumentException("box owner cell index is out of range");
            }
            if (box.materialIndex() >= materialValues.size()) {
                throw new IllegalArgumentException("box material index is out of range");
            }
        }

        MemorySegment nativeHandle;
        // The cooker copies every array before returning, so the input lives for the call only.
        try (Arena cookArena = Arena.ofConfined()) {
            MemorySegment nativeMaterials = v3_block_material.allocateArray(materialValues.size(), cookArena);
            MemorySegment nativeCells = v3_block_cell.allocateArray(cellValues.size(), cookArena);
            MemorySegment nativeBoxes = v3_block_box.allocateArray(boxValues.size(), cookArena);
            MemorySegment out = cookArena.allocate(ADDRESS);
            for (int index = 0; index < materialValues.size(); index++) {
                writeBlockMaterial(v3_block_material.asSlice(nativeMaterials, index), materialValues.get(index));
            }
            for (int index = 0; index < cellValues.size(); index++) {
                writeBlockCell(v3_block_cell.asSlice(nativeCells, index), cellValues.get(index));
            }
            for (int index = 0; index < boxValues.size(); index++) {
                writeBlockBox(v3_block_box.asSlice(nativeBoxes, index), boxValues.get(index));
            }

            int status = V3Abi.v3_cook_block_grid(
                nativeMaterials,
                materialValues.size(),
                nativeCells,
                cellValues.size(),
                nativeBoxes,
                boxValues.size(),
                out
            );
            requireSuccess("cookBlockGrid", status, cellValues.size());
            nativeHandle = out.get(ADDRESS, 0);
        }
        if (nativeHandle.equals(MemorySegment.NULL)) {
            throw new IllegalStateException("cookBlockGrid succeeded without publishing a handle");
        }
        return new V3CookedGrid(nativeHandle);
    }

    public int activeWorldCount() {
        return V3Abi.v3_active_world_count();
    }

    static void requireSuccess(String operation, int status, long detail) {
        if (status != V3Abi.V3_OK()) {
            throw new V3Exception(V3Exception.Kind.NATIVE_STATUS, operation, Integer.toUnsignedLong(status), detail);
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

    private static void requireCookSize(int size, String name) {
        if (size == 0) {
            throw new IllegalArgumentException(name + " cannot be empty");
        }
        if (size > MAX_BLOCK_GRID_ITEMS) {
            throw new IllegalArgumentException(name + " exceeds the native limit");
        }
    }

    private static void writeBlockMaterial(MemorySegment target, V3BlockMaterial value) {
        v3_block_material.material_id(target, value.materialId());
        v3_block_material.density(target, value.density());
        v3_block_material.friction(target, value.friction());
        v3_block_material.restitution(target, value.restitution());
        v3_block_material.bond_strength(target, value.bondStrength());
        v3_block_material.compressive_factor(target, value.compressiveFactor());
        v3_block_material.flags(target, value.flags());
    }

    private static void writeBlockCell(MemorySegment target, V3BlockCell value) {
        v3_block_cell.x(target, value.x());
        v3_block_cell.y(target, value.y());
        v3_block_cell.z(target, value.z());
        v3_block_cell.material_index(target, value.materialIndex());
        v3_block_cell.feature_id(target, value.featureId());
        v3_block_cell.flags(target, 0);
        v3_block_cell.reserved(target, 0);
    }

    private static void writeBlockBox(MemorySegment target, V3BlockBox value) {
        v3_block_box.center_x(target, value.centerX());
        v3_block_box.center_y(target, value.centerY());
        v3_block_box.center_z(target, value.centerZ());
        v3_block_box.half_extent_x(target, value.halfExtentX());
        v3_block_box.half_extent_y(target, value.halfExtentY());
        v3_block_box.half_extent_z(target, value.halfExtentZ());
        v3_block_box.owner_cell_index(target, value.ownerCellIndex());
        v3_block_box.material_index(target, value.materialIndex());
        v3_block_box.feature_id(target, value.featureId());
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
