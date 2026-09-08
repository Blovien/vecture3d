package dev.hytalemodding.vecture3d.ffi;

import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;

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

    static V3BodyDefinition bodyDefinition(V3BoxBodyCommand body) {
        return new V3BodyDefinition(body.handle(), body.kind(), body.positionX(), body.positionY(), body.positionZ(),
            body.rotationX(), body.rotationY(), body.rotationZ(), body.rotationW(),
            body.linearVelocityX(), body.linearVelocityY(), body.linearVelocityZ(),
            body.angularVelocityX(), body.angularVelocityY(), body.angularVelocityZ(),
            body.linearDamping(), body.angularDamping(), body.flags());
    }

    /** Same centered unit cubes as box(), including the original body origins and unit dynamic mass. */
    static void attachCenteredUnitGrids(V3World world, List<V3BoxBodyCommand> bodies) {
        List<V3BlockCell> cells = new ArrayList<>();
        List<V3BlockBox> boxes = new ArrayList<>();
        for (int x = -1; x <= 0; x++) {
            for (int y = -1; y <= 0; y++) {
                for (int z = -1; z <= 0; z++) {
                    int index = cells.size();
                    cells.add(new V3BlockCell(x, y, z, 0, index + 1L));
                    boxes.add(new V3BlockBox(index, 0, index + 1L,
                        x < 0 ? 0.75f : 0.25f, y < 0 ? 0.75f : 0.25f, z < 0 ? 0.75f : 0.25f,
                        0.25f, 0.25f, 0.25f));
                }
            }
        }
        try (V3CookedGrid grid = library().cookBlockGrid(List.of(BLOCK_MATERIAL), cells, boxes)) {
            for (V3BoxBodyCommand body : bodies) {
                // Collision uses eight partial cells, but mass follows cells rather than clipped boxes.
                // Preserve the old unit cube: m=1, center=0, Ixx=Iyy=Izz=m*(1+1)/12=1/6.
                V3MassProperties mass = body.kind() == V3BoxBodyCommand.Kind.DYNAMIC
                    ? new V3MassProperties(1, 0, 0, 0, 1.0f / 6, 1.0f / 6, 1.0f / 6, 0, 0, 0)
                    : null;
                world.attachBlockGrid(bodyDefinition(body), grid, mass);
            }
        }
    }

    static final V3BlockMaterial BLOCK_MATERIAL = V3BlockMaterial.of(4_242L, 1.0f, 0.5f, 0.0f);

    /** A flat one-layer Terrain Section of full cubes, sixteen cells on each horizontal axis. */
    static BlockGridInput flatTerrainSection() {
        List<V3BlockCell> cells = new ArrayList<>();
        List<V3BlockBox> boxes = new ArrayList<>();
        for (int x = -8; x < 8; x++) {
            for (int z = -8; z < 8; z++) {
                int index = cells.size();
                cells.add(new V3BlockCell(x, 0, z, 0, index + 1L));
                boxes.add(V3BlockBox.fullCube(index, 0, index + 1L));
            }
        }
        return new BlockGridInput(List.copyOf(cells), List.copyOf(boxes));
    }

    static BlockGridInput fragmentedTerrainSection() {
        List<V3BlockCell> cells = new ArrayList<>();
        List<V3BlockBox> boxes = new ArrayList<>();
        for (int z = 0; z < 10; z++) {
            for (int x = 0; x < 16; x++) {
                int index = cells.size();
                cells.add(new V3BlockCell(x, 0, z, 0, index + 1L));
                boxes.add(new V3BlockBox(index, 0, index + 1L, 0.5f, 0.5f, 0.5f, 0.3f, 0.5f, 0.3f));
            }
        }
        return new BlockGridInput(List.copyOf(cells), List.copyOf(boxes));
    }

    static BlockGridInput fragmentedLandingBody() {
        List<V3BlockCell> cells = new ArrayList<>();
        List<V3BlockBox> boxes = new ArrayList<>();
        for (int z = 0; z < 10; z++) {
            for (int x = 0; x < 16; x++) {
                int index = cells.size();
                cells.add(new V3BlockCell(x, 0, z, 0, 1_000L + index));
                boxes.add(V3BlockBox.fullCube(index, 0, 1_000L + index));
            }
        }
        return new BlockGridInput(List.copyOf(cells), List.copyOf(boxes));
    }

    /**
     * A hollow 6x4x6 hull: every cell on a face is filled and the interior is empty.
     *
     * @param reverseBoxOrder lists the boxes back to front, which the cook must group by owner
     */
    static BlockGridInput hollowHull(boolean reverseBoxOrder) {
        List<V3BlockCell> cells = new ArrayList<>();
        for (int x = 0; x < 6; x++) {
            for (int y = 0; y < 4; y++) {
                for (int z = 0; z < 6; z++) {
                    if (x == 0 || x == 5 || y == 0 || y == 3 || z == 0 || z == 5) {
                        cells.add(new V3BlockCell(x, y, z, 0, 900_000L + cells.size()));
                    }
                }
            }
        }
        List<V3BlockBox> boxes = new ArrayList<>();
        for (int index = 0; index < cells.size(); index++) {
            int owner = reverseBoxOrder ? cells.size() - 1 - index : index;
            boxes.add(V3BlockBox.fullCube(owner, 0, cells.get(owner).featureId()));
        }
        return new BlockGridInput(List.copyOf(cells), List.copyOf(boxes));
    }

    record BlockGridInput(List<V3BlockCell> cells, List<V3BlockBox> boxes) {
        V3CookedGrid cook(V3NativeLibrary library) {
            return library.cookBlockGrid(List.of(BLOCK_MATERIAL), cells, boxes);
        }
    }

    static V3Transform transform(V3StepResult result, long logicalId) {
        return result.transforms().stream()
            .filter(value -> value.handle().logicalId() == logicalId)
            .findFirst()
            .orElseThrow(() -> new AssertionError("missing transform for logical body " + logicalId));
    }

}
