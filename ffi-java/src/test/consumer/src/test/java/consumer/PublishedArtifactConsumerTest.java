package consumer;

import java.nio.file.Path;
import java.util.List;

import dev.hytalemodding.vecture3d.ffi.V3NativeLibrary;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;

class PublishedArtifactConsumerTest {
    @Test
    void binaryArtifactCreatesStepsAndClosesAWorldWithoutJextract() {
        V3NativeLibrary library = V3NativeLibrary.load(Path.of(System.getProperty("v3.native.library")));
        try (var world = library.createWorld(0.0, 0.0, 0.0)) {
            assertEquals(0, world.step(0, List.of(), List.of(), List.of()).stats().bodyCount());
        }
    }
}
