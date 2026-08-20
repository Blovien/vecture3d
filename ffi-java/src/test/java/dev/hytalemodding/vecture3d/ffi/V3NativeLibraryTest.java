package dev.hytalemodding.vecture3d.ffi;

import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.attribute.PosixFileAttributeView;
import java.nio.file.attribute.PosixFilePermissions;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

class V3NativeLibraryTest {
    @AfterEach
    void noNativeWorldLeaksAcrossTests() {
        assertEquals(0, loadRealLibrary().activeWorldCount());
    }

    @Test
    void loadRejectsInvalidPathsBeforeNativeSideEffects(@TempDir Path tempDirectory) throws Exception {
        assertThrows(NullPointerException.class, () -> V3NativeLibrary.load(null));
        assertThrows(IllegalArgumentException.class, () -> V3NativeLibrary.load(Path.of("relative-library.so")));
        assertThrows(IllegalArgumentException.class, () -> V3NativeLibrary.load(tempDirectory.resolve("missing.so")));
        assertThrows(IllegalArgumentException.class, () -> V3NativeLibrary.load(tempDirectory));

        Path unreadable = Files.writeString(tempDirectory.resolve("unreadable.so"), "not a library");
        if (Files.getFileStore(unreadable).supportsFileAttributeView(PosixFileAttributeView.class)) {
            Files.setPosixFilePermissions(unreadable, PosixFilePermissions.fromString("---------"));
        } else {
            assertTrue(unreadable.toFile().setReadable(false, false));
        }
        assertFalse(Files.isReadable(unreadable));
        assertThrows(IllegalArgumentException.class, () -> V3NativeLibrary.load(unreadable));
        assertTrue(unreadable.toFile().setReadable(true, false));
    }

    @Test
    void sameCanonicalPathIsIdempotentAndLoadCreatesNoWorld() {
        V3NativeLibrary first = loadRealLibrary();
        Path realPath = realLibraryPath();
        Path equivalentPath = realPath.getParent().resolve(".").resolve(realPath.getFileName());
        V3NativeLibrary second = V3NativeLibrary.load(equivalentPath);

        assertSame(first, second);
        assertEquals(0, first.activeWorldCount());
    }

    @Test
    void differentPathIsRejectedAfterSuccessfulLoad(@TempDir Path tempDirectory) throws Exception {
        V3NativeLibrary library = loadRealLibrary();
        Path copy = Files.copy(realLibraryPath(), tempDirectory.resolve(realLibraryPath().getFileName()));

        assertThrows(IllegalStateException.class, () -> V3NativeLibrary.load(copy));
        assertEquals(0, library.activeWorldCount());
    }

    @Test
    void schemaMismatchRunsInForkWithoutPoisoningMainProcess() throws Exception {
        String javaExecutable = Path.of(System.getProperty("java.home"), "bin", "java").toString();
        Process process = new ProcessBuilder(
            javaExecutable,
            "--enable-native-access=ALL-UNNAMED",
            "-cp",
            System.getProperty("v3.test.classpath"),
            MismatchProbe.class.getName(),
            Path.of(System.getProperty("v3.mismatch.library")).toAbsolutePath().toString()
        ).redirectErrorStream(true).start();
        String output = new String(process.getInputStream().readAllBytes());

        assertEquals(0, process.waitFor(), output);
        assertTrue(output.contains("ABI_MISMATCH"), output);
        assertEquals(0, loadRealLibrary().activeWorldCount());
    }

    @Test
    void failedWorldCreationDoesNotLeakAnActiveWorld() {
        V3NativeLibrary library = loadRealLibrary();
        int baseline = library.activeWorldCount();

        assertThrows(IllegalArgumentException.class, () -> library.createWorld(Double.NaN, 0.0, 0.0));
        assertThrows(IllegalArgumentException.class, () -> library.createWorld(Double.MAX_VALUE, 0.0, 0.0));
        assertEquals(baseline, library.activeWorldCount());
    }

    @Test
    void statusTranslationRetainsOperationStatusAndBoundedDetail() {
        loadRealLibrary();
        V3Exception failure = assertThrows(
            V3Exception.class,
            () -> V3NativeLibrary.requireSuccess("replaceBoxBodies", 9, 17)
        );

        assertEquals(V3Exception.Kind.NATIVE_STATUS, failure.kind());
        assertEquals("replaceBoxBodies", failure.operation());
        assertEquals(9, failure.status());
        assertEquals(17, failure.detail());
    }

    private static V3NativeLibrary loadRealLibrary() {
        return V3NativeLibrary.load(realLibraryPath());
    }

    private static Path realLibraryPath() {
        return Path.of(System.getProperty("v3.native.library"));
    }

    public static final class MismatchProbe {
        private MismatchProbe() {
        }

        public static void main(String[] arguments) {
            try {
                V3NativeLibrary.load(Path.of(arguments[0]));
                throw new AssertionError("mismatch library was accepted");
            } catch (V3Exception failure) {
                if (failure.kind() != V3Exception.Kind.ABI_MISMATCH) {
                    throw failure;
                }
                System.out.println(failure.kind());
            }
        }
    }
}
