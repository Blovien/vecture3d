package dev.hytalemodding.vecture3d.ffi;

import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertThrows;

class V3NativeLibraryTest {
    @AfterEach
    void noNativeWorldLeaksAcrossTests() {
        assertEquals(0, loadRealLibrary().activeWorldCount());
    }

    @Test
    void loadRejectsInvalidPathsBeforeNativeSideEffects(@TempDir Path tempDirectory) {
        assertThrows(NullPointerException.class, () -> V3NativeLibrary.load(null));
        assertThrows(IllegalArgumentException.class, () -> V3NativeLibrary.load(Path.of("relative-library.so")));
        assertThrows(IllegalArgumentException.class, () -> V3NativeLibrary.load(tempDirectory.resolve("missing.so")));
        assertThrows(IllegalArgumentException.class, () -> V3NativeLibrary.load(tempDirectory));
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
        V3Exception highBitFailure = assertThrows(
            V3Exception.class,
            () -> V3NativeLibrary.requireSuccess("step", 0x80000001, 0)
        );

        assertEquals(V3Exception.Kind.NATIVE_STATUS, failure.kind());
        assertEquals("replaceBoxBodies", failure.operation());
        assertEquals(9L, failure.status());
        assertEquals(17, failure.detail());
        assertEquals(2_147_483_649L, highBitFailure.status());
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
                System.out.println(failure.kind() + ":" + failure.status());
            }
        }
    }

    public static final class NullWorldProbe {
        private NullWorldProbe() {
        }

        public static void main(String[] arguments) {
            V3NativeLibrary library = V3NativeLibrary.load(Path.of(arguments[0]));
            try {
                library.createWorld(0.0, 0.0, 0.0);
                throw new AssertionError("null world fixture created a world");
            } catch (V3Exception failure) {
                if (failure.kind() != V3Exception.Kind.NATIVE_STATUS || failure.status() != 5L) {
                    throw failure;
                }
                System.out.println(failure.kind() + ":" + failure.status() + ":" + library.activeWorldCount());
            }
        }
    }

    public static final class ConcurrentLoadProbe {
        private ConcurrentLoadProbe() {
        }

        public static void main(String[] arguments) throws Exception {
            int workerCount = 16;
            CountDownLatch ready = new CountDownLatch(workerCount);
            CountDownLatch start = new CountDownLatch(1);
            try (var executor = Executors.newFixedThreadPool(workerCount)) {
                List<Future<V3NativeLibrary>> loads = new java.util.ArrayList<>(workerCount);
                for (int index = 0; index < workerCount; index++) {
                    loads.add(executor.submit(() -> {
                        ready.countDown();
                        start.await();
                        return V3NativeLibrary.load(Path.of(arguments[0]));
                    }));
                }
                ready.await();
                start.countDown();
                V3NativeLibrary expected = loads.getFirst().get();
                for (Future<V3NativeLibrary> load : loads) {
                    if (load.get() != expected) {
                        throw new AssertionError("concurrent load returned multiple instances");
                    }
                }
                if (expected.activeWorldCount() != 0) {
                    throw new AssertionError("concurrent load created a world");
                }
            }
            System.out.println("CONCURRENT_LOAD_OK");
        }
    }

    private record ForkResult(int exitCode, String output) {
    }
}
