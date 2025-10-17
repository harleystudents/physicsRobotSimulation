package utils.debug;

import java.util.Map;

public class DebugUtils {
    public static void dumpVars(Map<String, Object> vars) {
        System.out.println("====== Debug Dump ======");
        for (var entry : vars.entrySet()) {
            System.out.printf("%s = %s%n", entry.getKey(), entry.getValue());
        }
        System.out.println("========================");
    }
}
