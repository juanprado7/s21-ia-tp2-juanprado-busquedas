import java.util.*;
import java.util.stream.Collectors;

/** TP2 IA — Prototipo 1D: BFS (exhaustiva) y A* (heurística)
 *  Estados = posiciones enteras sobre el eje horizontal H
 *  Operadores = mover ±ΔH
 *  Meta = alcanzar posición goal
 */
public class Main {

    // Nodo del grafo de búsqueda
    static class Node {
        final int x;             // posición en H
        final Node parent;       // para reconstruir camino
        final int g;             // costo acumulado (pasos)
        final int h;             // heurística (solo A*)
        Node(int x, Node parent, int g, int h) {
            this.x = x;
            this.parent = parent;
            this.g = g;
            this.h = h;
        }
        int f() { return g + h; }
    }

    // Heurística 1D: distancia lineal restante
    static int h(int x, int goal) { return Math.abs(x - goal); }

    // Reconstruye camino desde el nodo solución hasta el origen
    static List<Integer> reconstruct(Node goalNode) {
        LinkedList<Integer> path = new LinkedList<>();
        for (Node n = goalNode; n != null; n = n.parent) path.addFirst(n.x);
        return path;
    }

    // Imprime Abierta y Cerrada en un formato compacto
    static void printOpenClosed(int step, Collection<Node> open, Set<Integer> closed, boolean astar) {
        String openStr = open.stream()
                .map(n -> astar ? String.format("%+d[g=%d,h=%d,f=%d]", n.x, n.g, n.h, n.f())
                                : String.format("%+d", n.x))
                .collect(Collectors.joining(", "));
        String closedStr = closed.stream()
                .sorted(Comparator.comparingInt(i -> i))
                .map(i -> String.format("%+d", i))
                .collect(Collectors.joining(", "));
        System.out.printf("Paso %d%n", step);
        System.out.printf("  Abierta: [%s]%n", openStr);
        System.out.printf("  Cerrada: [%s]%n%n", closedStr);
    }

    /** BFS en 1D con corte al GENERAR la meta */
    static List<Integer> bfs1D(int start, int goal, int delta, int maxAbs) {
        Queue<Node> open = new ArrayDeque<>();
        Set<Integer> closed = new HashSet<>();
        open.add(new Node(start, null, 0, 0));

        int step = 0;
        printOpenClosed(++step, open, closed, false);

        while (!open.isEmpty()) {
            Node cur = open.remove();
            closed.add(cur.x);

            // Generación de sucesores: ±ΔH
            for (int dir : new int[]{+1, -1}) {
                int nx = cur.x + dir * delta;

                // Cotas de seguridad para evitar espacios infinitos
                if (Math.abs(nx) > maxAbs) continue;
                if (closed.contains(nx)) continue;

                Node succ = new Node(nx, cur, cur.g + 1, 0);

                // CORTE al generar la meta
                if (nx == goal) {
                    // Para la última impresión mostramos sólo la meta en Abierta
                    Queue<Node> snapshot = new ArrayDeque<>();
                    snapshot.add(succ);
                    printOpenClosed(++step, snapshot, closed, false);
                    return reconstruct(succ);
                }

                // Evitar duplicados en Abierta
                boolean alreadyInOpen = open.stream().anyMatch(n -> n.x == nx);
                if (!alreadyInOpen) open.add(succ);
            }

            printOpenClosed(++step, open, closed, false);
        }
        return Collections.emptyList(); // no encontrada
    }

    /** A* en 1D con h admisible y consistente: |x - goal| */
    static List<Integer> aStar1D(int start, int goal, int delta, int maxAbs) {
        Comparator<Node> byFThenH = Comparator.<Node>comparingInt(Node::f)
                                               .thenComparingInt(n -> n.h);
        PriorityQueue<Node> open = new PriorityQueue<>(byFThenH);
        Map<Integer, Integer> bestG = new HashMap<>(); // g óptimo conocido por posición
        Set<Integer> closed = new HashSet<>();

        Node startNode = new Node(start, null, 0, h(start, goal));
        open.add(startNode);
        bestG.put(start, 0);

        int step = 0;
        printOpenClosed(++step, open, closed, true);

        while (!open.isEmpty()) {
            Node cur = open.poll();
            if (cur.x == goal) {
                // Ya es la meta: mostramos estado final
                PriorityQueue<Node> snapshot = new PriorityQueue<>(byFThenH);
                snapshot.add(cur);
                printOpenClosed(++step, snapshot, closed, true);
                return reconstruct(cur);
            }
            closed.add(cur.x);

            // Generación de sucesores ±ΔH
            for (int dir : new int[]{+1, -1}) {
                int nx = cur.x + dir * delta;
                if (Math.abs(nx) > maxAbs) continue;
                int ng = cur.g + 1;
                int nh = h(nx, goal);
                Node succ = new Node(nx, cur, ng, nh);

                if (closed.contains(nx)) continue;

                // Relajación tipo A*: sólo encolar si mejora g conocido
                Integer knownG = bestG.get(nx);
                if (knownG == null || ng < knownG) {
                    bestG.put(nx, ng);
                    open.add(succ);
                }
            }

            printOpenClosed(++step, open, closed, true);
        }
        return Collections.emptyList();
    }

    // Utilidad para imprimir el camino solución
    static void printPath(String tag, List<Integer> path) {
        if (path.isEmpty()) {
            System.out.println(tag + " — Sin solución");
        } else {
            String s = path.stream().map(i -> String.format("%+d", i))
                           .collect(Collectors.joining(" -> "));
            System.out.println(tag + " — Camino: " + s);
            System.out.println(tag + " — Movimientos: " + (path.size() - 1));
        }
        System.out.println();
    }

    public static void main(String[] args) {
        // Parámetros desde CLI o valores por defecto
        int start = args.length > 0 ? Integer.parseInt(args[0]) : 0;   // B(0)
        int goal  = args.length > 1 ? Integer.parseInt(args[1]) : 3;   // A(+3)
        int dH    = args.length > 2 ? Integer.parseInt(args[2]) : 1;   // ΔH
        int maxAbs= args.length > 3 ? Integer.parseInt(args[3]) : 20;  // cota de seguridad

        System.out.printf("Caso 1D — start=%+d, goal=%+d, ΔH=%d%n%n", start, goal, dH);

        System.out.println("=== BFS (exhaustiva) — corte al GENERAR la meta ===");
        List<Integer> pathBfs = bfs1D(start, goal, dH, maxAbs);
        printPath("BFS", pathBfs);

        System.out.println("=== A* (heurística) — f = g + h, h = |x - goal| ===");
        List<Integer> pathA = aStar1D(start, goal, dH, maxAbs);
        printPath("A*", pathA);
    }
}
