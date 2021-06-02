package ru.vsu.cs.course1.graph;

import java.util.*;
import java.util.function.Consumer;

public class GraphAlgorithms {
    /**
     * Лемма:
     * Наличие двух различных рёберно простых путей между какими-либо двумя вершинами неориентированного графа
     * равносильно наличию цикла в этом графе.
     */
    public static List<Integer> neededVertices2(Graph graph, int from, int to) {
        if (from < 0 || to < 0 || from >= graph.vertexCount() || to >= graph.vertexCount() || to == from)
            return null;
        List<Integer> shortestPath = shortestPath(graph, from, to);
        HashSet<Integer> set = allVertInCycles(graph, from);
        List<Integer> list = new ArrayList<>();
        for (int i = 1; i < shortestPath.size() - 1; i++) {
            if (!set.contains(shortestPath.get(i - 1)) || !set.contains(shortestPath.get(i + 1)))
                list.add(shortestPath.get(i));
        }
        return list;
    }

    private enum State {
        NOT_VISITED, IN, OUT
    }

    private static HashSet<Integer> allVertInCycles(Graph graph, int from) {
        int[] fromVert = new int[graph.vertexCount()];
        Arrays.fill(fromVert, -1);
        State[] states = new State[graph.vertexCount()];
        Arrays.fill(states, State.NOT_VISITED);
        HashSet<Integer> set = new HashSet<>();
        class Inner {
            void visit(int curr, int prev) {
                if (states[curr] == State.IN) {
                    int now = prev;
                    do {
                        set.add(now);
                        states[now] = State.OUT;
                        now = fromVert[now];
                    } while (now != curr);
                    set.add(now);
                    states[now] = State.OUT;
                } else {
                    fromVert[curr] = prev;
                    states[curr] = State.IN;
                    for (Integer v : graph.adjacencies(curr)) {
                        if (v != prev && states[v] != State.OUT) {
                            visit(v, curr);
                        }
                    }
                    states[curr] = State.OUT;
                }
            }
        }
        new Inner().visit(from, -1);
        return set;
    }

    private static List<Integer> shortestPath(Graph graph, int from, int to) {
        boolean[] visited = new boolean[graph.vertexCount()];
        int[] fromVert = new int[graph.vertexCount()];
        Arrays.fill(fromVert, -1);
        Queue<Integer> queue = new LinkedList<>();
        queue.add(from);
        visited[from] = true;
        while (queue.size() > 0) {
            Integer curr = queue.remove();
            if (curr == to)
                break;
            for (Integer v : graph.adjacencies(curr)) {
                if (!visited[v]) {
                    queue.add(v);
                    fromVert[v] = curr;
                    visited[v] = true;
                }
            }
        }
        List<Integer> list = new ArrayList<>();
        while (to != -1) {
            list.add(to);
            to = fromVert[to];
        }
        return list;
    }

    /**
     * По времени (список смежности):
     * O( |V| * ( |V| + |E| ) )
     * Насыщенный граф ( |E|=O(|V|^2) ): O( |V|^3 )
     * Разреженный граф ( |E|=O(|V|) ): O( |V|^2 )
     * По времени (матрица смежности):
     * O( |V|^3 )
     * По памяти:
     * O( |V| )
     */
    public static List<Integer> neededVertices(Graph graph, int from, int to) {
        if (from < 0 || to < 0 || from >= graph.vertexCount() || to >= graph.vertexCount() || to == from
                || !myDfsRecursion(graph, from, from, to))
            return null;
        List<Integer> list = new ArrayList<>();
        for (int i = 0; i < graph.vertexCount(); i++) {
            if (i != from && i != to && !myDfsRecursion(graph, from, i, to))
                list.add(i);
        }
        return list;
    }

    private static boolean myDfsRecursion(Graph graph, int from, int visitedVertex, int to) {
        boolean[] visited = new boolean[graph.vertexCount()];
        visited[visitedVertex] = true;
        class Inner {
            void visit(Integer curr) {
                visited[curr] = true;
                if (curr == to)
                    return;
                for (Integer v : graph.adjacencies(curr)) {
                    if (!visited[v]) {
                        visit(v);
                    }
                }
            }
        }
        new Inner().visit(from);
        return visited[to];
    }

    /**
     * Поиск в глубину, реализованный рекурсивно
     * (начальная вершина также включена)
     *
     * @param graph   граф
     * @param from    Вершина, с которой начинается поиск
     * @param visitor Посетитель
     */
    public static void dfsRecursion(Graph graph, int from, Consumer<Integer> visitor) {
        boolean[] visited = new boolean[graph.vertexCount()];

        class Inner {
            void visit(Integer curr) {
                visitor.accept(curr);
                visited[curr] = true;
                for (Integer v : graph.adjacencies(curr)) {
                    if (!visited[v]) {
                        visit(v);
                    }
                }
            }
        }
        new Inner().visit(from);
    }

    /**
     * Поиск в глубину, реализованный с помощью стека
     * (не совсем "правильный"/классический, т.к. "в глубину" реализуется только "план" обхода, а не сам обход)
     *
     * @param graph   граф
     * @param from    Вершина, с которой начинается поиск
     * @param visitor Посетитель
     */
    public static void dfs(Graph graph, int from, Consumer<Integer> visitor) {
        boolean[] visited = new boolean[graph.vertexCount()];
        Stack<Integer> stack = new Stack<Integer>();
        stack.push(from);
        visited[from] = true;
        while (!stack.empty()) {
            Integer curr = stack.pop();
            visitor.accept(curr);
            for (Integer v : graph.adjacencies(curr)) {
                if (!visited[v]) {
                    stack.push(v);
                    visited[v] = true;
                }
            }
        }
    }

    /**
     * Поиск в ширину, реализованный с помощью очереди
     * (начальная вершина также включена)
     *
     * @param graph   граф
     * @param from    Вершина, с которой начинается поиск
     * @param visitor Посетитель
     */
    public static void bfs(Graph graph, int from, Consumer<Integer> visitor) {
        boolean[] visited = new boolean[graph.vertexCount()];
        Queue<Integer> queue = new LinkedList<Integer>();
        queue.add(from);
        visited[from] = true;
        while (queue.size() > 0) {
            Integer curr = queue.remove();
            visitor.accept(curr);
            for (Integer v : graph.adjacencies(curr)) {
                if (!visited[v]) {
                    queue.add(v);
                    visited[v] = true;
                }
            }
        }
    }

    /**
     * Поиск в глубину в виде итератора
     * (начальная вершина также включена)
     *
     * @param graph граф
     * @param from  Вершина, с которой начинается поиск
     * @return Итератор
     */
    public static Iterable<Integer> dfs(Graph graph, int from) {
        return new Iterable<Integer>() {
            private Stack<Integer> stack = null;
            private boolean[] visited = null;

            @Override
            public Iterator<Integer> iterator() {
                stack = new Stack<>();
                stack.push(from);
                visited = new boolean[graph.vertexCount()];
                visited[from] = true;

                return new Iterator<Integer>() {
                    @Override
                    public boolean hasNext() {
                        return !stack.isEmpty();
                    }

                    @Override
                    public Integer next() {
                        Integer result = stack.pop();
                        for (Integer adj : graph.adjacencies(result)) {
                            if (!visited[adj]) {
                                visited[adj] = true;
                                stack.add(adj);
                            }
                        }
                        return result;
                    }
                };
            }
        };
    }

    /**
     * Поиск в ширину в виде итератора
     * (начальная вершина также включена)
     *
     * @param from Вершина, с которой начинается поиск
     * @return Итератор
     */
    public static Iterable<Integer> bfs(Graph graph, int from) {
        return new Iterable<Integer>() {
            private Queue<Integer> queue = null;
            private boolean[] visited = null;

            @Override
            public Iterator<Integer> iterator() {
                queue = new LinkedList<>();
                queue.add(from);
                visited = new boolean[graph.vertexCount()];
                visited[from] = true;

                return new Iterator<Integer>() {
                    @Override
                    public boolean hasNext() {
                        return !queue.isEmpty();
                    }

                    @Override
                    public Integer next() {
                        Integer result = queue.remove();
                        for (Integer adj : graph.adjacencies(result)) {
                            if (!visited[adj]) {
                                visited[adj] = true;
                                queue.add(adj);
                            }
                        }
                        return result;
                    }
                };
            }
        };
    }
}
