#include <bits/stdc++.h>

using namespace std;
///////////////////////////////////////// 1
/// Representation of the problem state

struct Transition {
    std::pair<int, int> from, to;
};

using matrix = std::array<std::array<int, 3>, 3>;

// Boost::hash_combine
template<typename T>
constexpr void hash_combine(size_t &seed, T const &v) {
    seed ^= std::hash<T>{}(v) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
}

struct State {
    matrix m;
    std::pair<int, int> lastMoved;

    [[nodiscard]] std::size_t hash() const {
        std::size_t result = 0;

        for (const auto &row: m)
            for (const auto &value: row)
                hash_combine(result, value);

        hash_combine(result, lastMoved.first);
        hash_combine(result, lastMoved.second);

        return result;
    }

    constexpr void applyTransition(const Transition &T) {
        std::swap(m[T.from.first][T.from.second],
                  m[T.to.first][T.to.second]);
    }

    bool operator==(const State &rhs) const {
        return m == rhs.m && lastMoved == rhs.lastMoved;
    }

    // Used for std::map
    bool operator<(const State &rhs) const {
        return (rhs.m < m);
    }
};

// Define a custom function specialization in std for std::unordered_set to work
namespace std {
    template<>
    struct hash<State> {
        std::size_t operator()(const State &s) const {
            return s.hash();
        }
    };
}

///////////////////////////////////////// 2
/// Initialization function, pass instance, get state

[[nodiscard]] constexpr State getStateFromProblemInstance(const std::span<const int, 9> instance) {
    State state{.lastMoved{-1, -1}};
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            state.m[i][j] = instance[i * 3 + j];
        }
    }
    return state;
}

[[nodiscard]] constexpr auto getSolutionForEmptyPos(int x, int y) noexcept {
    matrix m{};
    int cnt = 0;
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            if (i != x || j != y)
                m[i][j] = ++cnt;
    return m;
}

[[nodiscard]] constexpr auto getFinalSolutions() noexcept {
    std::array<matrix, 9> solutions{};
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) {
            // iterate the position of the empty space
            solutions[i * 3 + j] = getSolutionForEmptyPos(i, j);
        }
    return solutions;
}

void printSolution(const matrix &m) {
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++)
            std::cout << m[i][j] << ' ';
        std::cout << '\n';
    }
    std::cout << '\n';
}

constexpr auto solutions = getFinalSolutions();

/// Check state is final

[[nodiscard]] constexpr bool isFinalState(const State &sol) noexcept {
    return std::ranges::any_of(solutions, [sol](matrix x) {
        return x == sol.m;
    });
}

///////////////////////////////////////// 3

/// Transition validation

[[nodiscard]] constexpr bool inMatrix(std::pair<int, int> pos) noexcept {
    return pos.first >= 0 && pos.first < 3 && pos.second >= 0 && pos.second < 3;
}

[[nodiscard]] constexpr bool isValidTransition(const State &S, const Transition &T) noexcept {
    return inMatrix(T.from) && inMatrix(T.to)
           && S.lastMoved != T.to && S.lastMoved != T.from
           && (S.m[T.from.first][T.from.second] == 0 || S.m[T.to.first][T.to.second] == 0);
}

/// Function that gets state & transition parameters and returns a new state

[[nodiscard]] constexpr State getNewState(const State &S, const Transition &T) noexcept {
    assert(isValidTransition(S, T));
    State newState = S;
    newState.applyTransition(T);
    return newState;
}

[[nodiscard]] constexpr std::vector<State> getReachableStates(const State &s) noexcept {
    std::vector<State> v;
    std::pair<int, int> zeroPos;
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            if (s.m[i][j] == 0)
                zeroPos = {i, j};
    const std::array dx = {1, -1, 0, 0};
    const std::array dy = {0, 0, 1, -1};
    for (int i = 0; i < 4; i++) {
        int newX = zeroPos.first + dx[i], newY = zeroPos.second + dy[i];
        auto transition = Transition{.from = zeroPos, .to = {newX, newY}};
        if (isValidTransition(s, transition))
            v.push_back(getNewState(s, transition));
    }
    return v;
}

[[nodiscard]] constexpr bool isNoneState(const State &state) noexcept {
    if (state.lastMoved.first == 10)
        return true;
    return false;
}


///////////////////////////////////////// Helps print the solution
void
printFoundSolution(const tuple<State, vector<State>, int> &solutionTuple, const char *name) {
    auto [solution, moveSequence, time] = solutionTuple;
    if (!isNoneState(solution)) {
        cout << "Algorithm name " << name << '\n';
        cout << "Number of moves is " << moveSequence.size() << '\n';
        cout << "Elapsed time is " << time << "ms" << '\n';
        printSolution(solution.m);
        if (moveSequence.size() < 10) {
            for (auto it: moveSequence)
                printSolution(it.m);
        }
    } else std::cout << "Did not find solution\n";
}

///////////////////////////////////////// Helps to reconstruct the path
vector<State>
getMoveSequence(map<State, State> &m, const State &crtState, const State &initState) {
    vector<State> moves;
    State state = crtState;
    while (state != initState) {
        moves.push_back(state);
        state = m[state];
    }
    moves.push_back(initState);
    std::reverse(moves.begin(), moves.end());
    return moves;
}
///////////////////////////////////////// 4

std::map<State, State> visited;
constexpr static auto noneState = State{.lastMoved{10, 10}};

[[nodiscard]] State limitedDepthDFS(const State &state, int depth) noexcept {
    if (isFinalState(state))
        return state;
    if (depth == 0)
        return noneState;
    auto neighbours = getReachableStates(state);
    for (const auto &it: neighbours)
        if (!visited.contains(it)) {
            visited[it] = state;
            auto res = limitedDepthDFS(it, depth - 1);
            if (!isNoneState(res))
                return res;
        }
    return noneState;
}

tuple<State, vector<State>, int>
IDDFS(const State &initState, int maxDepth) {
    auto start = chrono::high_resolution_clock::now();
    for (int depth = 0; depth < maxDepth; depth++) {
        visited.clear();
        visited[initState] = initState;
        auto sol = limitedDepthDFS(initState, depth);

        if (!isNoneState(sol)) {
            auto moveSequence = getMoveSequence(visited, sol, initState);
            auto stop = chrono::high_resolution_clock::now();
            auto duration = chrono::duration_cast<chrono::milliseconds>(stop - start);
            return make_tuple(sol, moveSequence, duration.count());
        }
    }
    throw exception();
}

void printSolutionForInstanceIDDFS(const std::span<const int, 9> instance) {
    const static int MAX_DEPTH = 10'000;
    auto initState =
            getStateFromProblemInstance(instance);
    auto solution = IDDFS(initState, MAX_DEPTH);
    printFoundSolution(solution, "IDDFS");
}

int getManhattanDistanceFromFinalState(const matrix &state, const matrix &finalState) {
    pair<int, int> posFinal[9];
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            posFinal[finalState[i][j]] = {i, j};

    int dist = 0;
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) {
            auto pos = posFinal[state[i][j]];
            dist = dist + abs(i - pos.first) + abs(j - pos.second);
        }
    return dist;
}

int getEuclideanDistanceFromFinalState(const matrix &state, const matrix &finalState) {
    pair<int, int> posFinal[9];
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            posFinal[finalState[i][j]] = {i, j};
    int dist = 0;
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) {
            auto pos = posFinal[state[i][j]];
            dist += (i - pos.first) * (i - pos.first)
                    + (j - pos.second) * (j - pos.second);
        }
    return dist;
}

int getHammingDistanceFromFinalState(const matrix &state, const matrix &finalState) {
    int dist = 0;
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            if (state[i][j] != finalState[i][j])
                dist++;
    return dist;
}

// The final state, transitions and the execution length
tuple<State, vector<State>, int>
greedy(const State &initState, const function<bool(const State &, const State &)> &f) {
    auto start = chrono::high_resolution_clock::now();
    priority_queue<State, vector<State>, decltype(f)> q(f);
    q.push(initState);
    map<State, State> m; // distance from the initial state and previous state
    m[initState] = initState;

    while (!q.empty()) {
        auto state = q.top();
        q.pop();

        if (isFinalState(state)) {
            auto moveSequence = getMoveSequence(m, state, initState);
            auto stop = chrono::high_resolution_clock::now();
            auto duration = chrono::duration_cast<chrono::milliseconds>(stop - start);
            return make_tuple(state, moveSequence, duration.count());
        }
        auto neighbours = getReachableStates(state);
        for (const auto &it: neighbours)
            if (!m.contains(it)) {
                m[it] = state;
                q.push(it);
            }
    }
    throw exception();
}

void printSolutionForGreedy(const std::span<const int, 9> instance) {
    auto initState =
            getStateFromProblemInstance(instance);

    auto hamming = [](const State &state1, const State &state2) -> bool {
        int dist1 = 10000, dist2 = 10000;
        for (const auto &it: solutions) {
            dist1 = min(dist1, getHammingDistanceFromFinalState(state1.m, it));
            dist2 = min(dist2, getHammingDistanceFromFinalState(state2.m, it));
        }
        return dist1 > dist2;
    };

    auto euclidean = [](const State &state1, const State &state2) -> bool {
        int dist1 = 10000, dist2 = 10000;
        for (const auto &it: solutions) {
            dist1 = min(dist1, getEuclideanDistanceFromFinalState(state1.m, it));
            dist2 = min(dist2, getEuclideanDistanceFromFinalState(state2.m, it));
        }
        return dist1 > dist2;
    };

    auto manhattan = [](const State &state1, const State &state2) -> bool {
        int dist1 = 10000, dist2 = 10000;
        for (const auto &it: solutions) {
            dist1 = min(dist1, getManhattanDistanceFromFinalState(state1.m, it));
            dist2 = min(dist2, getManhattanDistanceFromFinalState(state2.m, it));
        }
        return dist1 > dist2;
    };

    auto solutionHamming = greedy(initState, hamming);
    printFoundSolution(solutionHamming, "Greedy Hamming distance");

    auto solutionEuclidean = greedy(initState, euclidean);
    printFoundSolution(solutionEuclidean, "Greedy Euclidean distance ");

    auto solutionManhattan = greedy(initState, manhattan);
    printFoundSolution(solutionManhattan, "Greedy Manhattan distance");
}

int main() {
    const std::array instance1{8, 6, 7, 2, 5, 4, 0, 3, 1};
    const std::array instance2{2, 5, 3, 1, 0, 6, 4, 7, 8};
    const std::array instance3{2, 7, 5, 0, 8, 4, 3, 1, 6};

    printSolutionForGreedy(instance1);
    printSolutionForInstanceIDDFS(instance1);

    cout << "////////////////////////////////////////////////////////\n";

    printSolutionForGreedy(instance2);
    printSolutionForInstanceIDDFS(instance2);

    cout << "////////////////////////////////////////////////////////\n";

    printSolutionForGreedy(instance3);
    printSolutionForInstanceIDDFS(instance3);
    return 0;
}
