#include <bits/stdc++.h>


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

    void applyTransition(const Transition &T) {
        std::swap(m[T.from.first][T.from.second],
                  m[T.to.first][T.to.second]);
    }

    bool operator==(const State &rhs) const {
        return m == rhs.m && lastMoved == rhs.lastMoved;
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

[[nodiscard]] constexpr State getStateFromProblemInstance(const std::span<int, 9> instance) {
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
    const std::array<int, 4> dx = {1, -1, 0, 0};
    const std::array<int, 4> dy = {0, 0, 1, -1};
    for (int i = 0; i < 4; i++) {
        int newX = zeroPos.first + dx[i], newY = zeroPos.second + dy[i];
        auto transition = Transition{.from = zeroPos, .to = {newX, newY}};
        if (isValidTransition(s, transition))
            v.push_back(getNewState(s, transition));
    }
    return v;
}

[[nodiscard]] bool isNoneState(const State &state) {
    for (const auto &row: state.m)
        for (const auto el: row)
            if (el != 0)
                return false;
    return true;
}

[[nodiscard]] State getNoneState() {
    return State{};
}

///////////////////////////////////////// 4

std::unordered_set<State> visited;

State limitedDepthDFS(const State &state, int depth) {
    if (isFinalState(state))
        return state;
    if (depth == 0)
        return getNoneState();
    visited.insert(state);
    auto neighbours = getReachableStates(state);
    for (const auto &it: neighbours)
        if (!visited.contains(it)) {
            auto res = limitedDepthDFS(it, depth - 1);
            if (!isNoneState(res))
                return res;
        }
    return getNoneState();
}

State IDDFS(const State &initState, int maxDepth) {
    for (int depth = 0; depth < maxDepth; depth++) {
        visited.clear();
        auto sol = limitedDepthDFS(initState, depth);
        if (!isNoneState(sol))
            return sol;
    }
    return getNoneState();
}

void printSolutionForInstance(std::span<int, 9> instance) {
    const static int MAX_DEPTH = 10'000;
    auto initState =
            getStateFromProblemInstance(instance);
    auto solution = IDDFS(initState, MAX_DEPTH);
    if (!isNoneState(solution))
        printSolution(solution.m);
    else std::cout << "Did not find solution\n";
}

int main() {
    std::array instance1{8, 6, 7, 2, 5, 4, 0, 3, 1};
    std::array instance2{2, 5, 3, 1, 0, 6, 4, 7, 8};
    std::array instance3{2, 7, 5, 0, 8, 4, 3, 1, 6};

    printSolutionForInstance(instance1);
    printSolutionForInstance(instance2);
    printSolutionForInstance(instance3);


    return 0;
}
