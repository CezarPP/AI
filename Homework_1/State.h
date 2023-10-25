#pragma once

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