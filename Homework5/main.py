import random
import numpy as np
import matplotlib.pyplot as plt

# 7x10 matrix
# wind example: (3, 8) & left , wind 1 => (4, 7)
# the line increases by the power of the wind in the cell where you currently are
alpha = 0.03  # learning rate
miu = 1  # discount factor
epsilon = 0.01
M = 100000  # number of episodes

start_pos = (3, 0)
end_pos = (3, 7)

# for each column
wind_values = [0, 0, 0, 1, 1, 1, 2, 2, 1, 0]

# Q table contains action - value pairs Q(s, a)
# There are 7x10 states, each with around 4 actions
N = 7
C = 10
# Target g_t^m = r_t+1^m + miu * Q(s_t+1^m, a_t+1^m) -> for 1-step TD
# Target g_t^m = r_t+1^m + miu * max Q(s_t+1^m, a) -> for 1-step Q
# Off-policy since we have max

# Dictionary key -> state, all states are present at init time
# Values 0..4
directions = [(1, 0), (0, 1), (0, -1), (-1, 0)]
Q = {}


def is_valid_state(state: (int, int)) -> bool:
    return 0 <= state[0] < N and 0 <= state[1] < C


def get_next_state(state: (int, int), action: int):
    direction = directions[action]
    next_state = (state[0] + direction[0], state[1] + direction[1])

    if is_valid_state(next_state):
        wind_adjusted_line = next_state[0] + wind_values[state[1]]
        wind_adjusted_line = N - 1 if wind_adjusted_line >= N else wind_adjusted_line
        wind_adjusted = (wind_adjusted_line, next_state[1])
        return wind_adjusted
    return None


def init_q_table():
    for i in range(0, N):
        for j in range(0, C):
            Q[(i, j)] = [0] * 4
            for k in range(4):
                next_state = get_next_state((i, j), k)
                if next_state is not None:
                    Q[(i, j)][k] = 0.1
                else:
                    Q[(i, j)][k] = -10


def print_q_table():
    for i in range(0, N):
        for j in range(0, C):
            print(f"{i} {j}: {Q[(i, j)]}")


def epsilon_greedy_policy(state: (int, int)):
    if random.uniform(0, 1) < epsilon:
        return random.randint(0, 3)
    else:
        return np.argmax(Q[state])


def get_any_valid_action(state: (int, int)):
    for i in range(4):
        next_state = get_next_state(state, i)
        if next_state is not None:
            return i
    raise Exception(f"No next valid state found for state {state}")


def update_q_value(state, action, next_state, reward):
    global Q

    assert next_state is not None

    Q[state][action] += alpha * (reward + miu * max(Q[next_state]) - Q[state][action])


def plot_rewards_per_episode(rewards_per_episode, max_episode):
    plt.plot(range(1, max_episode + 2), rewards_per_episode)
    plt.xlabel('Episode Number')
    plt.ylabel('Total Reward')
    plt.title('Convergence: Rewards vs Episode number')
    plt.show()


def plot_change_in_q(difference_per_episode, max_episode):
    plt.plot(range(1, max_episode + 2), difference_per_episode)
    plt.xlabel('Episode Number')
    plt.ylabel('Difference from previous episode')
    plt.title('Convergence: Difference vs Episode number')
    plt.show()


def compute_q_table():
    global Q

    rewards_per_episode = []
    difference_per_episode = []

    episode = 0
    for episode in range(M):
        prev_q_values = np.array(list(Q.values())).flatten()
        state = start_pos

        total_reward = 0

        # Run episode until reaching the goal
        while state != end_pos:
            action = epsilon_greedy_policy(state)
            next_state = get_next_state(state, action)
            if next_state is None:
                action = np.argmax(Q[state])
                next_state = get_next_state(state, action)

            assert next_state is not None

            reward = 10 if next_state == end_pos else -1
            total_reward += reward  # Accumulate the reward

            update_q_value(state, action, next_state, reward)

            state = next_state

        rewards_per_episode.append(total_reward)

        new_q_values = np.array(list(Q.values())).flatten()
        max_dif = np.max(np.abs(new_q_values - prev_q_values))
        difference_per_episode.append(max_dif)
        if max_dif < 0.001:
            break

    plot_rewards_per_episode(rewards_per_episode, episode)
    plot_change_in_q(difference_per_episode, episode)


def extract_policy() -> dict:
    global Q

    policy = {}
    for state in Q:
        best_action = np.argmax(Q[state])
        policy[state] = best_action
    return policy


def test_policy(policy: dict):
    state = start_pos
    cnt_moves = 0
    while state != end_pos:
        action = policy[state]
        state = get_next_state(state, action)
        cnt_moves += 1
        if cnt_moves > 100:
            print("Not good")
            exit(0)

    print(f"Number of moves is {cnt_moves}")


def print_policy(policy: dict):
    for (key, val) in policy.items():
        print(f"{key}: {directions[val]}")


def main():
    init_q_table()
    compute_q_table()
    # print_q_table()
    policy = extract_policy()
    print_policy(policy)
    test_policy(policy)


if __name__ == '__main__':
    main()
