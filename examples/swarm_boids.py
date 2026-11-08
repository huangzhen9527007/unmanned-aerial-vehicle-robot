#!/usr/bin/env python3
"""基于 Boids 模型的无人机集群编队仿真示例。

本脚本只使用 Python 标准库，实现 Reynolds 提出的三条行为规则：

* 分离（separation）：避免与近邻碰撞；
* 对齐（alignment）：与近邻速度方向趋于一致；
* 聚合（cohesion）：向近邻的平均位置靠拢。

在此基础上加入简单的无人机运动约束（最大速度、最大加速度），
并在每 20 步输出一次集群统计量，用于观察编队是否收敛、是否出现
数值发散。脚本可作为后续集群控制研究的最小可运行骨架。

运行示例::

    python3 examples/swarm_boids.py --n 10 --steps 40
"""

from __future__ import annotations

import argparse
import math
import random
from typing import List, Sequence, Tuple

Vec3 = List[float]

# ---------------------------------------------------------------------------
# 仿真参数（默认值，可按研究需要在代码中调整）
# ---------------------------------------------------------------------------

DT = 0.1  # 仿真步长（秒）
SEED = 42  # 默认随机种子，保证结果可复现

SEPARATION_RADIUS = 3.0  # 分离规则的作用半径（米）
NEIGHBOR_RADIUS = 10.0  # 对齐 / 聚合规则的作用半径（米）
SEPARATION_WEIGHT = 1.5  # 分离规则权重
ALIGNMENT_WEIGHT = 1.0  # 对齐规则权重
COHESION_WEIGHT = 1.0  # 聚合规则权重

MAX_SPEED = 5.0  # 最大速度（米 / 秒）
MAX_ACCELERATION = 2.0  # 最大加速度（米 / 秒^2）

WORLD_HALF_SIZE = 30.0  # 仿真空间半边长（米），初始位置在其内均匀采样
BOUNDARY_MARGIN = 5.0  # 距离边界小于该值时施加回中转向
BOUNDARY_WEIGHT = 2.0  # 回中转向权重


# ---------------------------------------------------------------------------
# 向量运算：全部用长度为 3 的 list 表示，避免引入 numpy 依赖
# ---------------------------------------------------------------------------


def vec_add(a: Sequence[float], b: Sequence[float]) -> Vec3:
    return [a[0] + b[0], a[1] + b[1], a[2] + b[2]]


def vec_sub(a: Sequence[float], b: Sequence[float]) -> Vec3:
    return [a[0] - b[0], a[1] - b[1], a[2] - b[2]]


def vec_scale(a: Sequence[float], s: float) -> Vec3:
    return [a[0] * s, a[1] * s, a[2] * s]


def vec_norm(a: Sequence[float]) -> float:
    return math.sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2])


def vec_normalized(a: Sequence[float]) -> Vec3:
    """返回单位向量；零向量保持为零向量。"""
    norm = vec_norm(a)
    if norm == 0.0:
        return [0.0, 0.0, 0.0]
    return [a[0] / norm, a[1] / norm, a[2] / norm]


def vec_distance(a: Sequence[float], b: Sequence[float]) -> float:
    return vec_norm(vec_sub(a, b))


def clamp_magnitude(a: Sequence[float], limit: float) -> Vec3:
    """把向量长度限制在 limit 以内，方向不变。"""
    return vec_scale(vec_normalized(a), min(vec_norm(a), limit))


# ---------------------------------------------------------------------------
# 初始化与仿真核心
# ---------------------------------------------------------------------------


def random_swarm(n: int, rng: random.Random) -> Tuple[List[Vec3], List[Vec3]]:
    """在仿真空间内随机初始化位置，并赋予一个较小的随机初速度。"""
    positions: List[Vec3] = []
    velocities: List[Vec3] = []
    for _ in range(n):
        positions.append(
            [
                rng.uniform(-WORLD_HALF_SIZE / 2.0, WORLD_HALF_SIZE / 2.0),
                rng.uniform(-WORLD_HALF_SIZE / 2.0, WORLD_HALF_SIZE / 2.0),
                rng.uniform(-WORLD_HALF_SIZE / 2.0, WORLD_HALF_SIZE / 2.0),
            ]
        )
        velocities.append(
            [
                rng.uniform(-1.0, 1.0),
                rng.uniform(-1.0, 1.0),
                rng.uniform(-1.0, 1.0),
            ]
        )
    return positions, velocities


def boundary_steering(position: Sequence[float]) -> Vec3:
    """靠近边界时施加指向空间中心的转向分量。"""
    half = WORLD_HALF_SIZE
    steer = [0.0, 0.0, 0.0]
    for axis in range(3):
        if position[axis] < -half + BOUNDARY_MARGIN:
            steer[axis] += 1.0
        elif position[axis] > half - BOUNDARY_MARGIN:
            steer[axis] -= 1.0
    return vec_normalized(steer)


def compute_accelerations(positions: Sequence[Sequence[float]],
                          velocities: Sequence[Sequence[float]]) -> List[Vec3]:
    """按 Boids 三规则 + 边界转向计算每个个体的期望加速度。"""
    n = len(positions)
    accelerations: List[Vec3] = []
    for i in range(n):
        separation = [0.0, 0.0, 0.0]
        alignment_sum = [0.0, 0.0, 0.0]
        cohesion_sum = [0.0, 0.0, 0.0]
        neighbor_count = 0

        for j in range(n):
            if i == j:
                continue
            distance = vec_distance(positions[i], positions[j])
            if distance == 0.0:
                continue

            if distance < SEPARATION_RADIUS:
                # 距离越近，排斥力越强。
                separation = vec_add(
                    separation,
                    vec_scale(vec_sub(positions[i], positions[j]), 1.0 / (distance * distance)),
                )
            if distance < NEIGHBOR_RADIUS:
                alignment_sum = vec_add(alignment_sum, velocities[j])
                cohesion_sum = vec_add(cohesion_sum, positions[j])
                neighbor_count += 1

        accel = vec_scale(vec_normalized(separation), SEPARATION_WEIGHT)

        if neighbor_count > 0:
            alignment_center = vec_scale(alignment_sum, 1.0 / neighbor_count)
            cohesion_center = vec_scale(cohesion_sum, 1.0 / neighbor_count)
            alignment_dir = vec_normalized(vec_sub(alignment_center, velocities[i]))
            cohesion_dir = vec_normalized(vec_sub(cohesion_center, positions[i]))
            accel = vec_add(accel, vec_scale(alignment_dir, ALIGNMENT_WEIGHT))
            accel = vec_add(accel, vec_scale(cohesion_dir, COHESION_WEIGHT))

        accel = vec_add(accel, vec_scale(boundary_steering(positions[i]), BOUNDARY_WEIGHT))

        # 无人机加速度受执行机构限制。
        accelerations.append(clamp_magnitude(accel, MAX_ACCELERATION))
    return accelerations


def integrate(positions: List[Vec3], velocities: List[Vec3]) -> None:
    """半隐式欧拉积分，并施加最大速度约束。"""
    accelerations = compute_accelerations(positions, velocities)
    for i in range(len(positions)):
        velocities[i] = vec_add(velocities[i], vec_scale(accelerations[i], DT))
        # 无人机最大速度约束。
        velocities[i] = clamp_magnitude(velocities[i], MAX_SPEED)
        positions[i] = vec_add(positions[i], vec_scale(velocities[i], DT))


def mean_vector(vectors: Sequence[Sequence[float]]) -> Vec3:
    total = [0.0, 0.0, 0.0]
    for vector in vectors:
        total = vec_add(total, vector)
    count = float(len(vectors))
    return [total[0] / count, total[1] / count, total[2] / count]


def mean_nearest_neighbor_distance(positions: Sequence[Sequence[float]]) -> float:
    """所有个体到其最近邻距离的平均值。"""
    if len(positions) < 2:
        return float("nan")
    nearest: List[float] = []
    for i in range(len(positions)):
        nearest.append(
            min(
                vec_distance(positions[i], positions[j])
                for j in range(len(positions))
                if j != i
            )
        )
    return sum(nearest) / len(nearest)


def assert_no_nan(positions: Sequence[Sequence[float]],
                  velocities: Sequence[Sequence[float]],
                  step: int) -> None:
    """断言仿真状态中没有 NaN / inf，一旦出现立即报错。"""
    for position in positions:
        assert all(math.isfinite(value) for value in position), (
            "位置出现非有限数值（NaN/inf），仿真在 step=%d 发散" % step
        )
    for velocity in velocities:
        assert all(math.isfinite(value) for value in velocity), (
            "速度出现非有限数值（NaN/inf），仿真在 step=%d 发散" % step
        )


def format_vec(vector: Sequence[float]) -> str:
    return "(" + ", ".join("%8.3f" % value for value in vector) + ")"


def print_statistics(step: int,
                     positions: Sequence[Sequence[float]],
                     velocities: Sequence[Sequence[float]]) -> None:
    """打印给定步的集群统计量。"""
    avg_position = mean_vector(positions)
    avg_velocity = mean_vector(velocities)
    avg_nearest = mean_nearest_neighbor_distance(positions)
    print(
        "[step %4d] 平均位置 %s  平均速度 %s  平均最近邻距离 %8.3f"
        % (step, format_vec(avg_position), format_vec(avg_velocity), avg_nearest)
    )


def run_simulation(n: int, steps: int, seed: int, stats_interval: int = 20) -> None:
    """运行仿真，每 stats_interval 步打印一次统计并断言无 NaN。"""
    rng = random.Random(seed)
    positions, velocities = random_swarm(n, rng)

    print("无人机集群 Boids 仿真：n=%d, steps=%d, dt=%.2fs, seed=%d"
          % (n, steps, DT, seed))
    print("约束：最大速度 %.2f m/s，最大加速度 %.2f m/s^2"
          % (MAX_SPEED, MAX_ACCELERATION))

    assert_no_nan(positions, velocities, 0)
    print_statistics(0, positions, velocities)

    for step in range(1, steps + 1):
        integrate(positions, velocities)
        assert_no_nan(positions, velocities, step)
        if step % stats_interval == 0:
            print_statistics(step, positions, velocities)

    avg_nearest = mean_nearest_neighbor_distance(positions)
    print("仿真完成：共 %d 步，最终平均最近邻距离 %.3f m，数值稳定。" % (steps, avg_nearest))


# ---------------------------------------------------------------------------
# 命令行入口
# ---------------------------------------------------------------------------


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="基于 Boids 模型的无人机集群编队仿真（纯标准库实现）。"
    )
    parser.add_argument(
        "--n",
        type=int,
        default=20,
        help="集群个体数量（默认 20，至少为 2）",
    )
    parser.add_argument(
        "--steps",
        type=int,
        default=100,
        help="仿真步数（默认 100）",
    )
    parser.add_argument(
        "--seed",
        type=int,
        default=SEED,
        help="随机种子（默认 %d，固定种子保证可复现）" % SEED,
    )
    args = parser.parse_args(argv)

    if args.n < 2:
        parser.error("--n 至少为 2，否则无法计算最近邻距离")
    if args.steps < 1:
        parser.error("--steps 必须为正整数")
    return args


def main(argv: Sequence[str] | None = None) -> None:
    args = parse_args(argv)
    run_simulation(n=args.n, steps=args.steps, seed=args.seed)


if __name__ == "__main__":
    main()
