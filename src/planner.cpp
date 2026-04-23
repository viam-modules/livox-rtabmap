#include "planner.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <queue>
#include <vector>

namespace {

bool isFree(const cv::Mat &grid, int r, int c, bool allow_unknown) {
    if (r < 0 || r >= grid.rows || c < 0 || c >= grid.cols) return false;
    int8_t v = grid.at<int8_t>(r, c);
    if (v == 0)        return true;
    if (v < 0 && allow_unknown) return true;
    return false;
}

// Expand obstacle cells outward by kernel_cells (disk shape)
cv::Mat inflate(const cv::Mat &grid, int kernel_cells, bool allow_unknown) {
    cv::Mat out = grid.clone();
    int k = kernel_cells;
    for (int r = 0; r < grid.rows; r++) {
        for (int c = 0; c < grid.cols; c++) {
            if (grid.at<int8_t>(r, c) <= 0) continue;  // only expand occupied cells
            for (int dr = -k; dr <= k; dr++) {
                for (int dc = -k; dc <= k; dc++) {
                    if (dr*dr + dc*dc > k*k) continue;
                    int nr = r+dr, nc = c+dc;
                    if (nr < 0 || nr >= grid.rows || nc < 0 || nc >= grid.cols) continue;
                    if (out.at<int8_t>(nr, nc) == 0 ||
                        (out.at<int8_t>(nr, nc) < 0 && !allow_unknown)) {
                        out.at<int8_t>(nr, nc) = 100;
                    }
                }
            }
        }
    }
    return out;
}

// Bresenham line-of-sight check on the inflated grid
bool hasLOS(const cv::Mat &grid, int r0, int c0, int r1, int c1, bool allow_unknown) {
    int dr = std::abs(r1-r0), dc = std::abs(c1-c0);
    int sr = r0 < r1 ? 1 : -1, sc = c0 < c1 ? 1 : -1;
    int err = dr - dc, r = r0, c = c0;
    while (true) {
        if (!isFree(grid, r, c, allow_unknown)) return false;
        if (r == r1 && c == c1) break;
        int e2 = 2 * err;
        if (e2 > -dc) { err -= dc; r += sr; }
        if (e2 <  dr) { err += dr; c += sc; }
    }
    return true;
}

struct Cell { int r, c; };

// Greedy line-of-sight path simplification: skip intermediate nodes where possible
std::vector<Cell> simplify(const std::vector<Cell> &path,
                            const cv::Mat &grid, bool allow_unknown) {
    if (path.size() <= 2) return path;
    std::vector<Cell> result;
    result.push_back(path.front());
    size_t i = 0;
    while (i < path.size() - 1) {
        size_t j = path.size() - 1;
        while (j > i + 1) {
            if (hasLOS(grid, path[i].r, path[i].c,
                             path[j].r, path[j].c, allow_unknown)) break;
            j--;
        }
        result.push_back(path[j]);
        i = j;
    }
    return result;
}

} // namespace

std::vector<std::pair<float,float>> Planner::plan(
    const cv::Mat &grid,
    float xMin, float yMin, float cellSize,
    std::pair<float,float> start,
    std::pair<float,float> goal,
    const PlannerConfig &cfg)
{
    if (grid.empty() || cellSize <= 0) return {};

    int rows = grid.rows, cols = grid.cols;

    // World → grid (row 0 = yMin, col 0 = xMin)
    auto toR = [&](float wy) { return (int)((wy - yMin) / cellSize); };
    auto toC = [&](float wx) { return (int)((wx - xMin) / cellSize); };

    int sr = std::clamp(toR(start.second), 0, rows-1);
    int sc = std::clamp(toC(start.first),  0, cols-1);
    int gr = std::clamp(toR(goal.second),  0, rows-1);
    int gc = std::clamp(toC(goal.first),   0, cols-1);

    // Inflate obstacles
    int kern = std::max(1, (int)(cfg.inflation_radius / cellSize));
    cv::Mat inflated = inflate(grid, kern, cfg.allow_unknown);

    // If goal landed in an obstacle, find the nearest free cell
    if (!isFree(inflated, gr, gc, cfg.allow_unknown)) {
        bool found = false;
        for (int d = 1; d <= kern * 3 && !found; d++) {
            for (int dr = -d; dr <= d && !found; dr++) {
                for (int dc = -d; dc <= d && !found; dc++) {
                    if (std::abs(dr) != d && std::abs(dc) != d) continue;
                    int nr = gr+dr, nc = gc+dc;
                    if (nr >= 0 && nr < rows && nc >= 0 && nc < cols &&
                        isFree(inflated, nr, nc, cfg.allow_unknown)) {
                        gr = nr; gc = nc; found = true;
                    }
                }
            }
        }
        if (!found) {
            std::cerr << "[PLAN] Goal is unreachable (in obstacle)\n";
            return {};
        }
    }

    if (sr == gr && sc == gc) {
        return {{xMin + (gc + 0.5f) * cellSize, yMin + (gr + 0.5f) * cellSize}};
    }

    // A* — 8-connected grid
    const int dR[] = {-1,-1,-1, 0, 0, 1, 1, 1};
    const int dC[] = {-1, 0, 1,-1, 1,-1, 0, 1};
    const float dCost[] = {1.414f, 1.f, 1.414f, 1.f, 1.f, 1.414f, 1.f, 1.414f};

    auto idx = [&](int r, int c) { return r * cols + c; };
    int n = rows * cols;

    std::vector<float> g(n, std::numeric_limits<float>::infinity());
    std::vector<int>   parent(n, -1);
    std::vector<bool>  closed(n, false);

    auto h = [&](int r, int c) {
        float dr = (float)(r - gr), dc = (float)(c - gc);
        return std::sqrt(dr*dr + dc*dc);
    };

    using Entry = std::pair<float, int>;  // (f, flat_idx)
    std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> open;

    int si = idx(sr, sc);
    g[si] = 0.f;
    parent[si] = si;
    open.push({h(sr, sc), si});

    bool found = false;
    int goal_idx = idx(gr, gc);

    while (!open.empty()) {
        auto [f, cur] = open.top(); open.pop();
        if (closed[cur]) continue;
        closed[cur] = true;
        if (cur == goal_idx) { found = true; break; }

        int r = cur / cols, c = cur % cols;
        for (int i = 0; i < 8; i++) {
            int nr = r + dR[i], nc = c + dC[i];
            if (!isFree(inflated, nr, nc, cfg.allow_unknown)) continue;
            int ni = idx(nr, nc);
            if (closed[ni]) continue;
            float ng = g[cur] + dCost[i];
            if (ng < g[ni]) {
                g[ni] = ng;
                parent[ni] = cur;
                open.push({ng + h(nr, nc), ni});
            }
        }
    }

    if (!found) {
        std::cerr << "[PLAN] No path found\n";
        return {};
    }

    // Reconstruct raw path
    std::vector<Cell> raw;
    for (int cur = goal_idx; cur != si; ) {
        raw.push_back({cur / cols, cur % cols});
        int par = parent[cur];
        if (par == cur) break;
        cur = par;
    }
    raw.push_back({sr, sc});
    std::reverse(raw.begin(), raw.end());

    // Simplify via line-of-sight
    auto simplified = simplify(raw, inflated, cfg.allow_unknown);

    // Grid → world
    std::vector<std::pair<float,float>> waypoints;
    waypoints.reserve(simplified.size());
    for (const auto &cell : simplified) {
        waypoints.push_back({
            xMin + (cell.c + 0.5f) * cellSize,
            yMin + (cell.r + 0.5f) * cellSize
        });
    }

    std::cout << "[PLAN] Path: " << raw.size() << " cells → "
              << simplified.size() << " waypoints after simplification\n";
    return waypoints;
}
