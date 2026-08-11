#include "astar_planner/astar_3d.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>


namespace astar_planner
{

// ============================================================
// priority_queue里的节点
// ============================================================

struct OpenNode
{
    int index;

    double g;
    double f;
};


// priority_queue默认大顶堆，
// 我们希望f最小的节点先出来。
struct CompareOpenNode
{
    bool operator()(
        const OpenNode &a,
        const OpenNode &b) const
    {
        return a.f > b.f;
    }
};


// ============================================================
// 保存最新地图
// ============================================================

void AStar3D::setMap(
    const uav_mapping::msg::VoxelMap &map)
{
    resolution_ =
        map.resolution;

    origin_x_ =
        map.origin.x;

    origin_y_ =
        map.origin.y;

    origin_z_ =
        map.origin.z;

    size_x_ =
        static_cast<int>(map.size_x);

    size_y_ =
        static_cast<int>(map.size_y);

    size_z_ =
        static_cast<int>(map.size_z);

    map_data_ =
        map.data;


    const std::size_t expected_size =
        static_cast<std::size_t>(size_x_) *
        static_cast<std::size_t>(size_y_) *
        static_cast<std::size_t>(size_z_);


    map_received_ =
        resolution_ > 0.0 &&
        size_x_ > 0 &&
        size_y_ > 0 &&
        size_z_ > 0 &&
        map_data_.size() == expected_size;
}


// ============================================================
// 世界坐标 -> Grid
// ============================================================

bool AStar3D::worldToGrid(
    double x,
    double y,
    double z,
    int &ix,
    int &iy,
    int &iz) const
{
    ix = static_cast<int>(
        std::floor(
            (x - origin_x_) /
            resolution_));

    iy = static_cast<int>(
        std::floor(
            (y - origin_y_) /
            resolution_));

    iz = static_cast<int>(
        std::floor(
            (z - origin_z_) /
            resolution_));


    return validGrid(
        ix,
        iy,
        iz);
}


// ============================================================
// Grid -> 世界坐标
//
// 使用体素中心
// ============================================================

std::array<double, 3>
AStar3D::gridToWorld(
    int x,
    int y,
    int z) const
{
    return {
        origin_x_ +
            (x + 0.5) * resolution_,

        origin_y_ +
            (y + 0.5) * resolution_,

        origin_z_ +
            (z + 0.5) * resolution_
    };
}


// ============================================================
// Grid是否合法
// ============================================================

bool AStar3D::validGrid(
    int x,
    int y,
    int z) const
{
    return
        x >= 0 && x < size_x_ &&
        y >= 0 && y < size_y_ &&
        z >= 0 && z < size_z_;
}


// ============================================================
// 3D -> 1D
// ============================================================

int AStar3D::toIndex(
    int x,
    int y,
    int z) const
{
    return
        x +
        size_x_ *
        (y + size_y_ * z);
}


// ============================================================
// 1D -> 3D
// ============================================================

void AStar3D::indexToGrid(
    int index,
    int &x,
    int &y,
    int &z) const
{
    z =
        index /
        (size_x_ * size_y_);


    const int remaining =
        index %
        (size_x_ * size_y_);


    y =
        remaining /
        size_x_;


    x =
        remaining %
        size_x_;
}


// ============================================================
// 获取地图状态
// ============================================================

int8_t AStar3D::getState(
    int x,
    int y,
    int z) const
{
    if (!validGrid(x, y, z))
        return 100;


    return map_data_[
        toIndex(x, y, z)];
}


// ============================================================
// 是否允许进入
// ============================================================

bool AStar3D::isTraversable(
    int x,
    int y,
    int z) const
{
    if (!validGrid(x, y, z))
        return false;


    const int8_t state =
        getState(x, y, z);


    // OCCUPIED
    if (state >= 100)
        return false;


    // UNKNOWN
    if (state < 0)
        return allow_unknown_;


    // FREE
    return true;
}


// ============================================================
// 防止对角穿墙
//
// 举例，二维情况下：
//
// ■ │ destination
// ──┼──
// S │ ■
//
// 单纯检查destination可能允许斜着从两个障碍之间穿过去。
// 所以对角移动时，把该次移动涉及到的相邻voxel全部检查。
// ============================================================

bool AStar3D::checkMove(
    int x,
    int y,
    int z,
    int dx,
    int dy,
    int dz,
    bool &passes_unknown) const
{
    passes_unknown =
        false;


    const int max_x =
        (dx == 0) ? 0 : 1;

    const int max_y =
        (dy == 0) ? 0 : 1;

    const int max_z =
        (dz == 0) ? 0 : 1;


    for (int bx = 0;
         bx <= max_x;
         ++bx)
    {
        for (int by = 0;
             by <= max_y;
             ++by)
        {
            for (int bz = 0;
                 bz <= max_z;
                 ++bz)
            {
                const int ox =
                    bx ? dx : 0;

                const int oy =
                    by ? dy : 0;

                const int oz =
                    bz ? dz : 0;


                // 起点自身无需检查
                if (ox == 0 &&
                    oy == 0 &&
                    oz == 0)
                {
                    continue;
                }


                const int nx =
                    x + ox;

                const int ny =
                    y + oy;

                const int nz =
                    z + oz;


                if (!isTraversable(
                        nx,
                        ny,
                        nz))
                {
                    return false;
                }


                if (getState(
                        nx,
                        ny,
                        nz) < 0)
                {
                    passes_unknown =
                        true;
                }
            }
        }
    }


    return true;
}


// ============================================================
// 欧氏距离启发函数
// ============================================================

double AStar3D::heuristic(
    int x,
    int y,
    int z,
    int gx,
    int gy,
    int gz) const
{
    const double dx =
        (x - gx) *
        resolution_;

    const double dy =
        (y - gy) *
        resolution_;

    const double dz =
        (z - gz) *
        resolution_;


    return std::sqrt(
        dx * dx +
        dy * dy +
        dz * dz);
}


// ============================================================
// 3D A*
// ============================================================

bool AStar3D::search(
    const std::array<double, 3> &start,
    const std::array<double, 3> &goal,
    std::vector<std::array<double, 3>> &path,
    std::string &message)
{
    path.clear();


    if (!map_received_)
    {
        message =
            "还没有收到有效VoxelMap";

        return false;
    }


    // --------------------------------------------------------
    // 起点/终点转Grid
    // --------------------------------------------------------

    int sx, sy, sz;
    int gx, gy, gz;


    if (!worldToGrid(
            start[0],
            start[1],
            start[2],
            sx,
            sy,
            sz))
    {
        message =
            "起点超出地图范围";

        return false;
    }


    if (!worldToGrid(
            goal[0],
            goal[1],
            goal[2],
            gx,
            gy,
            gz))
    {
        message =
            "终点超出地图范围";

        return false;
    }


    if (!isTraversable(
            sx,
            sy,
            sz))
    {
        message =
            "起点位于障碍物内";

        return false;
    }


    if (!isTraversable(
            gx,
            gy,
            gz))
    {
        message =
            "终点位于障碍物内";

        return false;
    }


    const int start_index =
        toIndex(
            sx,
            sy,
            sz);

    const int goal_index =
        toIndex(
            gx,
            gy,
            gz);


    // --------------------------------------------------------
    // 搜索数据
    // --------------------------------------------------------

    const std::size_t total_size =
        map_data_.size();


    const double INF =
        std::numeric_limits<double>::infinity();


    // 每个节点当前最小g值
    std::vector<double>
        g_score(
            total_size,
            INF);


    // 记录每个节点的父节点
    std::vector<int>
        parent(
            total_size,
            -1);


    // CLOSED集合
    std::vector<uint8_t>
        closed(
            total_size,
            0);


    std::priority_queue<
        OpenNode,
        std::vector<OpenNode>,
        CompareOpenNode>
        open;


    g_score[start_index] =
        0.0;


    open.push(
        {
            start_index,
            0.0,
            heuristic(
                sx,
                sy,
                sz,
                gx,
                gy,
                gz)
        });


    bool found =
        false;


    int expanded_nodes =
        0;


    // --------------------------------------------------------
    // A*主循环
    // --------------------------------------------------------

    while (!open.empty())
    {
        const OpenNode current =
            open.top();

        open.pop();


        // 已经处理过
        if (closed[current.index])
            continue;


        // 这是priority_queue中的旧记录
        if (current.g >
            g_score[current.index] +
            1e-9)
        {
            continue;
        }


        closed[current.index] =
            1;


        ++expanded_nodes;


        // 到达终点
        if (current.index ==
            goal_index)
        {
            found =
                true;

            break;
        }


        int cx;
        int cy;
        int cz;


        indexToGrid(
            current.index,
            cx,
            cy,
            cz);


        // ----------------------------------------------------
        // 26邻域
        // ----------------------------------------------------

        for (int dx = -1;
             dx <= 1;
             ++dx)
        {
            for (int dy = -1;
                 dy <= 1;
                 ++dy)
            {
                for (int dz = -1;
                     dz <= 1;
                     ++dz)
                {
                    // 自己
                    if (dx == 0 &&
                        dy == 0 &&
                        dz == 0)
                    {
                        continue;
                    }


                    const int nx =
                        cx + dx;

                    const int ny =
                        cy + dy;

                    const int nz =
                        cz + dz;


                    if (!validGrid(
                            nx,
                            ny,
                            nz))
                    {
                        continue;
                    }


                    bool passes_unknown =
                        false;


                    // 同时完成：
                    // 1. 障碍物检测
                    // 2. 对角穿墙检测
                    if (!checkMove(
                            cx,
                            cy,
                            cz,
                            dx,
                            dy,
                            dz,
                            passes_unknown))
                    {
                        continue;
                    }


                    const int neighbor_index =
                        toIndex(
                            nx,
                            ny,
                            nz);


                    if (closed[
                            neighbor_index])
                    {
                        continue;
                    }


                    // 基础移动距离
                    const double move_length =
                        resolution_ *
                        std::sqrt(
                            static_cast<double>(
                                dx * dx +
                                dy * dy +
                                dz * dz));


                    // 未知空间增加代价
                    double move_cost =
                        move_length;


                    if (passes_unknown)
                    {
                        move_cost *=
                            unknown_cost_;
                    }


                    const double tentative_g =
                        g_score[
                            current.index] +
                        move_cost;


                    // 找到了更短路线
                    if (tentative_g <
                        g_score[
                            neighbor_index])
                    {
                        g_score[
                            neighbor_index] =
                            tentative_g;


                        parent[
                            neighbor_index] =
                            current.index;


                        const double h =
                            heuristic(
                                nx,
                                ny,
                                nz,
                                gx,
                                gy,
                                gz);


                        open.push(
                            {
                                neighbor_index,
                                tentative_g,
                                tentative_g + h
                            });
                    }
                }
            }
        }
    }


    // --------------------------------------------------------
    // 搜索失败
    // --------------------------------------------------------

    if (!found)
    {
        message =
            "A*搜索失败，没有可达路径";

        return false;
    }


    // --------------------------------------------------------
    // 回溯路径
    // --------------------------------------------------------

    std::vector<int>
        index_path;


    int current_index =
        goal_index;


    while (current_index != -1)
    {
        index_path.push_back(
            current_index);


        if (current_index ==
            start_index)
        {
            break;
        }


        current_index =
            parent[
                current_index];
    }


    if (index_path.empty() ||
        index_path.back() !=
            start_index)
    {
        message =
            "路径回溯失败";

        return false;
    }


    std::reverse(
        index_path.begin(),
        index_path.end());


    // --------------------------------------------------------
    // 转成真实世界坐标
    // --------------------------------------------------------

    path.reserve(
        index_path.size());


    for (const int index :
         index_path)
    {
        int x;
        int y;
        int z;


        indexToGrid(
            index,
            x,
            y,
            z);


        path.push_back(
            gridToWorld(
                x,
                y,
                z));
    }


    message =
        "A*成功，扩展节点数: " +
        std::to_string(
            expanded_nodes) +
        "，路径点数: " +
        std::to_string(
            path.size());


    return true;
}

} // namespace astar_planner